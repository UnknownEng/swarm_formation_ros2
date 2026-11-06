# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Standalone network impairment node simulating packet drop, delay, jitter, and link toggle."""

import random
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import (
    LinkState,
    NeighborList,
    NeighborMeasurement,
    NetworkStatus,
    ObstacleMeasurement,
)
from swarm_interfaces.srv import SetLinkState

from .network_model import ChannelLink, compute_packet_latency, should_drop_packet


class NetworkImpairmentNode(Node):
    """Inter-robot network communication channel simulator with configurable impairments."""

    def __init__(self) -> None:
        """Initialize parameters, communication links, services, and topic publishers."""
        super().__init__('network_impairment_node')

        self.declare_parameter('total_robots', 5)
        self.declare_parameter('default_drop_rate', 0.0)
        self.declare_parameter('default_delay_sec', 0.0)
        self.declare_parameter('default_jitter_sec', 0.0)
        self.declare_parameter('network_rate', 50.0)
        self.declare_parameter('enable_faults', False)

        self.total_robots = int(self.get_parameter('total_robots').value)
        self.default_drop = float(self.get_parameter('default_drop_rate').value)
        self.default_delay = float(self.get_parameter('default_delay_sec').value)
        self.default_jitter = float(self.get_parameter('default_jitter_sec').value)
        self.network_rate = float(self.get_parameter('network_rate').value)
        self.enable_faults = bool(self.get_parameter('enable_faults').value)

        self.rng = random.Random(1337)

        # Build mesh directed link table for all robot pairs (i != j)
        self.links: Dict[Tuple[int, int], ChannelLink] = {}
        for src in range(1, self.total_robots + 1):
            for dst in range(1, self.total_robots + 1):
                if src != dst:
                    self.links[(src, dst)] = ChannelLink(
                        from_id=src,
                        to_id=dst,
                        enabled=True,
                        drop_rate=self.default_drop,
                        delay_sec=self.default_delay,
                        jitter_sec=self.default_jitter,
                    )

        # Statistics
        self.total_forwarded = 0
        self.total_dropped = 0

        # Delayed transmission queue: list of (delivery_time, dst_id, NeighborMeasurement)
        self.delayed_queue: List[Tuple[float, int, NeighborMeasurement]] = []

        # Latest delivered state per robot:
        # dst_id -> {src_id: (NeighborMeasurement, delivery_time)}
        self.delivered_states: Dict[int, Dict[int, Tuple[NeighborMeasurement, float]]] = {
            i: {} for i in range(1, self.total_robots + 1)
        }
        self.latest_obstacles: Dict[int, List[ObstacleMeasurement]] = {
            i: [] for i in range(1, self.total_robots + 1)
        }

        # Subscriptions and Publishers for each robot
        self.raw_subs = []
        self.imp_pubs: Dict[int, rclpy.publisher.Publisher] = {}

        for i in range(1, self.total_robots + 1):
            pub = self.create_publisher(
                NeighborList, f'/robot_{i}/neighbors', 10
            )
            self.imp_pubs[i] = pub

            sub = self.create_subscription(
                NeighborList,
                f'/robot_{i}/neighbors_raw',
                lambda msg, dst_id=i: self._raw_neighbors_callback(msg, dst_id),
                10,
            )
            self.raw_subs.append(sub)

        # Runtime Link Configuration Service and Command Topic
        self.srv_set_link = self.create_service(
            SetLinkState, '/network/set_link', self._handle_set_link
        )
        self.sub_link_cmd = self.create_subscription(
            LinkState, '/network/link_command', self._handle_link_command, 10
        )
        self.pub_status = self.create_publisher(
            NetworkStatus, '/network/status', 10
        )

        dt = 1.0 / self.network_rate
        self.timer = self.create_timer(dt, self._network_dispatch_loop)

        self.get_logger().info(
            f'NetworkImpairmentNode online | Swarm: {self.total_robots} robots | '
            f'Default Drop: {self.default_drop} | Delay: {self.default_delay}s'
        )

    def _handle_set_link(
        self, request: SetLinkState.Request, response: SetLinkState.Response
    ) -> SetLinkState.Response:
        """Handle runtime service request to modify link connectivity, drop, or delay."""
        from_id = int(request.from_robot_id)
        to_id = int(request.to_robot_id)

        target_pairs = []
        if from_id == 0 and to_id == 0:
            target_pairs = list(self.links.keys())
        elif from_id == 0:
            target_pairs = [
                (src, to_id)
                for src in range(1, self.total_robots + 1)
                if src != to_id
            ]
        elif to_id == 0:
            target_pairs = [
                (from_id, dst)
                for dst in range(1, self.total_robots + 1)
                if from_id != dst
            ]
        else:
            if (from_id, to_id) in self.links:
                target_pairs = [(from_id, to_id)]

        if not target_pairs:
            response.success = False
            response.message = f'Invalid link endpoints: {from_id} -> {to_id}'
            return response

        for pair in target_pairs:
            link = self.links[pair]
            link.enabled = bool(request.enabled)
            link.drop_rate = float(request.drop_rate)
            link.delay_sec = float(request.delay_sec)
            link.jitter_sec = float(request.jitter_sec)

        response.success = True
        response.message = f'Successfully updated {len(target_pairs)} link(s).'
        self.get_logger().info(f'Link update: {response.message}')
        return response

    def _handle_link_command(self, msg: LinkState) -> None:
        """Handle runtime topic message to update link configuration."""
        pair = (int(msg.from_robot_id), int(msg.to_robot_id))
        if pair in self.links:
            link = self.links[pair]
            link.enabled = bool(msg.enabled)
            link.drop_rate = float(msg.drop_rate)
            link.delay_sec = float(msg.delay_sec)
            link.jitter_sec = float(msg.jitter_sec)
            self.get_logger().info(
                f'Link ({msg.from_robot_id}->{msg.to_robot_id}) set: '
                f'enabled={link.enabled}, drop={link.drop_rate}, delay={link.delay_sec}s'
            )

    def _raw_neighbors_callback(self, msg: NeighborList, dst_id: int) -> None:
        """Process raw neighbor measurements through directed RF channel link model."""
        now = self.get_clock().now().nanoseconds * 1e-9

        # Obstacles are direct physical line-of-sight sensing, not RF packets
        self.latest_obstacles[dst_id] = list(msg.obstacles)

        for n in msg.neighbors:
            src_id = int(n.neighbor_id)
            link = self.links.get((src_id, dst_id))
            if link is None:
                continue

            if should_drop_packet(link, self.rng):
                self.total_dropped += 1
                continue

            latency = compute_packet_latency(link, self.rng)
            if latency < 1e-4:
                # Instantaneous delivery
                self.delivered_states[dst_id][src_id] = (n, now)
                self.total_forwarded += 1
            else:
                deliv_time = now + latency
                self.delayed_queue.append((deliv_time, dst_id, n))

    def _network_dispatch_loop(self) -> None:
        """Deliver matured delayed packets and publish impaired NeighborList feeds."""
        now = self.get_clock().now().nanoseconds * 1e-9

        # 1. Drain matured packets from delayed queue
        ready_packets: List[Tuple[float, int, NeighborMeasurement]] = []
        remaining_queue: List[Tuple[float, int, NeighborMeasurement]] = []

        for item in self.delayed_queue:
            if now >= item[0]:
                ready_packets.append(item)
            else:
                remaining_queue.append(item)
        self.delayed_queue = remaining_queue

        for deliv_time, dst_id, n in ready_packets:
            src_id = int(n.neighbor_id)
            self.delivered_states[dst_id][src_id] = (n, deliv_time)
            self.total_forwarded += 1

        # 2. Publish impaired NeighborList for each robot
        for i in range(1, self.total_robots + 1):
            msg = NeighborList()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'robot_{i}/base_link'
            msg.robot_id = i
            msg.obstacles = self.latest_obstacles[i]

            # Collect active delivered neighbor states (purge stale packets older than 1.5s)
            active_neighbors: List[NeighborMeasurement] = []
            stale_threshold = 1.5

            for src_id, (n_meas, deliv_time) in list(self.delivered_states[i].items()):
                # Check if link was disabled at runtime: if link is disabled, drop measurement
                link = self.links.get((src_id, i))
                if link is not None and not link.enabled:
                    continue

                if (now - deliv_time) <= stale_threshold:
                    active_neighbors.append(n_meas)

            msg.neighbors = active_neighbors
            self.imp_pubs[i].publish(msg)

        # 3. Periodically publish network telemetry status
        self._publish_network_status()

    def _publish_network_status(self) -> None:
        """Publish status of all communication links and packet throughput."""
        status = NetworkStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.total_packets_forwarded = self.total_forwarded
        status.total_packets_dropped = self.total_dropped

        for (src, dst), link in self.links.items():
            ls = LinkState()
            ls.from_robot_id = src
            ls.to_robot_id = dst
            ls.enabled = link.enabled
            ls.drop_rate = link.drop_rate
            ls.delay_sec = link.delay_sec
            ls.jitter_sec = link.jitter_sec
            status.links.append(ls)

        self.pub_status.publish(status)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for NetworkImpairmentNode."""
    rclpy.init(args=args)
    node = NetworkImpairmentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception, BaseException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
