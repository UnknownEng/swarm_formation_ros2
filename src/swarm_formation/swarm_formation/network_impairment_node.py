# Copyright 2026 Mansoor Ahmed Rind
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

import math
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

from .network_model import (
    apply_link_overrides,
    ChannelLink,
    compute_packet_latency,
    parse_network_profile,
    randomize_links,
    should_drop_packet,
)
from .node_runner import run_node


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
        # Asymmetric channel: per-direction overrides from a YAML profile, and/or a random
        # drop rate and extra delay drawn independently for every directed link
        self.declare_parameter('network_profile', '')
        self.declare_parameter('random_link_max_drop', 0.0)
        self.declare_parameter('random_link_max_delay', 0.0)
        self.declare_parameter('random_link_seed', 0)

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

        rand_drop = float(self.get_parameter('random_link_max_drop').value)
        rand_delay = float(self.get_parameter('random_link_max_delay').value)
        if rand_drop > 0.0 or rand_delay > 0.0:
            randomize_links(
                self.links, rand_drop, rand_delay,
                random.Random(int(self.get_parameter('random_link_seed').value)),
            )
        profile = str(self.get_parameter('network_profile').value)
        if profile:
            overrides = parse_network_profile(profile)
            updated = apply_link_overrides(self.links, overrides)
            self.get_logger().info(
                f'Network profile {profile}: {len(overrides)} rule(s), {updated} link update(s)'
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
        self.latest_landmarks: Dict[int, List[ObstacleMeasurement]] = {
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

    def _apply_link_update(
        self, from_id: int, to_id: int, enabled: bool,
        drop_rate: float, delay_sec: float, jitter_sec: float,
    ) -> Tuple[bool, str]:
        """Validate and apply a link update; 0 as an endpoint means every robot."""
        values = (drop_rate, delay_sec, jitter_sec)
        if not all(math.isfinite(v) for v in values):
            return False, 'drop_rate, delay_sec and jitter_sec must be finite'
        if not 0.0 <= drop_rate <= 1.0 or delay_sec < 0.0 or jitter_sec < 0.0:
            return False, 'need 0 <= drop_rate <= 1, delay_sec >= 0, jitter_sec >= 0'
        robots = range(1, self.total_robots + 1)
        if (from_id != 0 and from_id not in robots) or (to_id != 0 and to_id not in robots):
            return False, f'robot ids must be 0 (any) or 1..{self.total_robots}'
        srcs = robots if from_id == 0 else [from_id]
        dsts = robots if to_id == 0 else [to_id]
        pairs = [(s, d) for s in srcs for d in dsts if (s, d) in self.links]
        if not pairs:
            return False, f'no link {from_id} -> {to_id}'
        for pair in pairs:
            link = self.links[pair]
            link.enabled = bool(enabled)
            link.drop_rate = float(drop_rate)
            link.delay_sec = float(delay_sec)
            link.jitter_sec = float(jitter_sec)
        return True, f'updated {len(pairs)} link(s) {from_id} -> {to_id}'

    def _handle_set_link(
        self, request: SetLinkState.Request, response: SetLinkState.Response
    ) -> SetLinkState.Response:
        """Handle runtime service request to modify link connectivity, drop, or delay."""
        response.success, response.message = self._apply_link_update(
            int(request.from_robot_id), int(request.to_robot_id), request.enabled,
            float(request.drop_rate), float(request.delay_sec), float(request.jitter_sec))
        log = self.get_logger().info if response.success else self.get_logger().warn
        log(f'set_link: {response.message}')
        return response

    def _handle_link_command(self, msg: LinkState) -> None:
        """Handle runtime topic message to update link configuration."""
        ok, text = self._apply_link_update(
            int(msg.from_robot_id), int(msg.to_robot_id), msg.enabled,
            float(msg.drop_rate), float(msg.delay_sec), float(msg.jitter_sec))
        log = self.get_logger().info if ok else self.get_logger().warn
        log(f'link_command: {text}')

    def _deliver(self, dst_id: int, n: NeighborMeasurement, deliv_time: float) -> None:
        """Store a delivered measurement unless a newer one from the same source arrived."""
        src_id = int(n.neighbor_id)
        current = self.delivered_states[dst_id].get(src_id)
        if current is not None:
            new_stamp = (n.stamp.sec, n.stamp.nanosec)
            old_stamp = (current[0].stamp.sec, current[0].stamp.nanosec)
            if new_stamp < old_stamp:
                return
        self.delivered_states[dst_id][src_id] = (n, deliv_time)
        self.total_forwarded += 1

    def _raw_neighbors_callback(self, msg: NeighborList, dst_id: int) -> None:
        """Process raw neighbor measurements through directed RF channel link model."""
        now = self.get_clock().now().nanoseconds * 1e-9

        # Obstacles are direct physical line-of-sight sensing, not RF packets
        self.latest_obstacles[dst_id] = list(msg.obstacles)
        self.latest_landmarks[dst_id] = list(msg.landmarks)

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
                self._deliver(dst_id, n, now)
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
            self._deliver(dst_id, n, deliv_time)

        # 2. Publish impaired NeighborList for each robot
        for i in range(1, self.total_robots + 1):
            msg = NeighborList()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'robot_{i}/base_link'
            msg.robot_id = i
            msg.obstacles = self.latest_obstacles[i]
            msg.landmarks = self.latest_landmarks[i]

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
    run_node(NetworkImpairmentNode, args)


if __name__ == '__main__':
    main()
