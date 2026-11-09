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

"""The simulated sensor must stamp measurements with the time its data refers to."""

import math

import rclpy

from swarm_formation.neighbor_sensor_node import NeighborSensorNode


def test_peer_position_is_evaluated_at_the_own_pose_time() -> None:
    """Peers are interpolated to the own-pose time, and that time is the stamp."""
    if not rclpy.ok():
        rclpy.init()
    node = NeighborSensorNode()
    try:
        node.peer_poses[2] = (1.0, 0.0)
        node.peer_samples[2] = [(10.0, 1.0, 0.0), (10.02, 1.02, 0.0)]   # 1 m/s along x
        x, y = node._peer_position_at(2, 10.01)
        assert math.isclose(x, 1.01) and math.isclose(y, 0.0)
        # Extrapolation is limited to 50 ms past the newest sample
        x_far, _ = node._peer_position_at(2, 11.0)
        assert math.isclose(x_far, 1.07)

        published = []

        class Capture:
            def publish(self, msg) -> None:
                published.append(msg)

        node.pub_neighbors = Capture()
        node.has_own_pose = True
        node.own_pose_time = 12.345
        node.peer_poses = {}
        node._publish_sensor_readings()
        stamp = published[0].header.stamp
        assert math.isclose(stamp.sec + stamp.nanosec * 1e-9, 12.345, abs_tol=1e-6)
    finally:
        node.destroy_node()
