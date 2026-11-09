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

"""Fault injector node orchestrating robot death, sensor bias, and failures."""

from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from swarm_interfaces.msg import FaultInjection
from swarm_interfaces.srv import TriggerFault

from .fault_model import FaultConfig, is_fault_active, parse_fault_yaml
from .node_runner import run_node

SUPPORTED_FAULTS = frozenset({'robot_death', 'sensor_bias', 'intermittent_failure'})


class FaultInjectorNode(Node):
    """Executes scheduled and runtime configurable fault scenarios across the swarm."""

    def __init__(self) -> None:
        """Initialize parameters, publishers, services, and scheduled profile evaluation."""
        super().__init__('fault_injector_node')

        self.declare_parameter('fault_profile', '')
        self.declare_parameter('total_robots', 5)
        self.declare_parameter('update_rate', 20.0)

        profile_path = str(self.get_parameter('fault_profile').value)
        self.total_robots = int(self.get_parameter('total_robots').value)
        self.update_rate = float(self.get_parameter('update_rate').value)

        # Parse initial scheduled faults from profile
        self.scheduled_faults: List[FaultConfig] = []
        if profile_path:
            self.scheduled_faults = parse_fault_yaml(profile_path)
            self.get_logger().info(
                f'Loaded fault profile: {profile_path} ({len(self.scheduled_faults)} scenario(s))'
            )

        self.dynamic_faults: List[FaultConfig] = []
        self.start_time: Optional[float] = None

        # Tracking state to log transitions
        self.robot_death_state: Dict[int, bool] = {
            i: False for i in range(1, self.total_robots + 1)
        }

        # Publishers per robot
        self.pub_motor_kill: Dict[int, rclpy.publisher.Publisher] = {}
        self.pub_sensor_fault: Dict[int, rclpy.publisher.Publisher] = {}

        for i in range(1, self.total_robots + 1):
            self.pub_motor_kill[i] = self.create_publisher(
                Bool, f'/robot_{i}/motor_kill', 10
            )
            self.pub_sensor_fault[i] = self.create_publisher(
                FaultInjection, f'/robot_{i}/sensor_fault', 10
            )

        # Interactive Trigger Service
        self.srv_trigger = self.create_service(
            TriggerFault, '/fault_injector/trigger_fault', self._handle_trigger_fault
        )

        dt = 1.0 / self.update_rate
        self.timer = self.create_timer(dt, self._fault_evaluation_loop)

        self.get_logger().info(
            f'FaultInjectorNode online | Swarm: {self.total_robots} robots | '
            f'Rate: {self.update_rate}Hz'
        )

    def _handle_trigger_fault(
        self, request: TriggerFault.Request, response: TriggerFault.Response
    ) -> TriggerFault.Response:
        """Handle runtime interactive service requests to trigger or release faults."""
        r_id = int(request.robot_id)
        f_type = str(request.fault_type)
        enable = bool(request.enable)
        if not 1 <= r_id <= self.total_robots:
            response.success = False
            response.message = f'robot_id must be 1..{self.total_robots}, got {r_id}'
            return response
        if f_type not in SUPPORTED_FAULTS:
            response.success = False
            response.message = f'fault_type must be one of {sorted(SUPPORTED_FAULTS)}'
            return response

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        elapsed = now - self.start_time

        if not enable:
            # Remove matching dynamic faults
            self.dynamic_faults = [
                f for f in self.dynamic_faults
                if not (f.robot_id == r_id and f.fault_type == f_type)
            ]
            response.success = True
            response.message = f'Released dynamic fault {f_type} for Robot {r_id}'
            self.get_logger().info(response.message)
            return response

        # Add active dynamic fault
        dyn_cfg = FaultConfig(
            robot_id=r_id,
            fault_type=f_type,
            start_time=elapsed,
            end_time=elapsed + 10000.0,
            range_bias=float(request.range_bias),
            bearing_bias=float(request.bearing_bias),
            period=float(request.period) if request.period > 0.0 else 1.0,
            duty_cycle=float(request.duty_cycle) if request.duty_cycle > 0.0 else 0.5,
        )
        self.dynamic_faults.append(dyn_cfg)
        response.success = True
        response.message = f'Injected dynamic fault {f_type} for Robot {r_id}'
        self.get_logger().info(response.message)
        return response

    def _fault_evaluation_loop(self) -> None:
        """Evaluate active faults against elapsed mission time and publish actuation."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now

        elapsed = now - self.start_time
        all_faults = self.scheduled_faults + self.dynamic_faults

        for r_id in range(1, self.total_robots + 1):
            robot_faults = [f for f in all_faults if f.robot_id == r_id]

            # 1. Evaluate Robot Death (Motor / actuation incapacitation)
            death_faults = [
                f for f in robot_faults
                if f.fault_type == 'robot_death' and is_fault_active(f, elapsed)
            ]
            is_dead = len(death_faults) > 0

            if is_dead != self.robot_death_state[r_id]:
                self.robot_death_state[r_id] = is_dead
                if is_dead:
                    self.get_logger().warn(f'Robot {r_id} ENTERED FAULT DROPOUT (robot_death)!')
                else:
                    self.get_logger().info(f'Robot {r_id} RECOVERED from fault dropout.')

            kill_msg = Bool()
            kill_msg.data = is_dead
            self.pub_motor_kill[r_id].publish(kill_msg)

            # 2. Evaluate Sensor Bias and Intermittent Failures
            sensor_faults = [
                f for f in robot_faults
                if f.fault_type in ('sensor_bias', 'intermittent_failure')
                and is_fault_active(f, elapsed)
            ]

            sf_msg = FaultInjection()
            sf_msg.robot_id = r_id

            if sensor_faults:
                active_sf = sensor_faults[0]
                sf_msg.fault_type = active_sf.fault_type
                sf_msg.active = True
                sf_msg.range_bias = active_sf.range_bias
                sf_msg.bearing_bias = active_sf.bearing_bias
                sf_msg.intermittent_duty_cycle = active_sf.duty_cycle
            else:
                sf_msg.active = False

            self.pub_sensor_fault[r_id].publish(sf_msg)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for FaultInjectorNode."""
    run_node(FaultInjectorNode, args)


if __name__ == '__main__':
    main()
