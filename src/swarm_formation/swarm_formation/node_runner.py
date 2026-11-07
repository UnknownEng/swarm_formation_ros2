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

"""Shared entry-point helper: spin a node without hiding crashes."""

import sys
import traceback
from typing import Callable, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def run_node(node_factory: Callable[[], Node], args: Optional[List[str]] = None) -> None:
    """
    Initialize rclpy, spin the node, and shut down cleanly.

    Ctrl-C and an external shutdown (e.g. ``ros2 launch`` ending) exit quietly. Any other
    exception is logged with its full traceback and the process exits with status 1, so a
    crashed node is visible in the launch output and to scripts checking exit codes.
    """
    rclpy.init(args=args)
    node: Optional[Node] = None
    exit_code = 0
    try:
        node = node_factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902 - report every crash, then exit non-zero
        # An exception raised after the context started shutting down (e.g. SIGINT from
        # ros2 launch interrupting a message take) is a normal exit, not a crash.
        if rclpy.ok():
            exit_code = 1
            message = f'Unhandled exception, node is shutting down:\n{traceback.format_exc()}'
            if node is not None:
                node.get_logger().fatal(message)
            else:
                print(message, file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    if exit_code:
        sys.exit(exit_code)
