# swarm_formation

ROS 2 Jazzy package for decentralized multi-robot formation control under relative
sensing, asymmetric network impairments and a control barrier function (CBF) safety layer,
simulated in Gazebo Harmonic.

All documentation lives in the workspace [README](../../README.md): architecture, node and
parameter reference, controller theory, the CBF safety layer, campaign and Gazebo results,
and build/run/reproduce instructions.

## Quick start

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch swarm_formation gazebo_swarm_launch.py headless:=false
```

Run the tests from this directory: `pytest-3 test -q`.

## License

Apache License 2.0 (see `package.xml` and the workspace `LICENSE`).
