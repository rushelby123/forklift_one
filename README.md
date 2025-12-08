## Robot Package Template

## test
```bash
ros2 topic pub /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'base_link'
twist:
  linear:
    x: -1.5
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
"
```