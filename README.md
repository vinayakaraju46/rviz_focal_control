# RViz focal point control

Build and run the keyboard publisher in an interactive terminal:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rviz_focal_control
source install/setup.bash
ros2 run rviz_focal_control keyboard_focal_control
```

Keep keyboard focus in this terminal. Each arrow press publishes one
`geometry_msgs/msg/Vector3` on `/rviz/focal_point_offset`:

| Key | X | Y | Z |
| --- | --- | --- | --- |
| Left | 0 | +0.1 | 0 |
| Right | 0 | -0.1 | 0 |
| Up | +0.1 | 0 | 0 |
| Down | -0.1 | 0 | 0 |

Press Enter to publish a `geometry_msgs/msg/PoseStamped` directly on
`/planning/mission_planning/goal`. The keyboard node reads the latest focal
point coordinates from `/rviz/focal_point_marker` and copies X, Y, and Z into
the goal, with zero yaw (+X heading). No goal is sent until a marker arrives.
Load FocalPanel. Tracked-view focal coordinates are converted to RViz's
fixed frame, including when the Target Frame follows the vehicle.

Holding an arrow key enables 50 Hz publishing after consecutive terminal repeats.
Each message moves 0.1 m (5 m/s at 50 Hz). A single tap moves one step.
The terminal's initial repeat delay still applies. Since terminals do not report
key release, movement stops within 0.1 seconds after the last repeat; a small
amount of movement after release is possible. Enter stops repeating and sends
the latest received focal point as the goal. Press `q` or Ctrl+C to exit.
The terminal settings are restored on exit.

To change the step or remap the topic:

```bash
ros2 run rviz_focal_control keyboard_focal_control --ros-args -p step:=0.1
ros2 run rviz_focal_control keyboard_focal_control --ros-args -r /rviz/focal_point_offset:=/other_topic
```

Load this package's `FocalPanel` in RViz and select a view controller with a
`Focal Point` property (such as Orbit) to see movement. Offsets use the focal
point's X/Y axes, so their screen direction depends on the camera orientation.

If the crosshair is missing, check the status text in FocalPanel. Select
Orbit or XYOrbit in Views → Type; views without a Focal Point are unsupported.
Add a **Marker** display (not MarkerArray), enable it, set Topic to
`/rviz/focal_point_marker`, and enable the `focal_crosshair` namespace.
The panel shows the published frame, coordinates, and subscriber count.
After rebuilding, restart RViz from a terminal with `source install/setup.bash`
so it loads the rebuilt plugin.

To change movement publishing frequency while keeping the 0.1 m step:

```bash
ros2 run rviz_focal_control keyboard_focal_control --ros-args -p step:=0.1 -p publish_rate:=50.0
```

`key_timeout` defaults to `0.1` seconds. Terminal repeats must arrive faster
than this timeout for continuous movement.

Press `R` (or `r`) to stop arrow repeat and reset the focal point/crosshair to
`base_link`. This sets the view's Target Frame to `base_link` and its Focal
Point to `(0, 0, 0)`, returning the camera center to the vehicle and following
it as it moves. The frame name is sent on `/rviz/focal_point_reset`. Camera zoom and orientation stay the
same. If TF is unavailable, the focal point stays unchanged and RViz logs a
warning. Reset does not publish a navigation goal; press Enter to send one.

The focal marker is a camera-facing target reticle: two cyan rings, a segmented
outer ring, four gold aiming ticks, and a gold center cross. Its 1.56 m outer
span stays centered on the exact focal point; the goal uses the marker pose,
independent of the decorative geometry. Restart both RViz and the keyboard
node after updating the reticle.
# rviz_focal_control
