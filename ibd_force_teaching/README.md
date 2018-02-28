ibd_force_teaching
====================

# General description of the package
Teaching of force magnitude during insertion by deformation
<!--- protected region package descripion begin -->
<!--- protected region package descripion end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/ibd_force_teaching.png" width="300px" />-->

# Node: ibd_force_teaching
Update frequency: 200 Hz.

This node is using `\tf` to get transform information.

<!--- protected region ibd_force_teaching begin -->
<!--- protected region ibd_force_teaching end -->

## Dynamic Parameters

All dynamic parameters can be set through the command line:
```
rosrun ibd_force_teaching ibd_force_teaching _[param_name]:=[new_value]
```
`wrench_window` *(int, default: 10)*
<!--- protected region param wrench_window begin -->
number of samples used for detecting manipulation start or stop
<!--- protected region param wrench_window end -->
`wrench_std` *(double, default: 0.1)*
<!--- protected region param wrench_std begin -->
maximum std used to detect manipulation limits
<!--- protected region param wrench_std end -->
`force_frame` *(string, default: "/force_sensor")*
<!--- protected region param force_frame begin -->
tf name of the sensor frame
<!--- protected region param force_frame end -->
`receptacle_object_frame` *(string, default: "/static_object")*
<!--- protected region param receptacle_object_frame begin -->
tf name of the object that will receive the clip (static object in unimanual experiment
<!--- protected region param receptacle_object_frame end -->

## Subscribed Topics

A topic can be remapped from the command line:
```
rosrun ibd_force_teaching ibd_force_teaching [old_name]:=[new_name]
```

`wrench` *(geometry_msgs::WrenchStamped)*
<!--- protected region subscriber wrench begin -->
Wrench measured by the sensor
<!--- protected region subscriber wrench end -->

## Action proposed

A simple action launched can be obtained with:

```
rosrun actionlib axclient.py /do_action
```

Any action name can be readjusted at node launch:

```
rosrun ibd_force_teaching ibd_force_teaching _[old_name]:=[new_name]
```

`learn` *(force_teaching_msgs::TeachIbDForce)*
<!--- protected region action server learn begin -->
to launch the force learning
<!--- protected region action server learn end -->

