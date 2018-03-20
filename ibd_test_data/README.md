ibd_test_data
====================

# General description of the package
Package containing reference data for initial dev
<!--- protected region package descripion begin -->

The wrench information is read from a csv file, and is then published as a wrench stamped.

The csv file contains the pose and the wrnch measured by the two objects.
<!--- protected region package descripion end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/bridge_wrench_action.png" width="300px" />-->

# Node: wrench_from_csv
Update frequency: 1000 Hz.

<!--- protected region wrench_from_csv begin -->

A launch file is created to launch the data stored in a csv file.

```
roslaunch ibd_test_data pub_fake_data.launch
```

<!--- protected region wrench_from_csv end -->

## Dynamic Parameters

All dynamic parameters can be set through the command line:
```
rosrun ibd_test_data wrench_from_csv _[param_name]:=[new_value]
```
`csv_file` *(std::string, default: "Undef")*
<!--- protected region param csv_file begin -->
csv file containing wrench information to publish
<!--- protected region param csv_file end -->
`inc` *(int, default: 1)*
<!--- protected region param inc begin -->
from the wrench measure, increment applied at each publication loop
<!--- protected region param inc end -->

## Published Topics

A topic can be remapped from the command line:
```
rosrun ibd_test_data wrench_from_csv [old_name]:=[new_name]
```

`wrench` *(geometry_msgs::WrenchStamped)*
<!--- protected region publisher wrench begin -->
emulated wrench measure
<!--- protected region publisher wrench end -->
`looping` *(std_msgs::Empty)*
<!--- protected region publisher looping begin -->
indicates when is looping on the data
<!--- protected region publisher looping end -->

# Node: bridge_wrench_action
Update frequency: 1000 Hz.

<!--- protected region bridge_wrench_action begin -->
<!--- protected region bridge_wrench_action end -->

## Subscribed Topics

A topic can be remapped from the command line:
```
rosrun ibd_test_data bridge_wrench_action [old_name]:=[new_name]
```

`looping` *(std_msgs::Empty)*
<!--- protected region subscriber looping begin -->
to be informed when we are looping on the scv data
<!--- protected region subscriber looping end -->

## Action used
Any action client direaction can be readjusted at node launch:

```
rosrun ibd_test_data bridge_wrench_action _[old_name]_remap:=[new_name]
```
`ibd_learn` *(force_teaching_msgs::TeachIbDForce)*
<!--- protected region action client ibd_learn begin -->
to launch the learning of the forces
<!--- protected region action client ibd_learn end -->

