ibd_test_data
====================

# General description of the package
Package containing reference data for initial dev
<!--- protected region package descripion begin -->
<!--- protected region package descripion end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/wrench_from_csv.png" width="300px" />-->

# Node: wrench_from_csv
Update frequency: 1000 Hz.

<!--- protected region wrench_from_csv begin -->
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

