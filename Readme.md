# SARAFun force learning

This code is developed in the context of the [SARAFun][sarafun_website] project


[sarafun_website]: http://h2020sarafun.eu/

## Content
* [`ibd_force_teaching`](ibd_force_teaching/README.md): main nodes enabling the extraction of the insertion by deformation characteristics.
* [`ibd_test_data`](ibd_test_data/README.md): fake data to be used for testing the node
* `force_teaching_msgs`: specific ROS communication definition.

## Use

**Warning:**
The code is still under develpment.

**Launch the force teaching module**

```
rosrun ibd_force_teaching ibd_force_teaching
# to launch an action interface
rosrun actionlib axclient.py /learn
```

To adjust the module to a larger application, the nimimum parameters to set are the followings:
```
rosrun ibd_force_teaching ibd_force_teaching _force_frame:=/[tf_force_frame] _receptacle_object_frame:=/[tf_object_frame] wrench:=[wrench_topic_name]
```
with:

* `force_frame` : the `tf` frame related to the force sensor
* `receptacle_object_frame` : the `tf` frame related to the object into which the deformable object will be inserted.
* `wrench`: name of the topic containining the WrenchStamped measured by the force sensor.

**Launch with fake data**

This will launch the data stored in `ibd_test_data`.

```
roslaunch ibd_test_data pub_fake_data.launch
rosrun ibd_force_teaching ibd_force_teaching
# to launch an action interface
rosrun actionlib axclient.py /learn
```


The illustrative files is generated using:
```
python ar_sarafun_ibd.py --c config/trial_deformation.yaml
```




Doing:
* Check how the wrench should be transformed according to static object pose
 * we will assume pykdl is available.
   Same method will be used to get the wrench measure at the object frame.
   We assume that the object frame is well positioned.
   Eventually later on we could add parameters to set the appropriate axes for the maximum search.
 * so we need the name of the object frame, and the name of the sensor frame
 * handled with dynamic parameters again.

Todo:
* Check how to add properlly dependency on pykdl or kdl actually
* Check for the intial and final state analysis: how to detect stability?
 * mention it with Pierre eventually?
* list potential needs in the node_generator...


Node generator:
* implementation of the "static parameter", wrt dynamic parameters
* think of model of action-based ros node: how to better implement them.
 * this could be a pure action component, in which the update stands for the inside loop of the action?
* add a program to see files in a created package that do not come from the model
 * use argparse (https://docs.python.org/3.3/library/argparse.html) to enable the spec of files and directory to keep while making the update.