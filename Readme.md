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
rosrun ibd_test_data bridge_wrench_action _ibd_learn_remap:=/learn
```

The last script, `bridge_wrench_action` detect when the whole wrench data has been published.
At that moment, it launches the action of learning.
The data reproduction (`pub_fake_data.launch`) once published everything, wait for one second, and thn resume the publication from the begining.
So that the learning is started at the begining of the data.

Looking at the data published:

![pose and wrench data][ibd_test_data/data/sarafun_insertion_by_deformation_03.png]

The two magnitudes looked at (and returned by the action) should be around 6N (`max_force_deformation`) and 14N (`max_force_snap`).



The illustrative files is generated using:
```
python ar_sarafun_ibd.py --c config/trial_deformation.yaml
```

