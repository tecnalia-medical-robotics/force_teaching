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
* Implement a static transformer, based on the pose of the static object, as we have it now
 * transform from the sensor to the object would be needed.
* Implement the transformation
* Check for the intial and final state analysis: how to detect stability?
 * mention it with Pierre eventually?
* list potential needs in the node_generator...


Node generator:
* implementation of the "static parameter", wrt dynamic parameters
* think of model of action-based ros node: how to better implement them.
 * this could be a pure action component, in which the update stands for the inside loop of the action?
* add a program to see files in a created package that do not come from the model
 * use argparse (https://docs.python.org/3.3/library/argparse.html) to enable the spec of files and directory to keep while making the update.