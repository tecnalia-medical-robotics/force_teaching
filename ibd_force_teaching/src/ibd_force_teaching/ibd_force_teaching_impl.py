#!/usr/bin/env python
"""
@package ibd_force_teaching
@file ibd_force_teaching_impl.py
@author Anthony Remazeilles
@brief Teaching of force magnitude during insertion by deformation

Copyright (C) {packageCopyright}
https://www.gnu.org/licenses/gpl.txt
"""

import rospy
from geometry_msgs.msg import WrenchStamped
from force_teaching_msgs.msg import TeachIbDForceFeedback, TeachIbDForceResult
import tf

# protected region user include package begin #
from copy import deepcopy
import numpy
import PyKDL
from ar_signal import ar_window_signal
# protected region user include package end #

class IbdForceTeachingConfig(object):
    """
    set of elements accessible through dynamic reconfigure
    autogenerated: don't touch this class
    """
    def __init__(self):
        self.wrench_window = 10
        self.wrench_std = 0.1
        self.force_frame = "/force_sensor"
        self.receptacle_object_frame = "/static_object"
        pass

    def __str__(self):
        msg = "Instance of IbdForceTeachingConfig class: {"
        msg += "wrench_window: {} ".format(self.wrench_window)
        msg += "wrench_std: {} ".format(self.wrench_std)
        msg += "force_frame: {} ".format(self.force_frame)
        msg += "receptacle_object_frame: {} ".format(self.receptacle_object_frame)
        msg += "}"
        return msg

class IbdForceTeachingData(object):
    """
    set of input / output handled through the update methods
    autogenerated: don't touch this class
    """
    def __init__(self):
        """
        Definition of the IbdForceTeachingData attributes
        """
        # input data
        self.in_wrench = WrenchStamped()
        self.in_wrench_updated = bool()
        pass

    def __str__(self):
        msg = "Instance of IbdForceTeachingData class: \n {"
        msg += "in_wrench: {} \n".format(self.in_wrench)
        msg += "in_wrench_updated: {} \n".format(self.in_wrench_updated)
        msg += "}"
        return msg

class IbdForceTeachingPassthrough(object):
    """
    set of passthrough elements slightly violating interface / implementation separation
    Autogenerated: don't touch this class
    """
    def __init__(self):
        """ Class to contain variable breaking the interface separation
        """
        self.tf_listen = tf.TransformListener()
        self.as_learn = None
        pass

class IbdForceTeachingImplementation(object):
    """
    Class to contain Developer implementation.
    """
    def __init__(self):
        """
        Definition and initialisation of class attributes
        """
        self.passthrough = IbdForceTeachingPassthrough()

        # protected region user member variables begin #
        # internal copy of the config variables
        self.config = None
        # last message received
        self.last_wrench = None
        # received messages during processing
        self.wrenches = list()

        # whether manipulation start is detected
        self.is_start_detected = False
        # iteration
        self.start_id = -1
        # whether manipulation end is detected
        self.is_end_detected = False
        # iteration
        self.end_id = -1
        # window process to detect the manipulation start (added force)
        self.detect_begin = None
        # window process to detect the manipulation stop (force stabilized)
        self.detect_end = None
        # todo add a timeout
        # protected region user member variables end #

    def configure(self, config):
        """
        @brief configuration of the implementation
        @param      self The object
        @param      config set of configuration parameters
        @return True on success
        """
        # protected region user configure begin #
        self.config = deepcopy(config)
        # todo: read these parameters from the config file
        self.detect_begin = ar_window_signal.StableStateViolation(dim=3,
                                                                  size_window=100,
                                                                  std_factor=10,
                                                                  verbose=True)


        self.detect_end = ar_window_signal.UnStableStateViolation(dim=3,
                                                                  size_window=100,
                                                                  th_deviation=0.01,
                                                                  verbose=True)


        return True
        # protected region user configure end #



    def update(self, data, config):
        """
        @brief { function_description }

        @param      self The object
        @param      data data handled through the ros class
        @param      config parameters handled through dyn. recon.

        @return nothing
        """
        # protected region user update begin #
        if data.in_wrench_updated:
            #rospy.loginfo("Last value: {}".format(data.in_wrench))
            self.last_wrench = data.in_wrench
            #self.wrenches.append(data.in_wrench)
            #nb_elt = len(self.wrenches)
        # protected region user update end #


    def callback_learn(self, goal):
        """
        @brief callback of service learn

        @param self The object
        @param goal(TeachIbDForce) goal provided

        @return (TeachIbDForceResponse) action output
        @warning may send some feedback during the task execution
        """

        # protected region user implementation of action callback for learn begin #
        # to send the feedback, one should use:
        # self.passthrough.as_learn.publish_feedback(feedback)
        feedback = TeachIbDForceFeedback()
        feedback.current_stage = 0
        # to contain the outcome of the task at completion
        # to send the result, one should use:
        # on suceess:
        # self.passthrough.as_learn.set_succeeded(result)
        result = TeachIbDForceResult()
        rate = rospy.Rate(200)
        rospy.loginfo("Received goal: {}".format(goal))

        self.is_start_detected = goal.is_on_contact
        self.start_id = -1
        self.is_end_detected = False
        self.end_id = -1

        min_force = None
        max_force = None

        id_cur = 0
        is_done = False
        success = False
        while not is_done:
            if self.passthrough.as_learn.is_preempt_requested():
               rospy.loginfo('Preempted action learn')
               self.passthrough.as_learn.set_preempted()
               success = False
               break

            tmp_force = self.last_wrench.wrench.force
            tmp_torque = self.last_wrench.wrench.torque

            array_force = geometry_msgs_vector3_to_array(tmp_force)
            # trying to detect the motion start
            if not self.is_start_detected and self.detect_begin.process(array_force):

                # contact detected.
                # todo: memorize the deviation observed to apply it to the next one
                self.start_id = id_cur
                self.is_start_detected = True
                feedback.current_stage = 1

                deviation = self.detect_begin._std
                rospy.loginfo("Deviation observed: {}".format(deviation))

                # we defined a minimal value, to be permissive
                th_value = 0.01

                low_values = deviation < th_value
                deviation[low_values] = th_value
                rospy.loginfo("Deviation restricted to : {}".format(deviation))

                self.detect_end.clear()
                self.detect_end._th_deviation = deviation

                # in case, we reset the stabilization detecteor.
            if self.is_start_detected and not self.is_end_detected and \
               self.detect_end.process(array_force):

               self.end_id = id_cur
               self.is_end_detected = True
               is_done = True
               success = True
               feedback.current_stage = 2

            # get the pose of the sensor relative to the object
            sensor_frame = self.last_wrench.header.frame_id
            object_frame = self.config.receptacle_object_frame
            timestamp = self.last_wrench.header.stamp

            try:
                (trans, rot) = self.passthrough.tf_listen.lookupTransform(object_frame,
                                                                          sensor_frame,
                                                                          timestamp)
                # rospy.logwarn("Transform received: \n {} \n {}".format(trans, rot))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("no tf received")
                continue

            # frame mapping points from the wrench frame to the object frame
            oFs = PyKDL.Frame()
            oFs.p = PyKDL.Vector(*trans)
            oFs.M = PyKDL.Rotation.Quaternion(*rot)

            sWrench = PyKDL.Wrench()

            sWrench.force = PyKDL.Vector(tmp_force.x, tmp_force.y, tmp_force.z)
            sWrench.torque = PyKDL.Vector(tmp_torque.x, tmp_torque.y, tmp_torque.z)

            owrench = oFs * sWrench
            # convert the frame into pykdl structure
            # move the wrench into that frame
            # make the computations

            owrench = kdl_wrench_to_array(owrench)
            if min_force is None:
                min_force = owrench
                max_force = owrench

            min_force = numpy.minimum(min_force, owrench)
            max_force = numpy.maximum(max_force, owrench)

            feedback.max_force_deformation = max(abs(min_force[0]), abs(max_force[0]))
            feedback.max_force_snap = max(abs(min_force[1]), abs(max_force[1]))

            rospy.loginfo("Min: {}".format(min_force))
            rospy.loginfo("Max: {}".format(max_force))

            # if len(self.wrenches < self.config.wrench_window):
            #     continue
            # # enough data for making process
            # # get the last window
            # # convert it into an array structure
            # # compute the related std
            # # look at how this was done in contact_detection
            # is_std_overcome = False
            # if not self.is_start_detected and is_std_overcome:
            #     # start detected
            #     self.is_start_detected = True
            #     # what to do with the id?
            #     continue
            # if self.is_start_detected and not is_std_overcome:
            #     self.is_end_detected = True
            #     is_done = True
            id_cur += 1
            self.passthrough.as_learn.publish_feedback(feedback)
            rate.sleep()

        print "End of the action !"
        result.success = success
        if success:
            result.max_force_deformation = max(abs(min_force[0]), abs(max_force[0]))
            result.max_force_snap = max(abs(min_force[1]), abs(max_force[1]))
            self.passthrough.as_learn.set_succeeded(result)
        # protected region user implementation of action callback for learn end #

    # protected region user additional functions begin #

def geometry_msgs_wrench_to_array(wrench):
    """
    convert a geometry_msgs array into an 1d array
    :param wrench: the wrench to convert
    :return: the related array
    """
    return numpy.array([wrench.force.x, wrench.force.y, wrench.force.z,
                        wrench.torque.x, wrench.torque.y, wrench.torque.z])

def geometry_msgs_vector3_to_array(vector):
    """
    convert a geometry_msgs array into an 1d array
    :param wrench: the wrench to convert
    :return: the related array
    """
    return numpy.array([vector.x, vector.y, vector.z])


def kdl_vector_to_array(vector):
    """
    Converts a C{PyKDL.Vector} with fields into a numpy array.

    @type vector: PyKDL.Vector
    @param vector: The C{PyKDL.Vector} to be converted
    @rtype: array
    @return: The resulting numpy array
    @warning taken from https://github.com/crigroup/criros/blob/master/src/criros/conversions.py
    """
    return numpy.array([vector.x(), vector.y(), vector.z()])

def kdl_wrench_to_array(kdl_wrench):
    array = numpy.zeros(6)
    array[:3] = kdl_vector_to_array(kdl_wrench.force)
    array[3:] = kdl_vector_to_array(kdl_wrench.torque)
    return array

    # protected region user additional functions end #
