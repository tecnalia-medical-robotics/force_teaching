#!/usr/bin/env python
"""
@package ibd_test_data
@file bridge_wrench_action_ros.py
@author Anthony Remazeilles
@brief Package containing reference data for initial dev

Copyright (C) {packageCopyright}
https://www.gnu.org/licenses/gpl.txt
"""

import rospy
import actionlib

# ROS message & services includes
from std_msgs.msg import Empty
from force_teaching_msgs.msg import TeachIbDForceAction

# other includes
from ibd_test_data import bridge_wrench_action_impl
from copy import deepcopy

# todo set a function to write correctly the name
class BridgeWrenchActionROS(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_data_ = bridge_wrench_action_impl.BridgeWrenchActionData()
        self.component_config_ = bridge_wrench_action_impl.BridgeWrenchActionConfig()
        self.component_implementation_ = bridge_wrench_action_impl.BridgeWrenchActionImplementation()

        self.looping_ = rospy.Subscriber('looping', Empty, self.topic_callback_looping)
        # to enable action name adjustment when loading the node
        remap = rospy.get_param("~ibd_learn_remap", "ibd_learn")
        self.component_implementation_.passthrough.ac_ibd_learn = actionlib.SimpleActionClient(remap,TeachIbDForceAction)
        rospy.loginfo("Waiting for action server {}".format(remap))
        self.component_implementation_.passthrough.ac_ibd_learn.wait_for_server()

    def topic_callback_looping(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_looping = msg
        self.component_data_.in_looping_updated = True

    def configure(self):
        """
        function setting the initial configuration of the node
        """
        return self.component_implementation_.configure(self.component_config_)

    def activate_all_output(self):
        """
        activate all defined output
        """
        pass

    def set_all_input_read(self):
        """
        set related flag to state that input has been read
        """
        self.component_data_.in_looping_updated = False
        pass

    def update(self, event):
        """
        @brief update function

        @param      self The object
        @param      event The event

        @return { description_of_the_return_value }
        """
        self.activate_all_output()
        config = deepcopy(self.component_config_)
        data = deepcopy(self.component_data_)
        self.set_all_input_read()
        self.component_implementation_.update(data, config)



def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("bridge_wrench_action", anonymous=True)

    node = BridgeWrenchActionROS()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.Timer(rospy.Duration(1.0 / 1000), node.update)
    rospy.spin()
