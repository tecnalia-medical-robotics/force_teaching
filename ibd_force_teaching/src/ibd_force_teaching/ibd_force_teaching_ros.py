#!/usr/bin/env python
"""
@package ibd_force_teaching
@file ibd_force_teaching_ros.py
@author Anthony Remazeilles
@brief Teaching of force magnitude during insertion by deformation

Copyright (C) {packageCopyright}
https://www.gnu.org/licenses/gpl.txt
"""

import rospy
import actionlib
from dynamic_reconfigure.server import Server
from ibd_force_teaching.cfg import ibd_force_teachingConfig

# ROS message & services includes
from geometry_msgs.msg import WrenchStamped
from force_teaching_msgs.msg import TeachIbDForceAction

# other includes
from ibd_force_teaching import ibd_force_teaching_impl
from copy import deepcopy

# todo set a function to write correctly the name
class IbdForceTeachingROS(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_data_ = ibd_force_teaching_impl.IbdForceTeachingData()
        self.component_config_ = ibd_force_teaching_impl.IbdForceTeachingConfig()
        self.component_implementation_ = ibd_force_teaching_impl.IbdForceTeachingImplementation()

        srv = Server(ibd_force_teachingConfig, self.configure_callback)
        self.wrench_ = rospy.Subscriber('wrench', WrenchStamped, self.topic_callback_wrench)
        # to enable action name adjustment when loading the node
        remap = rospy.get_param("~learn_remap", "learn")
        self.component_implementation_.passthrough.as_learn = actionlib.SimpleActionServer(remap,
                                                                                                TeachIbDForceAction,
                                                                                                execute_cb=self.component_implementation_.callback_learn,
                                                                                                auto_start=False)
        self.component_implementation_.passthrough.as_learn.start()

    def topic_callback_wrench(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_wrench = msg
        self.component_data_.in_wrench_updated = True

    def configure_callback(self, config, level):
        """
        callback on the change of parameters dynamically adjustable
        """
        self.component_config_.wrench_window = config.wrench_window
        self.component_config_.wrench_std = config.wrench_std
        return config

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

    def set_all_output_read(self):
        """
        set related flag to state that input has been read
        """
        self.component_data_.in_wrench_updated = False
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
        self.set_all_output_read()
        self.component_implementation_.update(data, config)



def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("ibd_force_teaching", anonymous=True)

    node = IbdForceTeachingROS()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.Timer(rospy.Duration(1.0 / 200), node.update)
    rospy.spin()
