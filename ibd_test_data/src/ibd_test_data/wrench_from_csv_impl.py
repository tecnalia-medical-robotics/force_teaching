#!/usr/bin/env python
"""
@package ibd_test_data
@file wrench_from_csv_impl.py
@author Anthony Remazeilles
@brief Package containing reference data for initial dev

Copyright (C) {packageCopyright}
https://www.gnu.org/licenses/gpl.txt
"""

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Empty

# protected region user include package begin #
import csv
import numpy
# protected region user include package end #

class WrenchFromCsvConfig(object):
    """
    set of elements accessible through dynamic reconfigure
    autogenerated: don't touch this class
    """
    def __init__(self):
        self.csv_file = "Undef"
        self.inc = 1
        pass

    def __str__(self):
        msg = "Instance of WrenchFromCsvConfig class: {"
        msg += "csv_file: {} ".format(self.csv_file)
        msg += "inc: {} ".format(self.inc)
        msg += "}"
        return msg

class WrenchFromCsvData(object):
    """
    set of input / output handled through the update methods
    autogenerated: don't touch this class
    """
    def __init__(self):
        """
        Definition of the WrenchFromCsvData attributes
        """
        # output data
        self.out_wrench = WrenchStamped()
        self.out_wrench_active = bool()
        self.out_looping = Empty()
        self.out_looping_active = bool()
        pass

    def __str__(self):
        msg = "Instance of WrenchFromCsvData class: \n {"
        msg += "out_wrench: {} \n".format(self.out_wrench_active)
        msg += "out_wrench_active: {} \n".format(self.out_wrench_active)
        msg += "out_looping: {} \n".format(self.out_looping_active)
        msg += "out_looping_active: {} \n".format(self.out_looping_active)
        msg += "}"
        return msg

class WrenchFromCsvPassthrough(object):
    """
    set of passthrough elements slightly violating interface / implementation separation
    Autogenerated: don't touch this class
    """
    def __init__(self):
        """ Class to contain variable breaking the interface separation
        """
        pass

class WrenchFromCsvImplementation(object):
    """
    Class to contain Developer implementation.
    """
    def __init__(self):
        """
        Definition and initialisation of class attributes
        """
        self.passthrough = WrenchFromCsvPassthrough()

        # protected region user member variables begin #
        # list of wrenches read
        self.wrenches = list()
        # id of the last wrench published
        self.id_wrench = -1
        # protected region user member variables end #

    def configure(self, config):
        """
        @brief configuration of the implementation
        @param      self The object
        @param      config set of configuration parameters
        @return True on success
        """
        # protected region user configure begin #
        rospy.loginfo("opening file {}".format(config.csv_file))

        try:
            with open(config.csv_file, 'rb') as file_handler:
                reader = csv.reader(file_handler, delimiter=";")
                # skip the header
                next(reader, None)
                for row in reader:
                    row_array = numpy.asarray(row)
                    row_array[row_array == ''] = '0'
                    # todo: set it as a configuration parameter
                    # data = row_array[13:19]
                    data = row_array[19:25]

                    wrench_stamped = WrenchStamped()
                    wrench_stamped.header.frame_id = "force_sensor"

                    # rospy.loginfo("Read: {}".format(data))
                    wrench_stamped.wrench.force.x = float(data[0])
                    wrench_stamped.wrench.force.y = float(data[1])
                    wrench_stamped.wrench.force.z = float(data[2])
                    wrench_stamped.wrench.torque.x = float(data[3])
                    wrench_stamped.wrench.torque.y = float(data[4])
                    wrench_stamped.wrench.torque.z = float(data[5])
                    self.wrenches.append(wrench_stamped)

        except IOError as error:
            rospy.logerr("Prb while loading file")
            rospy.logerr("Error: {}".format(error))
            return False

        rospy.loginfo("Loaded {} wrenches".format(len(self.wrenches)))
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
        # rospy.loginfo("Check : inc is {} ".format(config.inc))
        data.out_looping_active = False
        self.id_wrench += config.inc
        if self.id_wrench >= len(self.wrenches):
            rospy.loginfo("Published all wrenches, looping at next iteration")
            self.id_wrench = - config.inc
            data.out_wrench_active = False
            data.out_looping_active = True
            print "Adding a short sleep"
            rospy.sleep(1)
            print "Done"
            return
        wrench = self.wrenches[self.id_wrench]
        wrench.header.stamp = rospy.get_rostime()
        data.out_wrench = wrench
        # protected region user update end #


    # protected region user additional functions begin #
    # protected region user additional functions end #
