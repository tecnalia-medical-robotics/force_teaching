#!/usr/bin/env python
"""
@package ar_signal
@file ar_window_signal.py
@author Anthony Remazeilles
@brief see README.md

Copyright Tecnalia Research and Innovation 2018
Distributed under the GNU GPL v3.
For full terms see https://www.gnu.org/licenses/gpl.txt
"""

import numpy
from termcolor import colored
import sys


class WindowSignal(object):
    """
    Template class for window-based signal analysis
    """
    def __init__(self, dim=1, size_window=50, verbose=False):
        """constructor

        Args:
            dim (int, optional): signal dimension
            size_window (int, optional): size processing window
            verbose (bool, optional): true to be verbose
        """
        # number of samples required to get representatie data
        self._size_window = size_window
        # dimension of the signal
        self._dim = dim
        # signal accumulated so far
        self._signals = []
        # last information received
        self._last_sample = None
        # numbre of samples received so far
        self._num_sample = 0
        # wether we display aditional information
        self._verbose = verbose

        self.clear()

    def clear(self):
        """Clear the data
        """
        self._signals = []
        for _ in xrange(self._dim):
            self._signals.append([])
        self._last_sample = None
        self._num_sample = 0

        self._clear()

    def _clear(self):
        """To be implemented by the derived class

        Raises:
            NotImplementedError: this class should not be instanciated
        """
        raise NotImplementedError

    def _process(self, data_array):
        """core of the sample process

        Args:
            data_array (numpy.array): sample to process

        Raises:
            NotImplementedError: this class should not be instanciated
        """
        raise NotImplementedError

    def process(self, data):
        """launch the new sample process

        Args:
            data (numpy.array): sample to process

        Returns:
            Bool: True if the condition is reached
        """
        data_array = numpy.asarray(data)

        dim, = data_array.shape
        if dim != self._dim:
            # todo should throw an exception instead.
            print colored("data shape: {} not consistent with spec {}".format(dim, self._dim), "red")
            return False

        self._last_sample = data_array
        self._num_sample += 1
        return self._process(data_array)


class StableStateViolation(WindowSignal):
    """ Used to detect transition from stable (constant) to unstable signal
    A new measurement is compared to the current mean
    If the difference if higher than the allowed one, violation is set.
    violation if for any of x in dim, x_(t) x_mean > k * x_std
    """
    def __init__(self, dim=1, size_window=50, std_factor=10, verbose=False):
        # call mother constructor
        super(StableStateViolation, self).__init__(dim, size_window, verbose)
        # mean over the window
        self._mean = None
        # std over the window
        self._std = None
        # multiplicator applied to the std for stability violation
        self._std_factor = std_factor

    def _clear(self):
        """Clear used variable
        """
        self._mean = None
        self._std = None

    def _process(self, data_array):
        """process received data
        Args:
            data_array (numpy.array): new sample
        Returns:
            Bool: wether signal is not stable anymore
        """
        is_stability_violated = False

        if self._num_sample < self._size_window:
            for i, val in enumerate(data_array):
                self._signals[i].append(val)
            return False

        if self._num_sample == self._size_window:
            # we compute the mean and std
            self._mean = numpy.mean(self._signals, axis=1)
            self._std = numpy.std(self._signals, axis=1)
            if self._verbose:
                print "[ar_signal_processing] mean : {}".format(self._mean)
                print "[ar_signal_processing] std  : {}".format(self._std)
            return False

        # todo this test could be skipped, since this is the third and last case
        if self._num_sample > self._size_window:
            deviation = numpy.abs(data_array - self._mean)

            max_violation = deviation > (self._std_factor * self._std)
            is_stability_violated = numpy.any(max_violation)

        if self._verbose and is_stability_violated:
            color = 'red' if is_stability_violated else 'blue'
            print "num_sample = {}".format(self._num_sample)
            print colored("dev : {}".format(deviation), color)
            print colored("th  : {}".format(self._std_factor * self._std), color)
            print colored("viol: {}".format(max_violation), color)
        return is_stability_violated

class UnStableStateViolation(WindowSignal):
    """ Detect when the signal gets stable
    """
    def __init__(self, dim=1, size_window=50, th_deviation=0.1, verbose=False):
        # call mother constructor
        super(UnStableStateViolation, self).__init__(dim, size_window, verbose)
        # mean in the window
        self._mean = None
        # std in the window
        self._std = None
        # maximmm deviation authorized (unique value, or a value per dim)
        self._th_deviation = th_deviation
        # deviation observed
        self._deviation = None

    def _clear(self):
        """ Clear signal
        """
        self._mean = None
        self._std = None
        self._deviation = None

    def _process(self, data_array):

        is_stability_reached = False

        #print "Processing sample {}".format(self._num_sample)

        if self._num_sample < self._size_window:
            for i, val in enumerate(data_array):
                self._signals[i].append(val)
            return False

        if self._num_sample == self._size_window:
            # we compute the mean and std
            self._mean = numpy.mean(self._signals, axis=1)
            self._std = numpy.std(self._signals, axis=1)
            if self._verbose:
                print "[ar_signal_processing] mean : {}".format(self._mean)
                print "[ar_signal_processing] std  : {}".format(self._std)
            return False

        # todo this test could be skipped, since this is the third and last case
        if self._num_sample > self._size_window:

            # deviation = numpy.abs(data_array - self._mean)
            # norm = numpy.linalg.norm(deviation)
            # self._std_violation = norm > self._th_deviation
            # self._deviation = deviation

            # if self._verbose and self._std_violation:
            #     color = 'red' if self._std_violation else 'blue'
            #     print "num_sample = {}".format(self._num_sample)
            #     print colored("dev : {}".format(deviation), color)
            #     print colored("th  : {}".format(self._th_deviation), color)
            #     print colored("viol: {}".format(norm), color)


            # in any case, we then add the new element
            for i, val in enumerate(data_array):
                self._signals[i].append(val)
                self._signals[i].pop(0)
            self._mean = numpy.mean(self._signals, axis=1)
            self._std = numpy.std(self._signals, axis=1)
            self._deviation = self._std

            is_below = self._std < self._th_deviation

            if self._verbose:
                color = 'red'
                if numpy.any(is_below):
                    color = 'blue'
                if numpy.all(is_below):
                    color = 'green'

                print colored("Sample: {}. std: {}. below: {}".format(self._num_sample,
                                                                      self._deviation,
                                                                      is_below),
                              color)

            is_stability_reached = numpy.all(is_below)

        return is_stability_reached

class MaximumNormViolation(WindowSignal):
    """
    Just checking the norm of the current value,
    and discarding any value > given threshold
    """
    def __init__(self, dim=1, th_norm=0.1, verbose=False):
        super(MaximumNormViolation, self).__init__(dim=dim, size_window=1, verbose=verbose)

        # maximum norm to raise violation message
        self._th_norm = th_norm

    def _clear(self):
        pass

    def _process(self, data_array):
        norm = numpy.linalg.norm(data_array)

        return norm > self._th_norm
