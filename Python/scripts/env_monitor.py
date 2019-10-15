#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import sys
import glob
import serial
import time
import logging
import matplotlib.pyplot as plt
from collections import deque


LOG_FILE = 'myapp.log'
UPDATE_DELAY = 0.5           # [s]


class UpdatablePlot:

    def __init__(self, *labels):
        self.x_data = deque()
        self.y_data = []
        self.labels = list(labels)
        self.curves_num = len(self.labels)

        # trying to use interactive mode
        plt.ion()
        self.fig = plt.figure()
        self.axes = self.fig.add_subplot(111)        # TODO: various subplots support

        self.curves = [None] * self.curves_num
        for idx, label in enumerate(labels):
            self.y_data.append(deque())
            self.curves[idx], = self.axes.plot(self.x_data, self.y_data[idx], label=label)
        self.axes.set_autoscaley_on(True)
        self.fig.legend(loc='upper left')
        self.fig.tight_layout()

    def add_point(self, x_val, *y_vals):
        assert len(y_vals) == self.curves_num, \
            ('Wrong number of Y values. Expected {}, found {}.'
             ''.format(self.curves_num, len(y_vals)))
        self.x_data.append(x_val)
        for idx, y in enumerate(y_vals):
            self.y_data[idx].append(y)
        self.update_plot()

    def update_plot(self):
        for idx, curve in enumerate(self.curves):
            curve.set_xdata(self.x_data)
            curve.set_ydata(self.y_data[idx])
        self.axes.set_xlim(self.x_data[0], self.x_data[-1] + 1)
        self.axes.relim()
        self.axes.autoscale_view()  # rescale the y-axis
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def list_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def user_select():
    """ Lists serial port names and asks user to choose port

        :returns:
            port name selected by user
    """
    print("Available COM ports:")
    port_list = list_ports()
    for idx, port in enumerate(port_list):
        print("[{}] {}".format(idx, port))
    print("Please select port by index >>>", end="")
    select = sys.stdin.readline()
    selected_port = port_list[int(select)]
    print(selected_port + " selected!")
    return selected_port


def device_init(ser_dev):
    """Check serial device ID.
    Turn off device "update by time" mode.

    :raises EnvironmentError:
        On unsupported or unknown device

    :param ser_dev:  serial.Serial instance
    :return:         None
    """
    ser_dev.write(b'IDN?')
    time.sleep(1)
    response = ser_dev.readline()
    print("Device ID = '{}'".format(response.strip()))
    assert response.startswith("Arduino"), "Unsupported device!"
    ser_dev.write(b'EXT!')
    time.sleep(1)


def device_close(ser_dev):
    ser_dev.write(b'INT!')
    time.sleep(1)
    ser_dev.close()


def request_temperature(ser_dev):
    # print("Requesting temperature...")
    ser_dev.write(b'TC?')
    # response = ser_dev.readline()
    response = ser_dev.readline()
    # print("Response = [{}]".format(response.strip()))
    return response


def check_file():
    if len(sys.argv) > 1:
        assert not os.path.isfile(sys.argv[1]), \
            "File {} already exists.".format(sys.argv[1])
        filename = os.path.abspath(sys.argv[1])
        if not os.path.isdir(os.path.dirname(filename)):
            os.makedirs(os.path.dirname(filename))
        return filename
    else:
        return LOG_FILE


def main():
    # select COM port
    port = user_select()

    # connect to COM port
    sensor = serial.Serial(port, baudrate=9600, timeout=1.0)

    # wait for arduino auto-restart after serial connection
    time.sleep(5)

    # check device and start COM connection
    device_init(sensor)

    # get log file name
    log_file = check_file()

    # create logger
    logging.basicConfig(level=logging.INFO,
                        format='%(message)s',
                        filename=log_file,
                        filemode='w')
    logger = logging.getLogger(__name__)

    # get time origin
    time0 = time.time()

    # print output header
    sys.stdout.write('{:<15}{:<11}{:<11}\n'.format('Time', 'NTC', 'Ext.T'))

    # create plot
    plot1 = UpdatablePlot('Int. NTC', 'Ext. DS')

    # main loop
    try:
        while True:
            # get actual internal and external temperature values
            int_temp, ext_temp = (float(val) for val in request_temperature(sensor).strip().split())
            # add new point to file, plot, console
            logger.info('%f %.2f %.2f', time.time() - time0, int_temp, ext_temp)
            plot1.add_point(time.time() - time0, int_temp, ext_temp)
            sys.stdout.write('\r{:f},   {:.2f},   {:.2f}'.format(time.time() - time0, int_temp, ext_temp))
            sys.stdout.flush()
            # refresh rate correction
            time.sleep(UPDATE_DELAY)
    except KeyboardInterrupt:
        # turn on arduino standalone monitor mode and close connection
        device_close(sensor)
        # TODO: save plot to file
        print()
        print("Interrupted by user.")

    # TODO: keyboard interrupt handler
    # try:
    #     check_device(sensor)
    # except Exception:
    #     print('Wrong port {}. Device is not responding.'.format(port))
    #     sensor.close()
    #     raise EnvironmentError('Unsupported device')


if __name__ == '__main__':
    main()
