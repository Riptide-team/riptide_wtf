#!/usr/bin/env python3

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from riptide_msgs.msg import Pressure, Actuators
from sensor_msgs.msg import Imu, BatteryState

import curses
import dbus
import os
import signal
import sys
from pynput import keyboard

import numpy as np
from scipy.spatial.transform import Rotation

from Window import InfoWindow, StatusWindow, TimedWindow

class RiptideWTF(Node):

    def __init__(self):
        super().__init__('riptide_wtf')

        self.duration_peremted = 10000
        
        self.barometer_subscription = self.create_subscription(Pressure, '/riptide_1/pressure_broadcaster/pressure_status', self.barometer_callback, 10)
        self.battery_card_subscription = self.create_subscription(BatteryState, '/riptide_1/battery_card_broadcaster/battery_status', self.battery_card_callback, 10)
        self.actuators_subscription = self.create_subscription(Actuators, '/riptide_1/actuators_broadcaster/actuators_status', self.actuators_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, '/riptide_1/imu_broadcaster/imu_status', self.imu_callback, 10)

        self.barometer_msg = Pressure()
        self.battery_card_msg = BatteryState()
        self.actuators_msg = Actuators()
        self.imu_msg = Imu()

        self.barometer_time = self.get_clock().now()
        self.battery_card_time = self.get_clock().now()
        self.actuators_time = self.get_clock().now()
        self.imu_time = self.get_clock().now()

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.init_ncurses()
        signal.signal(signal.SIGINT, self.sigint_handler)

        self.listener = keyboard.Listener(
            on_press=self.keyboard_press,
            on_release=self.keyboard_release
        )
        self.listener.start()

        self.current_tab = 0

    def sigint_handler(self, sig=None, frame=None):
        self.listener.stop()
        rclpy.shutdown()
        curses.endwin()
        os.system('clear')
        sys.exit(0)

    def timer_callback(self):
        self.menu_window()
        if self.current_tab == 0:
            self.host_window()
            self.daemon_window()
            self.barometer_window()
            self.battery_window()
            self.imu_window()
            self.actuators_window()

    def init_ncurses(self):
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.raw()
        curses.cbreak()
        curses.curs_set(False)
        curses.start_color()
        curses.use_default_colors()
        for i in range(0, curses.COLORS):
            curses.init_pair(i + 1, i, -1)

        self.menuWindow = InfoWindow("Menu", 3, 65, 1, 1)
        self.hostWindow = InfoWindow("Host", 3, 32, 1, 4)
        self.batteryWindow = TimedWindow("Battery", 4, 32, 1, 7)
        self.barometerWindow = TimedWindow("Barometer", 6, 32, 1, 11)
        self.actuatorsWindow = TimedWindow("Actuators", 6, 32, 1, 17)
        self.daemonWindow = StatusWindow("Daemon", 3, 32, 34, 4)
        self.imuWindow = TimedWindow("Imu", 14, 32, 34, 7)

    def host_window(self):
        self.hostWindow.window.addstr(1, 4, f"{os.uname()[1]}", curses.color_pair(255))
        self.hostWindow.refresh()

    def menu_window(self):
        self.menuWindow.window.addstr(1, 2, "␛ Quit", curses.color_pair(255))
        self.menuWindow.window.addstr(1, 15, "⭾ Change tab", curses.color_pair(255))

        color = curses.color_pair(5)
        highlight_color = curses.A_REVERSE | curses.color_pair(5)
        if (self.current_tab == 0):
            self.menuWindow.window.addstr(1, 43, "⒈ Sensors", highlight_color)
            self.menuWindow.window.addstr(1, 53, "⒉ Launcher", color)
        else:
            self.menuWindow.window.addstr(1, 43, "⒈ Sensors", color)
            self.menuWindow.window.addstr(1, 53, "⒉ Launcher", highlight_color)
        self.menuWindow.refresh()
    
    def daemon_window(self):
        # Get the system bus
        bus = dbus.SystemBus()

        # Get the service manager object
        manager = dbus.Interface(bus.get_object('org.freedesktop.systemd1', '/org/freedesktop/systemd1'), 'org.freedesktop.systemd1.Manager')

        # Get the unit object for your service
        unit = manager.LoadUnit('ros2_control.service')

        # Get the properties of the unit
        props = dbus.Interface(bus.get_object('org.freedesktop.systemd1', unit), 'org.freedesktop.DBus.Properties')
        status = props.Get('org.freedesktop.systemd1.Unit', 'ActiveState')

        color = curses.color_pair(3)
        if (status!="active"):
            color = curses.color_pair(2)
        self.daemonWindow.window.addstr(1, 3, f"• Ros2Control ({status})".ljust(28), color)
        self.daemonWindow.refresh()

    def barometer_window(self):
        self.barometerWindow.message_duration = self.duration_ms_from_times(self.barometer_time)
        if (self.barometerWindow.message_duration > self.duration_peremted):
            self.barometerWindow.expired = True

        self.barometerWindow.window.addstr(1, 2, "• pressure:", curses.color_pair(255))
        self.barometerWindow.window.addstr(1, 17, f"{self.barometer_msg.pressure:.2f}".rjust(8), curses.color_pair(255))
        self.barometerWindow.window.addstr(1, 26, "mbar", curses.color_pair(255))
        self.barometerWindow.window.addstr(2, 2, "• temperature:", curses.color_pair(255))
        self.barometerWindow.window.addstr(2, 17, f"{self.barometer_msg.temperature:.2f}".rjust(8), curses.color_pair(255))
        self.barometerWindow.window.addstr(2, 26, "°C", curses.color_pair(255))
        self.barometerWindow.window.addstr(3, 2, "• depth:", curses.color_pair(255))
        self.barometerWindow.window.addstr(3, 17, f"{self.barometer_msg.depth:.2f}".rjust(8), curses.color_pair(255))
        self.barometerWindow.window.addstr(3, 26, "m", curses.color_pair(255))
        self.barometerWindow.window.addstr(4, 2, "• altitude:", curses.color_pair(255))
        self.barometerWindow.window.addstr(4, 17, f"{self.barometer_msg.altitude:.2f}".rjust(8), curses.color_pair(255))
        self.barometerWindow.window.addstr(4, 26, "m", curses.color_pair(255))
        self.barometerWindow.refresh()

    def actuators_window(self):
        self.actuatorsWindow.message_duration = self.duration_ms_from_times(self.actuators_time)
        if (self.actuatorsWindow.message_duration > self.duration_peremted):
            self.actuatorsWindow.expired = True
        self.actuatorsWindow.window.addstr(1, 2, "• thruster:", curses.color_pair(255))
        self.actuatorsWindow.window.addstr(1, 17, f"{self.actuators_msg.thruster:.2f}".rjust(8), curses.color_pair(255))
        self.actuatorsWindow.window.addstr(1, 26, "usi", curses.color_pair(255))
        self.actuatorsWindow.window.addstr(2, 2, "• d fin:", curses.color_pair(255))
        self.actuatorsWindow.window.addstr(2, 17, f"{self.actuators_msg.d_fin*180/np.pi:.2f}".rjust(8), curses.color_pair(255))
        self.actuatorsWindow.window.addstr(2, 26, "°", curses.color_pair(255))
        self.actuatorsWindow.window.addstr(3, 2, "• p fin:", curses.color_pair(255))
        self.actuatorsWindow.window.addstr(3, 17, f"{self.actuators_msg.p_fin*180/np.pi:.2f}".rjust(8), curses.color_pair(255))
        self.actuatorsWindow.window.addstr(3, 26, "°", curses.color_pair(255))
        self.actuatorsWindow.window.addstr(4, 2, "• s fin:", curses.color_pair(255))
        self.actuatorsWindow.window.addstr(4, 17, f"{self.actuators_msg.s_fin*180/np.pi:.2f}".rjust(8), curses.color_pair(255))
        self.actuatorsWindow.window.addstr(4, 26, "°", curses.color_pair(255))
        self.actuatorsWindow.refresh()

    def battery_window(self):
        self.batteryWindow.message_duration = self.duration_ms_from_times(self.battery_card_time)
        if (self.batteryWindow.message_duration > self.duration_peremted):
            self.batteryWindow.expired = True
        self.batteryWindow.window.addstr(1, 2, "• tension:", curses.color_pair(255))
        self.batteryWindow.window.addstr(1, 17, f"{self.battery_card_msg.voltage:.2f}".rjust(8), curses.color_pair(255))
        self.batteryWindow.window.addstr(1, 26, "V", curses.color_pair(255))
        self.batteryWindow.window.addstr(2, 2, "• current:", curses.color_pair(255))
        self.batteryWindow.window.addstr(2, 17, f"{self.battery_card_msg.current:.2f}".rjust(8), curses.color_pair(255))
        self.batteryWindow.window.addstr(2, 26, "A", curses.color_pair(255))
        self.batteryWindow.refresh()

    def imu_window(self):
        self.imuWindow.message_duration = self.duration_ms_from_times(self.imu_time)
        if (self.imuWindow.message_duration > self.duration_peremted):
            self.imuWindow.expired = True

        angles = Rotation.from_quat([
            self.imu_msg.orientation.x,
            self.imu_msg.orientation.y,
            self.imu_msg.orientation.z,
            self.imu_msg.orientation.w
        ]).as_euler('zyx')

        self.imuWindow.window.addstr(1, 2, "• orientation:", curses.color_pair(255))
        self.imuWindow.window.addstr(2, 6, "phi:", curses.color_pair(255))
        self.imuWindow.window.addstr(2, 17, f"{angles[0]*180/np.pi:.2f}".rjust(8), curses.color_pair(255))
        self.imuWindow.window.addstr(2, 26, "°", curses.color_pair(255))
        self.imuWindow.window.addstr(3, 6, "theta:", curses.color_pair(255))
        self.imuWindow.window.addstr(3, 17, f"{angles[1]*180/np.pi:.2f}".rjust(8), curses.color_pair(255))
        self.imuWindow.window.addstr(3, 26, "°", curses.color_pair(255))
        self.imuWindow.window.addstr(4, 6, "psi:", curses.color_pair(255))
        self.imuWindow.window.addstr(4, 17, f"{angles[2]*180/np.pi:.2f}".rjust(8), curses.color_pair(255))
        self.imuWindow.window.addstr(4, 26, "°", curses.color_pair(255))

        self.imuWindow.window.addstr(5, 2, "• angular velocity:", curses.color_pair(255))
        self.imuWindow.window.addstr(6, 6, "x:", curses.color_pair(255))
        self.imuWindow.window.addstr(6, 17, f"{self.imu_msg.angular_velocity.x*180/np.pi:.2f}".rjust(8), curses.color_pair(255))
        self.imuWindow.window.addstr(6, 26, "°/s", curses.color_pair(255))
        self.imuWindow.window.addstr(7, 6, "y:", curses.color_pair(255))
        self.imuWindow.window.addstr(7, 17, f"{self.imu_msg.angular_velocity.y*180/np.pi:.2f}".rjust(8), curses.color_pair(255))
        self.imuWindow.window.addstr(7, 26, "°/s", curses.color_pair(255))
        self.imuWindow.window.addstr(8, 6, "z:", curses.color_pair(255))
        self.imuWindow.window.addstr(8, 17, f"{self.imu_msg.angular_velocity.z*180/np.pi:.2f}".rjust(8), curses.color_pair(255))
        self.imuWindow.window.addstr(8, 26, "°/s", curses.color_pair(255))

        self.imuWindow.window.addstr(9, 2, "• linear acceleration:", curses.color_pair(255))
        self.imuWindow.window.addstr(10, 6, "x:", curses.color_pair(255))
        self.imuWindow.window.addstr(10, 17, f"{self.imu_msg.linear_acceleration.x:.2f}".rjust(8), curses.color_pair(255))
        self.imuWindow.window.addstr(10, 26, "m/s²", curses.color_pair(255))
        self.imuWindow.window.addstr(11, 6, "y:", curses.color_pair(255))
        self.imuWindow.window.addstr(11, 17, f"{self.imu_msg.linear_acceleration.y:.2f}".rjust(8), curses.color_pair(255))
        self.imuWindow.window.addstr(11, 26, "m/s²", curses.color_pair(255))
        self.imuWindow.window.addstr(12, 6, "z:", curses.color_pair(255))
        self.imuWindow.window.addstr(12, 17, f"{self.imu_msg.linear_acceleration.z:.2f}".rjust(8), curses.color_pair(255))
        self.imuWindow.window.addstr(12, 26, "m/s²", curses.color_pair(255))
        self.imuWindow.refresh()

    def duration_ms_from_times(self, t0):
        t1 = self.get_clock().now()
        t = t1.nanoseconds / 1e6 - t0.nanoseconds / 1e6
        return int(t)

    def barometer_callback(self, msg):
        self.barometer_msg = msg
        self.barometer_time = msg.header.stamp

    def battery_card_callback(self, msg):
        self.battery_card_msg = msg
        self.battery_card_time = msg.header.stamp

    def actuators_callback(self, msg):
        self.actuators_msg = msg
        self.actuators_time = msg.header.stamp

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.imu_time = msg.header.stamp

    def keyboard_press(self, key):
        if key == keyboard.Key.esc:
            self.sigint_handler()
        elif key == keyboard.Key.tab:
            self.current_tab = (self.current_tab + 1) % 2
            self.stdscr.clear()
            self.stdscr.refresh()
        else:
            return True

    def keyboard_release(self, key):
        pass

def main(args=None):
    rclpy.init(args=args)
    riptide_wtf = RiptideWTF()
    rclpy.spin(riptide_wtf)


if __name__ == '__main__':
    main()
