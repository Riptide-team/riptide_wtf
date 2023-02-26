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

from scipy.spatial.transform import Rotation

class RiptideWTF(Node):

    def __init__(self):
        super().__init__('riptide_wtf')

        self.duration_peremted = 10000
        
        self.pressure_subscription = self.create_subscription(Pressure, '/riptide_1/pressure_broadcaster/pressure_status', self.pressure_callback, 10)
        self.battery_card_subscription = self.create_subscription(BatteryState, '/riptide_1/battery_card_broadcaster/battery_status', self.battery_card_callback, 10)
        self.actuators_subscription = self.create_subscription(Actuators, '/riptide_1/actuators_broadcaster/actuators_status', self.actuators_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, '/riptide_1/imu_broadcaster/imu_status', self.imu_callback, 10)

        self.pressure_msg = Pressure()
        self.battery_card_msg = BatteryState()
        self.actuators_msg = Actuators()
        self.imu_msg = Imu()

        self.pressure_time = self.get_clock().now()
        self.battery_card_time = self.get_clock().now()
        self.actuators_time = self.get_clock().now()
        self.imu_time = self.get_clock().now()

        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.init_ncurses()
        signal.signal(signal.SIGINT, self.sigint_handler)

    def sigint_handler(self, sig, frame):
        rclpy.shutdown()
        curses.endwin()
        os.system('cls||clear')
        sys.exit(0)

    def timer_callback(self):
        self.host_window()
        self.daemon_window()
        self.pressure_window()
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

        self.hostWindow = curses.newwin(3, 33, 1, 1)
        self.hostWindow.box()

        self.daemonWindow = curses.newwin(3, 33, 1, 35)
        self.daemonWindow.box()

        self.actuatorsWindow = curses.newwin(6, 33, 4, 1)
        self.actuatorsWindow.box()

        self.pressureWindow = curses.newwin(6, 33, 4, 35)
        self.pressureWindow.box()

        self.imuWindow = curses.newwin(14, 33, 10, 1)
        self.imuWindow.box()

        self.batteryWindow = curses.newwin(4, 33, 10, 35)
        self.batteryWindow.box()

    def host_window(self):
        self.hostWindow.addstr(0, 1, f" • Host ", curses.A_BOLD | curses.color_pair(5))
        self.hostWindow.addstr(1, 4, f"{os.uname()[1]}")
        self.hostWindow.refresh()
    
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
        self.daemonWindow.addstr(1, 3, f"• Ros2Control ({status})".ljust(29), color)

        marker = "✔"
        color = curses.color_pair(3)
        if status != "active":
            marker = "✘"
            color = curses.color_pair(2)

        self.daemonWindow.addstr(0, 2, f" {marker} Daemon ", curses.A_BOLD | color)
        self.daemonWindow.refresh()

    def pressure_window(self):
        marker = "✔"
        color = curses.color_pair(3)
        duration = self.duration_ms_from_times(self.pressure_time)
        if (duration > self.duration_peremted):
            marker = "✘"
            color = curses.color_pair(2)
        self.pressureWindow.addstr(0, 1, f" {marker} Pressure ({duration} ms) ", curses.A_BOLD | color)

        self.pressureWindow.addstr(1, 2, "• pressure:")
        self.pressureWindow.addstr(1, 17, f"{self.pressure_msg.pressure:.2f}".rjust(8))
        self.pressureWindow.addstr(1, 26, "mbar")
        self.pressureWindow.addstr(2, 2, "• temperature:")
        self.pressureWindow.addstr(2, 17, f"{self.pressure_msg.temperature:.2f}".rjust(8))
        self.pressureWindow.addstr(2, 26, "°C")
        self.pressureWindow.addstr(3, 2, "• depth:")
        self.pressureWindow.addstr(3, 17, f"{self.pressure_msg.depth:.2f}".rjust(8))
        self.pressureWindow.addstr(3, 26, "m")
        self.pressureWindow.addstr(4, 2, "• altitude:")
        self.pressureWindow.addstr(4, 17, f"{self.pressure_msg.altitude:.2f}".rjust(8))
        self.pressureWindow.addstr(4, 26, "m")
        self.pressureWindow.refresh()

    def actuators_window(self):
        marker = "✔"
        color = curses.color_pair(3)
        duration = self.duration_ms_from_times(self.actuators_time)
        if (duration > self.duration_peremted):
            marker = "✘"
            color = curses.color_pair(2)
        self.actuatorsWindow.addstr(0, 1, f" {marker} Actuators ({duration} ms) ", curses.A_BOLD | color)

        self.actuatorsWindow.addstr(1, 2, "• thruster:")
        self.actuatorsWindow.addstr(1, 17, f"{self.actuators_msg.thruster:.2f}".rjust(8))
        self.actuatorsWindow.addstr(1, 26, "usi")
        self.actuatorsWindow.addstr(2, 2, "• d_fin:")
        self.actuatorsWindow.addstr(2, 17, f"{self.actuators_msg.d_fin:.2f}".rjust(8))
        self.actuatorsWindow.addstr(2, 26, "rad")
        self.actuatorsWindow.addstr(3, 2, "• p_fin:")
        self.actuatorsWindow.addstr(3, 17, f"{self.actuators_msg.p_fin:.2f}".rjust(8))
        self.actuatorsWindow.addstr(3, 26, "rad")
        self.actuatorsWindow.addstr(4, 2, "• s_fin:")
        self.actuatorsWindow.addstr(4, 17, f"{self.actuators_msg.s_fin:.2f}".rjust(8))
        self.actuatorsWindow.addstr(4, 26, "rad")
        self.actuatorsWindow.refresh()

    def battery_window(self):
        marker = "✔"
        color = curses.color_pair(3)
        duration = self.duration_ms_from_times(self.battery_card_time)
        if (duration > self.duration_peremted):
            marker = "✘"
            color = curses.color_pair(2)
        self.batteryWindow.addstr(0, 2, f" {marker} Battery ({duration} ms) ", curses.A_BOLD | color)

        self.batteryWindow.addstr(1, 2, "• tension:")
        self.batteryWindow.addstr(1, 17, f"{self.battery_card_msg.voltage:.2f}".rjust(8))
        self.batteryWindow.addstr(1, 26, "V")
        self.batteryWindow.addstr(2, 2, "• current:")
        self.batteryWindow.addstr(2, 17, f"{self.battery_card_msg.current:.2f}".rjust(8))
        self.batteryWindow.addstr(2, 26, "A")
        self.batteryWindow.refresh()

    def imu_window(self):
        marker = "✔"
        color = curses.color_pair(3)
        duration = self.duration_ms_from_times(self.imu_time)
        if (duration > self.duration_peremted):
            marker = "✘"
            color = curses.color_pair(2)
        self.imuWindow.addstr(0, 2, f" {marker} Imu ({self.duration_ms_from_times(self.imu_time)} ms) ", curses.A_BOLD | color)

        angles = Rotation.from_quat([
            self.imu_msg.orientation.x,
            self.imu_msg.orientation.y,
            self.imu_msg.orientation.z,
            self.imu_msg.orientation.w
        ]).as_euler('zyx')

        self.imuWindow.addstr(1, 2, "• orientation:")

        self.imuWindow.addstr(2, 6, "phi:")
        self.imuWindow.addstr(2, 17, f"{angles[0]:.2f}".rjust(8))
        self.imuWindow.addstr(2, 26, "rad")
        self.imuWindow.addstr(3, 6, "theta:")
        self.imuWindow.addstr(3, 17, f"{angles[1]:.2f}".rjust(8))
        self.imuWindow.addstr(3, 26, "rad")
        self.imuWindow.addstr(4, 6, "psi:")
        self.imuWindow.addstr(4, 17, f"{angles[2]:.2f}".rjust(8))
        self.imuWindow.addstr(4, 26, "rad")

        self.imuWindow.addstr(5, 2, "• angular velocity:")
        self.imuWindow.addstr(6, 6, "x:")
        self.imuWindow.addstr(6, 17, f"{self.imu_msg.angular_velocity.x:.2f}".rjust(8))
        self.imuWindow.addstr(6, 26, "rad/s")
        self.imuWindow.addstr(7, 6, "y:")
        self.imuWindow.addstr(7, 17, f"{self.imu_msg.angular_velocity.y:.2f}".rjust(8))
        self.imuWindow.addstr(7, 26, "rad/s")
        self.imuWindow.addstr(8, 6, "z:")
        self.imuWindow.addstr(8, 17, f"{self.imu_msg.angular_velocity.z:.2f}".rjust(8))
        self.imuWindow.addstr(8, 26, "rad/s")

        self.imuWindow.addstr(9, 2, "• linear acceleration:")
        self.imuWindow.addstr(10, 6, "x:")
        self.imuWindow.addstr(10, 17, f"{self.imu_msg.linear_acceleration.x:.2f}".rjust(8))
        self.imuWindow.addstr(10, 26, "m/s²")
        self.imuWindow.addstr(11, 6, "y:")
        self.imuWindow.addstr(11, 17, f"{self.imu_msg.linear_acceleration.y:.2f}".rjust(8))
        self.imuWindow.addstr(11, 26, "m/s²")
        self.imuWindow.addstr(12, 6, "z:")
        self.imuWindow.addstr(12, 17, f"{self.imu_msg.linear_acceleration.z:.2f}".rjust(8))
        self.imuWindow.addstr(12, 26, "m/s²")
        self.imuWindow.refresh()

    def duration_ms_from_times(self, t0):
        t1 = self.get_clock().now()
        t = t1.nanoseconds / 1e6 - t0.nanoseconds / 1e6
        return int(t)

    def pressure_callback(self, msg):
        self.pressure_msg = msg
        self.pressure_time = msg.header.stamp

    def battery_card_callback(self, msg):
        self.battery_card_msg = msg
        self.battery_card_time = msg.header.stamp

    def actuators_callback(self, msg):
        self.actuators_msg = msg
        self.actuators_time = msg.header.stamp

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.imu_time = msg.header.stamp


def main(args=None):
    rclpy.init(args=args)
    riptide_wtf = RiptideWTF()
    rclpy.spin(riptide_wtf)


if __name__ == '__main__':
    main()
