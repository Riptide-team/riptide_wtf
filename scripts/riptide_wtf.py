#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from riptide_msgs.msg import Pressure, Actuators
from sensor_msgs.msg import Imu, BatteryState

import dbus
from systemd import journal

import curses

class RiptideWTF(Node):

    def __init__(self):
        super().__init__('riptide_wtf')
        
        self.pressure_subscription = self.create_subscription(Pressure, '/riptide_1/pressure_broadcaster/pressure_status', self.pressure_callback, 10)
        self.battery_card_subscription = self.create_subscription(BatteryState, '/riptide_1/battery_card_broadcaster/battery_status', self.battery_card_callback, 10)

        self.pressure_msg = Pressure()
        self.battery_card_msg = BatteryState()

        self.init_ncurses()

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.pressure_window()
        self.battery_window()
        self.daemon_window()

    def init_ncurses(self):
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        curses.curs_set(False)
        if curses.has_colors():
            curses.start_color()

        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_BLUE, curses.COLOR_BLACK)

        self.daemonWindow = curses.newwin(4, 65, 1, 1)
        self.daemonWindow.box()

        self.pressureWindow = curses.newwin(7, 32, 5, 1)
        self.pressureWindow.box()

        self.batteryWindow = curses.newwin(7, 32, 5, 34)
        self.batteryWindow.box()

        self.imuWindow = curses.newwin(13, 32, 5, 1)
        self.imuWindow.box()
    
    def daemon_window(self):
        color = curses.color_pair(1)

        # Get the system bus
        bus = dbus.SystemBus()

        # Get the service manager object
        manager = dbus.Interface(bus.get_object('org.freedesktop.systemd1', '/org/freedesktop/systemd1'), 'org.freedesktop.systemd1.Manager')

        # Get the unit object for your service
        unit = manager.LoadUnit('ros2_control.service')

        # Get the properties of the unit
        props = dbus.Interface(bus.get_object('org.freedesktop.systemd1', unit), 'org.freedesktop.DBus.Properties')
        status = props.Get('org.freedesktop.systemd1.Unit', 'ActiveState')

        self.daemonWindow.addstr(2, 15, status)
        if (status=="active"):
            color = curses.color_pair(1)
        else:
            color = curses.color_pair(2)
        
        self.daemonWindow.addstr(2, 4, f"• Ros2Control ({status})        ", color)
        if status != "active":
            color = curses.color_pair(2)

        self.daemonWindow.addstr(1, 2, "✔ Daemon", curses.A_BOLD | color)

        self.daemonWindow.refresh()

    def pressure_window(self):
        self.pressureWindow.addstr(1, 2, "• Pressure", curses.A_BOLD | curses.color_pair(3))
        self.pressureWindow.addstr(2, 4, "pressure:")
        self.pressureWindow.addstr(2, 20, f"{self.pressure_msg.pressure:.2f} mbar".rjust(10))
        self.pressureWindow.addstr(3, 4, "temperature:")
        self.pressureWindow.addstr(3, 18, f"{self.pressure_msg.temperature:.2f} °C".rjust(10))
        self.pressureWindow.addstr(4, 4, "depth:")
        self.pressureWindow.addstr(4, 17, f"{self.pressure_msg.depth:.2f} m".rjust(10))
        self.pressureWindow.addstr(5, 4, "altitude:")
        self.pressureWindow.addstr(5, 17, f"{self.pressure_msg.altitude:.2f} m".rjust(10))
        self.pressureWindow.refresh()

    def battery_window(self):
        self.batteryWindow.addstr(1, 2, "• Battery", curses.A_BOLD | curses.color_pair(3))
        self.batteryWindow.addstr(2, 4, "tension:")
        self.batteryWindow.addstr(2, 19, f"{self.battery_card_msg.voltage:.2f} V".rjust(10))
        self.batteryWindow.addstr(3, 4, "current:")
        self.batteryWindow.addstr(3, 19, f"{self.battery_card_msg.current:.2f} A".rjust(10))
        self.batteryWindow.refresh()

    def pressure_callback(self, msg):
        self.pressure_msg = msg

    def battery_card_callback(self, msg):
        self.battery_card_msg = msg


def main(args=None):
    rclpy.init(args=args)
    riptide_wtf = RiptideWTF()
    rclpy.spin(riptide_wtf)
    riptide_wtf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
