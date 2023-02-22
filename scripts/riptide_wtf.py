#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from riptide_msgs.msg import Pressure
from sensor_msgs.msg import Imu, BatteryState

import curses

class RiptideWTF(Node):

    def __init__(self):
        super().__init__('riptide_wtf')
        
        self.pressure_subscription = self.create_subscription(Pressure, '/riptide_1/pressure_broadcaster/pressure_status', self.pressure_callback, 10)
        self.battery_card_subscription = self.create_subscription(BatteryState, '/riptide_1/battery_card_broadcaster/battery_status', self.battery_card_callback, 10)

        self.pressure_msg = Pressure()
        self.battery_card_msg = BatteryState()

        self.init_ncurses()
        self.pressure_window()
        self.battery_window()

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.pressure_window()
        self.battery_window()

    def init_ncurses(self):
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        curses.curs_set(False)
        if curses.has_colors():
            curses.start_color()

        self.pressureWindow = curses.newwin(7, 40, 1, 1)
        self.pressureWindow.box()

        self.batteryWindow = curses.newwin(7, 40, 1, 42)
        self.batteryWindow.box()

    def pressure_window(self):
        self.pressureWindow.addstr(1, 4, "Pressure", curses.A_BOLD)
        self.pressureWindow.addstr(2, 4, "pressure:")
        self.pressureWindow.addstr(2, 26, f"{self.pressure_msg.pressure:.2f} mbar".rjust(10))
        self.pressureWindow.addstr(3, 4, "temperature:")
        self.pressureWindow.addstr(3, 24, f"{self.pressure_msg.temperature:.2f} °C".rjust(10))
        self.pressureWindow.addstr(4, 4, "depth:")
        self.pressureWindow.addstr(4, 23, f"{self.pressure_msg.depth:.2f} m".rjust(10))
        self.pressureWindow.addstr(5, 4, "altitude:")
        self.pressureWindow.addstr(5, 23, f"{self.pressure_msg.altitude:.2f} m".rjust(10))
        self.pressureWindow.refresh()

    def battery_window(self):
        self.batteryWindow.addstr(1, 4, "Battery", curses.A_BOLD)
        self.batteryWindow.addstr(2, 4, "tension:")
        self.batteryWindow.addstr(2, 23, f"{self.battery_card_msg.voltage:.2f} V".rjust(10))
        self.batteryWindow.addstr(3, 4, "current:")
        self.batteryWindow.addstr(3, 23, f"{self.battery_card_msg.current:.2f} A".rjust(10))
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
