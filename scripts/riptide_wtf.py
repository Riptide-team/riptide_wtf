#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import curses

class RiptideWTF(Node):

    def __init__(self):
        super().__init__('riptide_wtf')
        self.init_ncurses()
        self.pressure_window()

    def init_ncurses(self):
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        curses.curs_set(False)
        if curses.has_colors():
            curses.start_color()

        self.pressureWindow = curses.newwin(10, 40, 1, 1)
        self.pressureWindow.box()

    def pressure_window(self):
        self.pressureWindow.addstr(1, 3, "Pressure", curses.A_BOLD)
        self.pressureWindow.refresh()


def main(args=None):
    rclpy.init(args=args)
    riptide_wtf = RiptideWTF()
    rclpy.spin(riptide_wtf)
    riptide_wtf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
