import curses
from abc import ABC, abstractmethod

from sensor_msgs.msg import Joy

import numpy as np
import time as time

curses.initscr()
curses.start_color()

class Window(ABC):
    def __init__(self, title, h, w, x, y, base_color=curses.color_pair(242)):
        # Window
        self.title = title
        self.base_color = base_color
        self.window = curses.newwin(h, w, y, x)
        self.set_window()
        self.refresh()

    def refresh(self):
        self.set_window()
        self.set_title()
        self.set_content()
        self.window.refresh()

    def set_window(self):
        self.window.bkgd(' ', self.base_color)
        self.window.box()

    @abstractmethod
    def set_title(self):
        pass

    @abstractmethod
    def set_content(self):
        pass


class InfoWindow(Window):
    def __init__(self, title, h, w, x, y, base_color=curses.color_pair(242)):
        super().__init__(title, h, w, x, y, base_color)

    def set_title(self):
        self.title_color = curses.color_pair(5)
        self.window.addstr(0, 1, f" • {self.title} ", curses.A_BOLD | self.title_color)
    
    def set_content(self):
        pass

class StatusWindow(Window):
    def __init__(self, title, h, w, x, y, base_color=curses.color_pair(242)):
        self.status = False
        super().__init__(title, h, w, x, y, base_color)

    def set_title(self):
        self.title_color = curses.color_pair(5)
        if self.status:
            marker = "✔"
            self.title_color = curses.color_pair(3)
        else:
            marker = "✘"
            self.title_color = curses.color_pair(2)
        self.window.addstr(0, 1, f" {marker} {self.title} ", curses.A_BOLD | self.title_color)
    
    def set_content(self):
        pass

class TimedWindow(Window):
    def __init__(self, title, h, w, x, y, base_color=curses.color_pair(242)):
        self.expired = False
        self.message_duration = 0.
        super().__init__(title, h, w, x, y, base_color)

    def set_title(self):
        self.title_color = curses.color_pair(5)
        if self.expired:
            marker = "✘"
            self.title_color = curses.color_pair(2)
        else:
            marker = "✔"
            self.title_color = curses.color_pair(3)
        self.window.addstr(0, 1, f" {marker} {self.title} ({self.message_duration} ms) ", curses.A_BOLD | self.title_color)

    def set_content(self):
        pass

class RCWindow(TimedWindow):
    def __init__(self, x, y, base_color=curses.color_pair(242)):
        self.msg = Joy()
        super().__init__("RC", 8, 30, x, y, base_color)

    def set_content(self):
        def octal(x):
            return [int(s) for s in f"{int(40*(value+1)):03o}"]
        
        for i, value in enumerate(self.msg.axes):
            self.window.addstr(i+1, 2, f"• CH{i+1}: {value: 3.2f}", curses.color_pair(255))
            o = octal(value)
            bar = ("\u2595" + (8 * o[0] + o[1]) * "\u2588" + eval(r"u'\u258%x'" % (15-o[2]))).ljust(12)
            bar = bar[:-1] + "\u258F"
            self.window.addstr(i+1, 17, bar, curses.color_pair(255))
