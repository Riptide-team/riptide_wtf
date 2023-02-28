import curses
from abc import ABC, abstractmethod

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
        self.window.refresh()

    def set_window(self):
        self.window.bkgd(' ', self.base_color)
        self.window.box()

    @abstractmethod
    def set_title(self):
        pass


class InfoWindow(Window):
    def __init__(self, title, h, w, x, y, base_color=curses.color_pair(242)):
        super().__init__(title, h, w, x, y, base_color)

    def set_title(self):
        self.title_color = curses.color_pair(5)
        self.window.addstr(0, 1, f" • {self.title} ", curses.A_BOLD | self.title_color)

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