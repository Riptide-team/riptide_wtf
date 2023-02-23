#!/usr/bin/env python3

import rclpy
from rclpy.time import Time
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

        self.init_ncurses()

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.daemon_window()
        self.pressure_window()
        self.battery_window()
        self.imu_window()
        self.actuators_window()

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

        self.imuWindow = curses.newwin(11, 32, 12, 1)
        self.imuWindow.box()

        self.actuatorsWindow = curses.newwin(7, 32, 12, 34)
        self.actuatorsWindow.box()
    
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

    def duration_ms_from_times(self, t0):
        t1 = self.get_clock().now()
        t = t1.nanoseconds / 1e6 - t0.nanoseconds / 1e6
        return int(t)

    def pressure_window(self):
        self.pressureWindow.addstr(1, 2, "• Pressure", curses.A_BOLD | curses.color_pair(3))

        # # Timestamp
        # duration = self.duration_ms_from_times(self.pressure_time)
        # if (duration < 100):
        #     color = curses.color_pair(1)
        # else:
        #     color = curses.color_pair(2)
        # self.pressureWindow.addstr(1, 18, f"{duration} ms".rjust(10), color)

        self.pressureWindow.addstr(2, 4, "pressure:")
        self.pressureWindow.addstr(2, 20, f"{self.pressure_msg.pressure:.2f} mbar".rjust(10))
        self.pressureWindow.addstr(3, 4, "temperature:")
        self.pressureWindow.addstr(3, 18, f"{self.pressure_msg.temperature:.2f} °C".rjust(10))
        self.pressureWindow.addstr(4, 4, "depth:")
        self.pressureWindow.addstr(4, 17, f"{self.pressure_msg.depth:.2f} m".rjust(10))
        self.pressureWindow.addstr(5, 4, "altitude:")
        self.pressureWindow.addstr(5, 17, f"{self.pressure_msg.altitude:.2f} m".rjust(10))
        self.pressureWindow.refresh()

    def actuators_window(self):
        self.actuatorsWindow.addstr(1, 2, "• Actuators", curses.A_BOLD | curses.color_pair(3))
        self.actuatorsWindow.addstr(2, 4, "thruster:")

        # # Timestamp
        # duration = self.duration_ms_from_times(self.actuators_time)
        # if (duration < 100):
        #     color = curses.color_pair(1)
        # else:
        #     color = curses.color_pair(2)
        # self.actuatorsWindow.addstr(1, 19, f"{duration} ms".rjust(10), color)

        self.actuatorsWindow.addstr(2, 20, f"{self.actuators_msg.thruster:.2f} usi".rjust(10))
        self.actuatorsWindow.addstr(3, 4, "d_fin:")
        self.actuatorsWindow.addstr(3, 20, f"{self.actuators_msg.d_fin:.2f} rad".rjust(10))
        self.actuatorsWindow.addstr(4, 4, "p_fin:")
        self.actuatorsWindow.addstr(4, 20, f"{self.actuators_msg.p_fin:.2f} rad".rjust(10))
        self.actuatorsWindow.addstr(5, 4, "s_fin:")
        self.actuatorsWindow.addstr(5, 20, f"{self.actuators_msg.s_fin:.2f} rad".rjust(10))
        self.actuatorsWindow.refresh()

    def battery_window(self):
        self.batteryWindow.addstr(1, 2, "• Battery", curses.A_BOLD | curses.color_pair(3))

        # # Timestamp
        # duration = self.duration_ms_from_times(self.battery_card_time)
        # if (duration < 100):
        #     color = curses.color_pair(1)
        # else:
        #     color = curses.color_pair(2)
        # self.batteryWindow.addstr(1, 20, f"{duration} ms".rjust(10), color)

        self.batteryWindow.addstr(2, 4, "tension:")
        self.batteryWindow.addstr(2, 20, f"{self.battery_card_msg.voltage:.2f} V".rjust(10))
        self.batteryWindow.addstr(3, 4, "current:")
        self.batteryWindow.addstr(3, 20, f"{self.battery_card_msg.current:.2f} A".rjust(10))
        self.batteryWindow.refresh()

    def imu_window(self):
        self.imuWindow.addstr(1, 2, "• Imu", curses.A_BOLD | curses.color_pair(3))

        # # Timestamp
        # duration = self.duration_ms_from_times(self.imu_time)
        # if (duration < 100):
        #     color = curses.color_pair(1)
        # else:
        #     color = curses.color_pair(2)
        # self.imuWindow.addstr(1, 18, f"{duration} ms".rjust(10), color)

        self.imuWindow.addstr(2, 4, "Linear acceleration:")
        self.imuWindow.addstr(3, 6, "x:")
        self.imuWindow.addstr(3, 20, f"{self.imu_msg.linear_acceleration.x:.2f} m/s^2".rjust(10))
        self.imuWindow.addstr(4, 6, "y:")
        self.imuWindow.addstr(4, 20, f"{self.imu_msg.linear_acceleration.y:.2f} m/s^2".rjust(10))
        self.imuWindow.addstr(5, 6, "z:")
        self.imuWindow.addstr(5, 20, f"{self.imu_msg.linear_acceleration.z:.2f} m/s^2".rjust(10))
        self.imuWindow.addstr(6, 4, "Angular velocity:")
        self.imuWindow.addstr(7, 6, "x:")
        self.imuWindow.addstr(7, 20, f"{self.imu_msg.angular_velocity.x:.2f} rad/s".rjust(10))
        self.imuWindow.addstr(8, 6, "y:")
        self.imuWindow.addstr(8, 20, f"{self.imu_msg.angular_velocity.y:.2f} rad/s".rjust(10))
        self.imuWindow.addstr(9, 6, "z:")
        self.imuWindow.addstr(9, 20, f"{self.imu_msg.angular_velocity.z:.2f} rad/s".rjust(10))
        self.imuWindow.refresh()

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
    riptide_wtf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
