#!/usr/bin/python

__author__ = 'gdiaz'

# COMMAND PARSER
import rospy

# Controller commands
SET_VOLTAGE = 1
SET_TORQUE = 2
SET_SPEED = 3
KEEP_ATTITUDE = 4
SET_MODE = 5
STOP = 6
USE_CURRENT_SETPOINT = 7
INCREASE_SETPOINT = 8
DECREASE_SETPOINT = 9
PRINT_SPEED = 10
AUTOMATIC_MODE = 11
STOP_X = 12
STOP_Y = 13
STOP_Z = 14

HDD_ZERO_SPEED = 1000

class CommandParser:
    def __init__(self):
        self.command = "default"

    def parse_command(self, command, args):
        mode = args[0]
        attitude = args[1]
        speed =args[2]
        torque =args[3]
        voltage =args[4]
        if (command == "set-voltage"):
            self.command_code = [SET_VOLTAGE, voltage[0], voltage[1], voltage[2]]
        elif (command == "set-torque"):
            self.command_code = [SET_TORQUE, torque[0], torque[1], torque[2]]
        elif (command == "set-speed"):
            self.command_code = [SET_SPEED, speed[0], speed[1], speed[2]]
        elif (command == "keep-attitude"):
            self.command_code = [KEEP_ATTITUDE, attitude[0], attitude[1], attitude[2]]
        elif (command == "set-mode"):
            self.command_code = [SET_MODE, mode, 0, 0]
        elif (command == "stop"):
            self.command_code = [STOP, HDD_ZERO_SPEED, HDD_ZERO_SPEED, HDD_ZERO_SPEED]
        elif (command == "use-current-setpoint"):
            self.command_code = [USE_CURRENT_SETPOINT, 0, 0, 0]
        elif (command == "increase-setpoint"):
            self.command_code = [INCREASE_SETPOINT, 0, 0, 0]
        elif (command == "decrease-setpoint"):
            self.command_code = [DECREASE_SETPOINT, 0, 0, 0]
        elif (command == "print-speed"):
            self.command_code = [PRINT_SPEED, 0, 0, 0]
        elif (command == "automatic-mode"):
            self.command_code = [AUTOMATIC_MODE, 0, 0, 0]
        else:
            self.command_code = [-1,-1,-1,-1]
            rospy.logwarn("Unknown command")
            print "Unknown command"
        return self.command_code


if __name__ == '__main__':
    comm = CommandParser()
    args = ["manual", [0,0,0], 200]
    comm.parse_command("set-speed", args)