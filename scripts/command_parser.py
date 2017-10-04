#!/usr/bin/python

__author__ = 'gdiaz'

# COMMAND PARSER
import rospy

class CommandParser:
    def __init__(self):
        self.command = "default"

    def parse_command(self, command, args):
        mode = args[0]
        attitude = args[1]
        speed =args[2]
        if (command == "set-speed"):
            self.command_code = [1, speed, 0, 0]
            # self.bt_receiver.btSocket.send(str(self.speed))
        elif (command == "keep-attitude"):
            self.command_code = [2, attitude[0], attitude[1], attitude[2]]
        elif (command == "set-mode"):
            self.command_code = [3, mode, 0, 0]
        elif (command == "stop"):
            self.command_code = [4, 0, 0, 0]
            # self.bt_receiver.btSocket.send("-1")
        elif (command == "use-current-setpoint"):
            self.command_code = [5, 0, 0, 0]
            # self.bt_receiver.btSocket.send("-4")
        elif (command == "increase-setpoint"):
            self.command_code = [6, 0, 0, 0]
            # self.bt_receiver.btSocket.send("-5")
        elif (command == "decrease-setpoint"):
            self.command_code = [7, 0, 0, 0]
            # self.bt_receiver.btSocket.send("-6")
        elif (command == "print-speed"):
            self.command_code = [8, 0, 0, 0]
            # self.bt_receiver.btSocket.send("-2")
        elif (command == "automatic-mode"):
            self.command_code = [9, 0, 0, 0]
            # self.bt_receiver.btSocket.send("-3")
        else:
            self.command_code = [-1,-1,-1,-1]
            rospy.logwarn("Unknown command")
            print "Unknown command"
        return self.command_code


if __name__ == '__main__':
    comm = CommandParser()
    args = ["manual", [0,0,0], 200]
    comm.parse_command("set-speed", args)