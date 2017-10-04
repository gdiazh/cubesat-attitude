#!/usr/bin/python

__author__ = 'gdiaz'

# ROS INTERFACE

"""Provides a high level interface over ROS for bluetooth data exchange.
"""

import rospy

from threading import Thread
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from bt_receiver import btReceiver
from file_manager import fileManager
from command_parser import CommandParser

class BTRosInterface:
    def __init__(self, test_name):
        # Only argument stuff
        self.running = False
        self.bt_receiver = btReceiver(debug = True)
        self.file_manager = fileManager(test_name, debug = True)
        self.command_parser = CommandParser()
        self.mode = "manual"
        self.attitude = [0,0,0]     #[ypr]
        self.speed = 0              #[RPM]

    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param('/rate', 1000)
        #Buetooth
        self.bt_receiver.initialize()

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        #subscribers
        self.command_sub = rospy.Subscriber('/controller_command', String, self.process_command)
        self.speed_sub = rospy.Subscriber('/mode', String, self.set_mode)
        self.speed_sub = rospy.Subscriber('/attitude', Float32MultiArray, self.set_attitude)
        self.speed_sub = rospy.Subscriber('/speed', Float32, self.set_speed)
        #publishers
        self.data1_pub = rospy.Publisher('/yaw', Float32, queue_size=70)
        self.data2_pub = rospy.Publisher('/pid_vel', Float32, queue_size=70)
        self.data3_pub = rospy.Publisher('/wheel_speed', Float32, queue_size=70)
        self.data4_pub = rospy.Publisher('/filtered_wheel_speed', Float32, queue_size=70)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.command_sub.unregister()
        self.speed_sub.unregister()
        self.data1_pub.unregister()
        self.data2_pub.unregister()
        self.data3_pub.unregister()
        self.data4_pub.unregister()
        self.bt_receiver.stop()
        self.file_manager.stop()

    def set_mode(self, msg):
        self.mode = msg.data

    def set_attitude(self, msg):
        self.attitude = msg.data

    def set_speed(self, msg):
        self.speed = msg.data

    def process_command(self, msg):
        print "msg to send:"+msg.data
        if (msg.data == "end") :
            self.running = False
            return
        else:
            args = [self.mode, self.attitude, self.speed]
            command_code = self.command_parser.parse_command(msg.data, args)
            if (command_code!=[-1,-1,-1,-1]):
                frame = self.file_manager.encode(command_code)
                self.bt_receiver.write(frame)
            else:
                rospy.logwarn("Unknown command")
                print "Unknown command"

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            # Receive data
            if(self.bt_receiver.read()):
                packet = self.bt_receiver.packet
                if (packet[0]==1):
                    self.bt_receiver.reset()
                    #write data to file
                    self.file_manager.save_data(packet)
                    #publish data
                    data = self.file_manager.decode(packet)
                    self.data1_pub.publish(data[0])
                    self.data2_pub.publish(data[1])
                    self.data3_pub.publish(data[2])
                    self.data4_pub.publish(data[3])
            rate.sleep()

if __name__ == '__main__':
    test_name = raw_input("test_name: ")
    rospy.init_node('bt_interface')
    bt_ros = BTRosInterface(test_name)
    bt_ros.initialize()
    bt_ros.start()
    rospy.spin()
    bt_ros.stop()