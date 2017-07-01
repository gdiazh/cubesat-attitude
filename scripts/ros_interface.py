#!/usr/bin/python

__author__ = 'gdiaz'

# ROS INTERFACE

"""Provides a high level interface over ROS for bluetooth data exchange.
"""

import rospy

from threading import Thread
from std_msgs.msg import String
from bt_receiver import btReceiver
from file_manager import fileManager

class BTRosInterface:
    def __init__(self):
        # Only argument stuff
        self.running = False
        self.bt_receiver = btReceiver(debug = True)
        self.file_manager = fileManager(test_name, debug = True)

    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param('/rate', 2)
        #Buetooth
        self.bt_receiver.initialize()

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
		#subscribers
        self.command_sub = rospy.Subscriber('/bt_command', String, self.process_command)
        #publishers
        self.data_pub = rospy.Publisher('/bt_data', String)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.command_sub.unregister()
        self.data_pub.unregister()
        self.bt_receiver.stop()
        self.file_manager.stop()

    def process_command(self, msg):
        print "msg to send:"+msg.data
        if (msg.data == "end") :
            self.running = False
            return
        else if (msg.data == "cmd1"): self.client_socket_.send("1")
        else if (msg.data == "cmd2"): self.client_socket_.send("2")
        else if (msg.data == "cmd3"): self.client_socket_.send("3")
        else: rospy.logwarn("Unknown command")

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            # Receive data
            self.bt_receiver.read()
            packet = self.bt_receiver.packet
            self.bt_receiver.reset()
            #write data to file
            self.file_manager.save_data(packet, 5)
            #publish data
            data = str(self.file_manager.decode(packet))
            self.data_pub.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('bt_interface')
    bt_ros = BTRosInterface()
    bt_ros.initialize()
    bt_ros.start()
    rospy.spin()
    bt_ros.stop()