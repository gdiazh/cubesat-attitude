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
from bt_receiver import btReceiver
from file_manager import fileManager

class BTRosInterface:
    def __init__(self, test_name):
        # Only argument stuff
        self.running = False
        self.bt_receiver = btReceiver(debug = False)
        self.file_manager = fileManager(test_name, debug = False)
        self.speed = 0

    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param('/rate', 150)
        #Buetooth
        self.bt_receiver.initialize()

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
		#subscribers
        self.command_sub = rospy.Subscriber('/controller_command', String, self.process_command)
        self.speed_sub = rospy.Subscriber('/speed', Int16, self.set_speed)
        #publishers
        self.data1_pub = rospy.Publisher('/data1', Float32, queue_size=50)
        self.data2_pub = rospy.Publisher('/data2', Float32, queue_size=50)
        self.data3_pub = rospy.Publisher('/data3', Float32, queue_size=50)
        self.data4_pub = rospy.Publisher('/data4', Float32, queue_size=50)
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

    def set_speed(self, msg):
        self.speed = msg.data

    def process_command(self, msg):
        print "msg to send:"+msg.data
        if (msg.data == "end") :
            self.running = False
            return
        elif (msg.data == "set-speed"):
            self.bt_receiver.btSocket.send(str(self.speed))
        elif (msg.data == "stop"):
            self.bt_receiver.btSocket.send("-1")
        elif (msg.data == "print-speed"):
            self.bt_receiver.btSocket.send("-2")
        elif (msg.data == "automatic-mode"):
            self.bt_receiver.btSocket.send("-3")
        elif (msg.data == "change-setpoint"):
            self.bt_receiver.btSocket.send("-4")
        elif (msg.data == "increase-setpoint"):
            self.bt_receiver.btSocket.send("-5")
        elif (msg.data == "decrease-setpoint"):
            self.bt_receiver.btSocket.send("-6")
        else:
            rospy.logwarn("Unknown command")
            print "Unknown command"

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            # Receive data
            self.bt_receiver.read()
            packet = self.bt_receiver.packet
            self.bt_receiver.reset()
            #write data to file
            self.file_manager.save_data(packet)
            #publish data
            data = self.file_manager.decode(packet)
            try: self.data1_pub.publish(data[0])
            except:
                print data
                print type(data)
                print type(data[0])
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