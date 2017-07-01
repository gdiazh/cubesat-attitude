#!/usr/bin/python

__author__ = 'gdiaz'

import time
import bluetooth
from threading import Timer

ACK = 3

class btReceiver(object):
    def __init__(self, debug = False):
        self.btSocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.packet = [0,0,0,0,0]
        self.debug = debug
        self.timeout = False

    def initialize(self):
        #Buetooth
        bt_addr = "00:13:04:03:00:02"
        local_port = 1
        # TO DO: check if actually succeed
        self.btSocket.connect((bt_addr, local_port))
        # self.btSocket.connect(("00:13:04:03:00:02", 1))
        print "Conection succed! starting comunication ..."

    def stop(self):
        print "Conection Finish. Closing ports ..."
        self.btSocket.close()

    def DEBUG_PRINT(self, msg_type, msg):
        if not(self.debug): return
        if msg_type == "info":
            print chr(27)+"[0;32m"+"[INFO]: "+chr(27)+"[0m" + msg
        elif msg_type == "warn":
            print chr(27)+"[0;33m"+"[WARN]: "+chr(27)+"[0m" + msg
        elif msg_type == "error":
            print chr(27)+"[0;31m"+"[ERROR]: "+chr(27)+"[0m" + msg
        elif msg_type == "alert":
            print chr(27)+"[0;34m"+"[ALERT]: "+chr(27)+"[0m" + msg
        else:
            print "NON implemented Debug print type"

    def reset(self):
        self.packet = [0,0,0,0,0]

    def checksum(self, packet, sz):
        sum = 0
        for j in range(0,sz-1): sum += packet[j]
        return sum

    def timeout_handler(self):
        self.DEBUG_PRINT("info", "Time out reach")
        self.timeout = True

    def write(self, FH, D1, D2, D3):
        checksum = (FH + D1 + D2 + D3) & 0x00ff
        #5 byte packet: FH D1 D2 D3 CS
        packet = bytearray([FH, D1, D2, D3, checksum])
        packet_str = str(FH)+str(D1)+str(D2)+str(D3)+str(checksum)
        packet_str_encode = bytes(packet_str, 'UTF-8')
        reply = 0
        t = Timer(10.0, self.timeout_handler)
        t.start()
        while (reply != ACK and not(self.timeout)):
            self.btSocket.send(packet)
            # self.btSocket.send(packet_str_encode)
            #time.sleep(0.01)
            if (self.read()):
                reply = self.packet[1]+self.packet[2]+self.packet[3]
            self.reset()
        if self.timeout:
            self.DEBUG_PRINT("warn", "Not reply received. Timed out.")
        else: #we succeed
            t.cancel()
            self.DEBUG_PRINT("info", "Packet sent OK.")
        self.timeout = False

    def read(self):
        i = 0
        k = 0
        while (k < 10):
            byte = self.btSocket.recv(1)
            # self.DEBUG_PRINT("info", "byte = "+str(byte))
            self.packet[i] = ord(byte)
            i+=1
            if (i==5):
                chksm = self.checksum(self.packet, 5) & 0x00FF #Low byte of data checksum
                if (chksm == self.packet[4] and chksm !=0):
                    return True #packet received OK
                else:
                    for j in range(0,4): self.packet[j] = self.packet[j+1] #Shift Left packet
                    self.packet[4] = 0 #Clean last byte to receive other packet
                    i = 4
                    self.DEBUG_PRINT("warn", "Bad checksum = "+str(chksm))
            k+=1
        # Packet not received Correctly
        for j in range(0,4): self.packet[j] = 0 #Reset packet
        return False

if __name__ == '__main__':
    bt_receiver = btReceiver(debug = True)
    bt_receiver.initialize()
    # test read
    while True:
        bt_receiver.read()
        print bt_receiver.packet
        bt_receiver.reset()
    # test write
    # bt_receiver.write(1, 6, 1, 8)
    bt_receiver.stop()