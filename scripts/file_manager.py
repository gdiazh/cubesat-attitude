#!/usr/bin/python

__author__ = 'gdiaz'

class fileManager(object):
    def __init__(self, file_name, debug = False):
        self.debug = debug
        self.file_path = "data/"
        self.file_name = file_name
        self.file_ext = ".txt"
        self.full_name = self.file_path+self.file_name+self.file_ext
        self.data_file = open(self.full_name, "ab+")

    def stop(self):
        self.data_file.close();

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

    def checksum(self, packet, sz):
        sum = 0
        for j in range(0,sz-1): sum += packet[j]
        return sum

    def encode(self, data):
        packet = [0,0,0,0,0]
        int1 = int(data[0])
        decimal1 = (data[0]-int1)*1000
        if(data[0]>=0):
            packet[0] = abs(int1)
            packet[1] = abs(int(decimal1))+1
        else:
            packet[0] = abs(int1)|0b10000000
            packet[1] = abs(int(decimal1))

        int2 = int(data[1])
        decimal2 = (data[1]-int2)*1000
        if(data[1]>=0):
            packet[2] = abs(int2)
            packet[3] = abs(int(decimal2))+1
        else:
            packet[2] = abs(int2)|0b10000000
            packet[3] = abs(int(decimal2))

        packet[4] = self.checksum(packet, 5) & 0x00ff

        return packet

    def decode(self, packet):
        sign1 = packet[0]>>7
        int1 = packet[0]&(0b01111111)
        decimal1 = packet[1]

        sign2 = packet[2]>>7
        int2 = packet[2]&(0b01111111)
        decimal2 = packet[3]

        if (sign1 == 0): data1 = int1+(float(decimal1)/1000)
        else: data1 = -(int1+(float(decimal1)/1000))
        if (sign2 == 0): data2 = int2+(float(decimal2)/1000)
        else: data2 = -(int2+(float(decimal2)/1000))

        return [data1, data2]

    def to_file(self, data, sz):
        self.data_file = open(self.full_name, "ab+")
        for i in range(0,sz):
            self.data_file.write(str(data[i]))
            self.data_file.write(";")
        self.data_file.write("\n")
        self.data_file.close()

    def save_data(self, packet, sz):
        data = self.decode(packet)
        self.to_file(data, 2)


if __name__ == '__main__':
    file_name = raw_input("file_name: ")
    file_manager = fileManager(file_name, debug = True)
    # test
    n_test = 10
    packet_test = [1,6,1,8,16]
    print "packet_encode = " + str(file_manager.encode([114.123, -121.056]))
    print "data_decode = "+str(file_manager.decode([114,124,135,56,173]))
    # data = [123.234, -123.435]
    # packet_test2 = file_manager.encode(data)
    # print "starting test ..."
    # print "data = "+str(data)
    # print "packet_encode = " + str(packet_test2)
    # print "data_decode = "+str(file_manager.decode(packet_test2))
    # i = 0
    # while i<n_test:
    #     # file_manager.to_file(packet_test, 5)
    #     file_manager.save_data(packet_test2, 5)
    #     i+=1
    print "test finish ..."
    file_manager.stop()