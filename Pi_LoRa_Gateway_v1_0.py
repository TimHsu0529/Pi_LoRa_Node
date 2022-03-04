from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD
import binascii
import os
import time
import sys
import gps
import serial

#serial port setup
ser = serial.Serial('/dev/ttyUSB0', 9600)

#lora hat setup
BOARD.setup()

#gps setup
session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

#data type
class Receive_Data():
    def __init__(self, receive):
        self.node_id = receive[1]
        self.crc_code = bytes(receive[2:6])
        self.data = bytes(receive[6:])
        self.out_data = ""

    def TX_string(self, data: str):
        self.out_data = data

#lora class
class LoRaRcvCont(LoRa):
    gateway_id = 0xe1

#lora setup
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)
        self.set_freq(433.0)
        self.set_spreading_factor(10)

#gateway start
    def start(self):
        print("Pi LoRaWan Gateway Start!")
        self.reset_ptr_rx()
        #check GPS enstable
        report = session.next()
        while report['class'] != 'TPV':
            report = session.next()
        while report.mode != 3:
            print('Waiting for GPS')
            report = session.next()
            while report['class'] != 'TPV':
                report = session.next()

        self.set_mode(MODE.RXCONT)
        while True:
            sleep(.5)
            rssi_value = self.get_rssi_value()
            status = self.get_modem_status()
            sys.stdout.flush()

            report = session.next()
            while report['class'] != 'TPV':
                report = session.next()
            date = report.time[0:4] + ',' + report.time[5:7] + ',' + report.time[8:10] + ','
            gps_time = str(int(report.time[11:13]) + 8) + ',' + report.time[14:16] + ',' + report.time[17:19] + ','
            data_time = date + gps_time + str(report.lon) + ',' + str(report.lat) + ',' + str(report.alt) + ","
            
            if int(report.time[11:13]) >= 24:
                int(report.time[11:13]) - 24
                
            #gateway get self sensor data and restore
            if ser.in_waiting > 0:
                line = ser.readline()
                self.gateway_data = data_time + line.decode("utf-8", 'ignore')[:-2]
                print(self.gateway_data)
                self.save_gateway_data(self.gateway_data)

    #open file to save gateway self sensor data            
    def save_gateway_data(self, data):
        print("Creat sensor data dir")
        today = str(time.localtime()[0])
        for date in time.localtime()[1:3]:
            today += "-"
            today += str(date)

        #save file
        print("open sensor dir")
        gateway_id = str(0xe1)
        if gateway_id == '225':
            gateway_id = 'Gateway'
            
        os.system('[ ! -d "/home/pi/Documents/sensor_data/" ] && sudo mkdir "/home/pi/Documents/sensor_data/"')
        os.system('[ ! -d "/home/pi/Documents/sensor_data/' + gateway_id + '" ] && sudo mkdir "/home/pi/Documents/sensor_data/' + gateway_id + '"')
        dir_path = "/home/pi/Documents/sensor_data/" + gateway_id + "/"
        save_path = dir_path + today
        print(save_path)
        data_file = open(save_path, 'a')
        print(data_file)

        # check data is different from last data
        temp_path = dir_path + "tmp"
        open(temp_path, "a").close()
        temp_file = open(temp_path, "r")
        old_data = temp_file.read()
        old_data = old_data[:-1]
        temp_file.close()
        if old_data == data:
            return 0

        # get date string for saving the save time
        print("Getting date & time")
        time_str = str(time.localtime()[3])
        for now_time in time.localtime()[4:6]:
            time_str += ":"
            time_str += str(now_time)

        # murge sensor data and saving time
        write_date = today + " " + time_str + " "
        print("Saveing data\n")
        data_file.write(write_date)
        data_file.write(data + "\r\n")
        data_file.close()

        # save last data to temp file
        temp_file = open(temp_path, 'w')
        temp_file.write(data + "\r\n")
        temp_file.close()

    def crc_check(self, receive: Receive_Data):
        data_crc = bytearray(hex(binascii.crc32(receive.data))[2:].encode())
        if len(data_crc) < 8:
            for i in range(8 - len(data_crc)):
                data_crc = bytearray("0".encode()) + data_crc

        receive.crc_code = binascii.b2a_hex(receive.crc_code)
        return data_crc == receive.crc_code

    def on_rx_done(self):
        print("\nReceived: ")
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)
        print(payload)
        print(bytes(payload).decode("utf-8", 'ignore'))
        if payload[0] == self.gateway_id:
            receive = Receive_Data(payload)
            print("checking CRC")
            if self.crc_check(receive):
                receive.data = receive.data[:-2].decode("utf-8", 'ignore')
                print("CRC check success")
                if receive.data == "KA" or receive.data == "READY":
                    receive.TX_string("CMD_AC")
                    self.lora_send_with_crc(receive)
                    print("Sand CMD_AC")
                elif receive.data == "GW":
                    print("Waiting for GPS")
                elif receive.data == "GK":
                    print("GPS Ready")

                else:
                    receive.TX_string("RECEIVE")
                    self.save_data(receive)
                    self.lora_send_with_crc(receive)
                    print("Sand RECEIVE")
            else:
                print("CRC check fail")
                print("CRC: ", receive.crc_code)
                print("Data_CRC: ", bytearray(hex(binascii.crc32(receive.data))[2:].encode()))
                receive.data = "CRC check fail"
                self.save_data(receive)

        else:
            print("Message not for me")
            print("Local_ID: ", self.gateway_id)
            print("Message_ID: ", payload[0])

        self.set_mode(MODE.RXCONT)

    def on_tx_done(self):
        print("\nSandout: ")
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def lora_send_with_crc(self, output: Receive_Data):
        output.out_data += "\r\n"
        crc = binascii.crc32(output.out_data.encode())
        crc1 = (crc & 0xff000000) >> 24
        crc2 = (crc & 0xff0000) >> 16
        crc3 = (crc & 0xff00) >> 8
        crc4 = (crc & 0xff)
        TX_data = bytes([output.node_id]) + bytes([self.gateway_id]) + bytes([crc1]) + bytes([crc2]) + bytes(
            [crc3]) + bytes([crc4]) + output.out_data.encode()
        self.write_payload(list(TX_data))
        self.set_mode(MODE.TX)

    def save_data(self, data: Receive_Data):
        n_id = str(data.node_id)
        n_data = data.data

        #change file's name
        if n_id == '209':
            n_id = 'Node4'
        elif n_id == '193':
            n_id = 'Node3'
        elif n_id == '177':
            n_id = 'Node2'
        elif n_id == '161':
            n_id = 'Node1'

        # open file to save sensor data
        print("Creat sensor data dir")
        today = str(time.localtime()[0])
        for date in time.localtime()[1:3]:
            today += "-"
            today += str(date)

        print("open sensor dir")
        # save file
        os.system('[ ! -d "/home/pi/Documents/sensor_data/" ] && sudo mkdir "/home/pi/Documents/sensor_data/"')
        os.system(
            '[ ! -d "/home/pi/Documents/sensor_data/' + n_id + '" ] && sudo mkdir "/home/pi/Documents/sensor_data/' + n_id + '"')
        dir_path = "/home/pi/Documents/sensor_data/" + n_id + "/"
        save_path = dir_path + today
        print(save_path)
        data_file = open(save_path, 'a')
        print(data_file)

        # check data is different then last data
        temp_path = dir_path + "tmp"
        open(temp_path, "a").close()
        temp_file = open(temp_path, "r")
        old_data = temp_file.read()
        old_data = old_data[:-1]
        temp_file.close()
        if old_data == n_data:
            return 0

        # get date string for saving the save time
        print("Getting date & time")
        time_str = str(time.localtime()[3])
        for now_time in time.localtime()[4:6]:
            time_str += ":"
            time_str += str(now_time)

        # murge sensor data and saving time
        write_date = today + " " + time_str + "  "
        print("Saveing data")
        data_file.write(write_date)
        data_file.write(n_data + "\r\n")
        data_file.close()

        # save last data to temp file
        if n_data == "CRC check fail":
            return 0
        temp_file = open(temp_path, 'w')
        temp_file.write(n_data + "\r\n")
        temp_file.close()


lora = LoRaRcvCont(verbose=False)
lora.set_mode(MODE.STDBY)

lora.set_pa_config(pa_select=1)

try:
    lora.start()
except KeyboardInterrupt:
    sys.stdout.flush()
    print("")
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print("")
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()