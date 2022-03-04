from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD
import binascii
import os
import time
import sys
import gps
import serial

# serial port setup
ser = serial.Serial('/dev/ttyACM0', 9600)

# lora hat pin setup
BOARD.setup()

# GPS setup
session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

# data type
class Receive_Data():
    def __init__(self, receive=[0, 0, 0, 0, 0, 0, 0]):
        self.ID = receive[1] 
        self.crc_code = bytes(receive[2:6]) #crc_code
        self.data = bytes(receive[6:]) # length of data
        self.out_data = ""

    def TX_string(self, data: str):
        self.out_data = data

# lora class
class LoRaRcvCont(LoRa):
    local_id = 0xf1
    gateway_id = 0xe1
    gateway = Receive_Data([local_id, gateway_id, 0, 0, 0, 0, 0])
    activte = False
    receive = False
    data = ""

# lora setup
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)
        self.set_freq(433.0)
        self.set_spreading_factor(10)

# node start
    def start(self):
        print("Pi LoRaWan Node Start!")
        self.reset_ptr_rx()
        #check GPS before send data
        report = session.next()
        while report['class'] != 'TPV': #GPS type
            report = session.next()
        while report.mode != 3:         #GPS format
            self.gateway.TX_string("GW")
            self.lora_send_with_crc(self.gateway)
            print("Waiting for GPS")
            report = session.next()
            while report['class'] != 'TPV' :
                report = session.next()

        self.gateway.TX_string("READY")
        self.lora_send_with_crc(self.gateway)
        self.set_mode(MODE.TX)
        last_time = time.time()
        timeout = time.time()
        
        while True:
            report = session.next()
            while report['class'] != 'TPV':
                report = session.next()
            while report.mode != 3:
                self.gateway.TX_string("GW")
                self.lora_send_with_crc(self.gateway)
                print("Waiting for GPS again")
                report = session.next()
                while report['class'] != 'TPV' :
                    report = session.next()
                
            #GPS compensate
            hour = int(report.time[11:13])
            day = int(report.time[8:10])
            hour += 8
            if hour >= 24:
                hour -= 24
                day += 1
            
            date = report.time[0:4] + "," +  report.time[5:7] + "," + str(int(day)) + ","
            gps_time = str(int(hour)) + "," + report.time[14:16] + "," + report.time[17:19] +  ","
            data_time = date + gps_time + str(report.lon) + "," + str(report.lat) + "," + str(report.alt) + ","

            if not self.activte:
                if time.time() - last_time > 9:
                    self.gateway.TX_string("KA")
                    self.lora_send_with_crc(self.gateway)
                    print("KA")
                    self.set_mode(MODE.RXCONT)
                    last_time = time.time()
            else:
                if  ser.in_waiting >0:
                    line = ser.readline()
                    self.data = data_time + line.decode("utf-8", 'ignore')[:-2]
                    self.gateway.TX_string(self.data)
                    self.lora_send_with_crc(self.gateway)
                    self.receive = False
                    print(self.data)
                    self.save_data(self.data)
                    last_time = time.time()
                    timeout = time.time()
                    self.set_mode(MODE.RXCONT)
                    
                elif time.time() - timeout > 2 and not self.receive:
                    self.gateway.TX_string(self.data)
                    self.lora_send_with_crc(self.gateway)
                    timeout = time.time()
                    self.set_mode(MODE.RXCONT)
                    
                elif self.receive and time.time() - timeout >59:
                    self.gateway.TX_string("serial no data")
                    self.lora_send_with_crc(self.gateway)
                    timeout = time.time()
                    self.set_mode(MODE.RXCONT)

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
        if payload[0] == self.local_id:
            receive = Receive_Data(payload)
            print("checking CRC")
            if self.crc_check(receive):
                receive.data = receive.data[:-2].decode("utf-8", 'ignore')
                print("CRC check success\n")
                if receive.data == "CMD_AC":
                    self.activte = True
                    self.runtime = 0
                elif receive.data == "RECEIVE":
                    self.receive = True
            else:
                print("CRC check fail")
                print("CRC: ", receive.crc_code)
                print("Data_CRC: ", bytearray(hex(binascii.crc32(receive.data))[2:].encode()))

        else:
            print("Message not for me")
            print("Local_ID: ", self.local_id)
            print("Message_ID: ", payload[0])

        self.set_mode(MODE.STDBY)

    def on_tx_done(self):
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)
        sys.stdout.flush()
        self.set_mode(MODE.RXSINGLE)

    def on_cad_done(self):
        print("\non_CadDone")
        print(self.get_irq_flags())

    def on_rx_timeout(self):
        print("\non_RxTimeout")
        print(self.get_irq_flags())

    def on_valid_header(self):
        print("\non_ValidHeader")
        print(self.get_irq_flags())

    def on_payload_crc_error(self):
        print("\non_PayloadCrcError")
        print(self.get_irq_flags())

    def lora_send_no_crc(self, output_data: Receive_Data):
        TX_data = bytes([output_data.ID]) + bytes([self.local_id]) + bytes([1]) + bytes(
            [len(output_data.out_data)]) + output_data.out_data.encode()
        self.write_payload(list(TX_data))
        self.set_mode(MODE.TX)

    def lora_send_with_crc(self, output: Receive_Data):
        output.out_data += "\r\n"
        crc = binascii.crc32(output.out_data.encode())
        crc1 = (crc & 0xff000000) >> 24
        crc2 = (crc & 0xff0000) >> 16
        crc3 = (crc & 0xff00) >> 8
        crc4 = (crc & 0xff)
        TX_data = bytes([output.ID]) + bytes([self.local_id]) + bytes([crc1]) + bytes([crc2]) + bytes(
            [crc3]) + bytes([crc4]) + output.out_data.encode()
        self.write_payload(list(TX_data))
        self.set_mode(MODE.TX)
        print("Send: ",TX_data.decode("utf-8", 'ignore'))

    def save_data(self, data):

        # open file to save sensor data
        print("Creat sensor data dir")
        today = str(time.localtime()[0])
        for date in time.localtime()[1:3]:
            today += "-"
            today += str(date)

        print("open sensor dir")
        # save file
        os.system('[ ! -d "/home/pi/Documents/sensor_data/" ] && sudo mkdir "/home/pi/Documents/sensor_data/"')
        dir_path = "/home/pi/Documents/sensor_data/"
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
        print("Saveing data")
        data_file.write(write_date)
        data_file.write(data + "\r\n")
        data_file.close()

        # save last data to temp file
        temp_file = open(temp_path, 'w')
        temp_file.write(data + "\r\n")
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