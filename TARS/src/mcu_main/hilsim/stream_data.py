import serial
import time
import os
import serial.tools.list_ports
import pandas
import csv_datastream
import traceback
import io 

# TODO: Automatically check for TARS port
serial_port = "COM3"

def get_ports():
    return serial.tools.list_ports.comports()

def raw_csv_to_dataframe(raw_csv):
    # Get column names
    header = raw_csv.split('\n')[0].split(",")
    csv = "\n".join(raw_csv.split('\n')[1:])
    csvStringIO = io.StringIO(csv)
    return pandas.read_csv(csvStringIO, sep=",", header=None, names=header)

def validate_ports():
    ports = get_ports()
    if(len(ports) == 0):
        return "No devices are plugged in. Please plug TARS into a serial port"
    return "OK"

def get_runtime(raw_csv):
    csv = raw_csv_to_dataframe(raw_csv)
    return (len(csv) * 10)/1000 + 10

def run_hilsim(raw_csv):
    print("Run hilsim request recieved")
    ser = serial.Serial(serial_port, 9600, timeout=10, write_timeout=1)
    hilsim_return_log = ""
    print("intialized")
    csv = raw_csv_to_dataframe(raw_csv)
    csv_list = csv.iterrows()
    print("CSV parsed")    

    last_time = time.time()*1000
    start_time = last_time

    approx_time_sec = (len(csv) * 10)/1000 + 10
    print("Approximate runtime: " + str(approx_time_sec) + "s")
    print("Awaiting serial connection (10s)...")

    watchdog_start = time.time()
    cur_line = 0

    while(True):      
        
        if(abs(watchdog_start - time.time()) > 3):
            print("Watchdog timer tripped")
            return hilsim_return_log
        if time.time()*1000 > last_time + 10:
            last_time += 10
            if time.time()*1000 < start_time + 10000:
                pass
            else:
                cur_line += 1

                print(f"{cur_line} / {len(csv)}")
                line_num, row = next(csv_list, (None, None))
                if line_num == None:
                    print("Ran out of data to send -- HILSIM data stream end")
                    break
                data = csv_datastream.csv_line_to_protobuf(row)
                if not data:
                    print("No data")
                    return hilsim_return_log
                try:
                    ser.write(data)
                except:
                    print("Exception during serial write")
                    print(traceback.format_exc())
                    return hilsim_return_log
        if ser.in_waiting:
            data = ser.read_all()
            string = data.decode("utf8")
            
            if string:
                watchdog_start = time.time()
                string = string[0 : (len(string)-1)]
                hilsim_return_log += string + "\n"
        else:
            watchdog_start = time.time()
    return hilsim_return_log


# Immediately execute data streaming if this is main
if __name__ == "__main__":
    ser = serial.Serial(serial_port, 9600, timeout=10)
    csv = pandas.read_csv('flight_computer.csv')
    time.sleep(5)
    print("intialized")

    last_time = time.time()*1000
    start_time = last_time
    line_num = 0
    csv_list = csv.iterrows()

    while(True):       
        if time.time()*1000 > last_time + 10:
            last_time += 10
            if time.time()*1000 < start_time + 10000:
                pass
                #ser.write(("".join("0"*70)).encode("ascii"))
            else:

                line_num, row = next(csv_list, (None, None))
                if line_num == None:
                    break
                data = csv_datastream.csv_line_to_protobuf(row)
                ser.write(data)
                print(line_num)
        if ser.in_waiting:
            data = ser.read_all()
            string = data.decode("utf8")
            
            if string:
                string = string[0 : (len(string)-1)]
                print(string, "")
