import serial
import time
import os
import serial.tools.list_ports

serial_port = "COM3"

def get_ports():
    return serial.tools.list_ports.comports()

def validate_ports():
    ports = get_ports()
    if(len(ports) == 0):
        return "No devices are plugged in. Please plug TARS into a serial port"
    return "OK"

def run_hilsim(raw_csv):
    print("Run hilsim request recieved")
    ser = serial.Serial(serial_port, 9600, timeout=10, write_timeout=1)
    hilsim_return_log = ""
    csv_lines = raw_csv.split("\n")
    cur_line = 0

    print("CSV parsed")    

    last_time = time.time()*1000
    start_time = last_time
    first_line = csv_lines[0]

    approx_time_sec = (len(csv_lines) * 10)/1000 + 10
    print("Approximate runtime: " + str(approx_time_sec) + "s")
    print("Awaiting serial connection (10s)...")

    watchdog_start = time.time()

    while(cur_line < len(csv_lines)):      
        if(abs(watchdog_start - time.time()) > 3):
            print("we should abort")
            return hilsim_return_log

        if time.time()*1000 > last_time + 10:
            last_time += 10
            if time.time()*1000 < start_time + 10000:
                ser.write((first_line + '\n').encode("utf8"))
            else:
                line = csv_lines[cur_line][:-1] + ",0,0,0"
                cur_line += 1
                if not line:
                    return hilsim_return_log
                try:
                    ser.write((line + '\n').encode("utf8"))
                except:
                    return hilsim_return_log
        if ser.in_waiting:
            data = ser.read_all()
            string = data.decode("utf8")
            
            if string:
                watchdog_start = time.time()
                string = string[0 : (len(string)-1)]
                hilsim_return_log += string + "\n"
           

    return hilsim_return_log


# Immediately execute data streaming if this is main
if __name__ == "__main__":
    ser = serial.Serial(serial_port, 9600, timeout=10)
    csv = open(os.path.join(os.path.dirname(__file__), "./flight_computer.csv"), "r")
    csv.readline()
    time.sleep(5)
    print("intialized")

    last_time = time.time()*1000
    start_time = last_time
    first_line = csv.readline()
    line_num = 0

    while(True):       
        if time.time()*1000 > last_time + 10:
            last_time += 10
            if time.time()*1000 < start_time + 10000:
                pass
                ser.write((first_line + '\n').encode("utf8"))
            else:
                line = csv.readline()[:-1] + ",0,0,0"
                line_num += 1
                if not line:
                    break
                ser.write((line + '\n').encode("utf8"))
                print(line)
        if ser.in_waiting:
            data = ser.read_all()
            string = data.decode("utf8")
            
            if string:
                string = string[0 : (len(string)-1)]
                print(string, "")
