import serial
import time
import os

serial_port = "/dev/tty.usbmodem132228101"


def run_hilsim(raw_csv):
    print("Run hilsim request recieved")
    # ser = serial.Serial(serial_port, 9600, timeout=10)
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
    while(cur_line < len(csv_lines)):       
        if time.time()*1000 > last_time + 1:
            last_time += 1
            if time.time()*1000 < start_time + 10000:
                pass
                # ser.write((first_line + '\n').encode("utf8"))
            else:
                print(str(cur_line) + " / " + str(len(csv_lines)) + " packets simulated [time: "+ str(round((cur_line/100)+10, 2)) + "s]                                  ", end="\r")
                line = csv_lines[cur_line][:-1] + ",0,0,0"
                cur_line += 1
                if not line:
                    break
                # ser.write((line + '\n').encode("utf8"))
        if True: #ser.in_waiting:
            #data = ser.read_all()
            string = "test dataa"
            #string = data.decode("utf8")
            
            if string:
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
