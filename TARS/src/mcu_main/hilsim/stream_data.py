import serial
import time


ser = serial.Serial("/dev/tty.usbmodem132228101", 9600, timeout=10)

csv = open("flight_computer.csv", "r")
csv.readline()

# while(True):
#     data = ser.read_all()
#     string = data.decode("utf8")
#     if string == "[TARS] Hardware-in-Loop Test Commenced":
#         break
#     elif string:
#         print(string)

time.sleep(5)

print("intilized")


last_time = time.time()*1000
start_time = last_time
first_line = csv.readline()
line_num = 0

while(True):       
    if time.time()*1000 > last_time + 10:
        last_time += 10
        if time.time()*1000 < start_time + 10000:
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
