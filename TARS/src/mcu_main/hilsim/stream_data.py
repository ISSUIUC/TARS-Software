import serial
import time


ser = serial.Serial("COM8", 9600, timeout=10)

csv = open("raw_parsed.csv", "r")
csv.readline()

while(True):
    data = ser.read_all()
    string = data.decode("utf8")
    if string == "begin":
        break
    elif string:
        print(string)
    

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
            line = csv.readline()
            line_num += 1
            if not line:
                break
            ser.write((line + '\n').encode("utf8"))
   
    data = ser.read_all()
    string = data.decode("utf8")
    
    if string:
        string = string[0 : (len(string)-1)]
        print(string, "")
