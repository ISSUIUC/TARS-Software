import serial

ser = serial.Serial("/dev/tty.usbmodem132228101", 9600, timeout=10)

csv = open("flight_computer.csv", "r")
csv.readline()

while True:
    line = csv.readline()

    if not line:
        break

    line = 'a,' + line[:-1] + ',0,0,0' + '\n'

    # print(line)

    # ser.write(line.encode("utf8"))
    ser.write(("a" + ",5"*18 + '\n').encode("utf-8"))

    data = ser.read_all()
    string = data.decode("utf8")
    if string:
        print(string)

        
