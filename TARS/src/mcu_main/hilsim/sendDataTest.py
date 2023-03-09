import serial
import pandas

import time

ser = serial.Serial("COM8", 9600, timeout=20)

csv = pandas.read_csv('flight_computer.csv')

written = ["highg_ax", "highg_ay", "highg_az", "barometer_altitude", "temperature", "pressure", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "mx", "my", "mz"]

# last = csv.iloc[0]["timestamp"]

for _, row in csv.iterrows():
    line = f"{','.join(str(item) for item in row[written])}\n"

    # print(line)
    ser.write(line.encode("utf8"))

    data = ser.read_all()
    string = data.decode("utf8")
    if string != "":
        print(string)

    time.sleep(1/1000)
