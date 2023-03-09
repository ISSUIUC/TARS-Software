import serial
import pandas

import time

# n means that the flight will be played n times slower. So 1 means real time, 2 means half speed, etc.
SPEED_FACTOR = 2.0
# n means that only 1 out of every n packets will be sent. Lower is more granular (min 1, must be integer).
GRANULARITY_FACTOR = 1

ser = serial.Serial("COM8", 9600, timeout=10, write_timeout=10)

csv = pandas.read_csv('flight_computer.csv')

written = ["highg_ax", "highg_ay", "highg_az", "barometer_altitude", "temperature", "pressure", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "mx", "my", "mz"]

string = ""

for i, row in csv.iterrows():
    if i % GRANULARITY_FACTOR != 0:
        continue

    start = time.time()
    line = f"{','.join(str(item) for item in row[written])}\n"

    ser.write(line.encode("utf8"))

    data = ser.read_all()
    decoded = data.decode("utf8")
    if decoded != "":
        string = decoded
    curr = time.time()

    sleep_time = min(max(0.0, 10 / 1000 * GRANULARITY_FACTOR * SPEED_FACTOR - (curr - start)), 10 / 1000 * GRANULARITY_FACTOR * SPEED_FACTOR)

    if i % 7 == 0:
        print(f"\rLoop {i}, Sleep Time: {sleep_time*1000:.2f} ms, Recent String: {string!r}", end="", flush=True)

    time.sleep(sleep_time)
