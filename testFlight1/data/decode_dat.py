import numpy as np
import struct
import csv
import os

def write_to_file(writer,line):
	writer.writerow(line)


# Get the names of all unconverted DAT files in folder
files = os.listdir()
dats_to_convert = []
for file in files:
	if file.endswith(".DAT"):
		csv_version = file.split('.')[0] + '.CSV'
		if csv_version not in files:
			dats_to_convert.append(file)



for dat_file in dats_to_convert:
	with open(dat_file,'rb') as file:
		data = file.read()
		file.close()

	with open(dat_file.split('.')[0] + '.CSV','w',newline='') as csvfile:
		csvwriter = csv.writer(csvfile,delimiter=',')



		data_split = data.split(b'\r\n')
		write_to_file(csvwriter,data_split[0].decode('ascii').split(','))

		packet_length = 68
		sensor_datapackets = [data_split[1][i:i+packet_length] for i in range(0,len(data_split[1]),packet_length)]
		# print(sensor_datapackets)
		for datapacket in sensor_datapackets:
			data_list = list(struct.unpack('f'*15,datapacket[:60]))

			gps_lock_int = struct.unpack('h',datapacket[60:62])[0]
			if gps_lock_int == 1:
				data_list.append(True)
			elif gps_lock_int == 0:
				data_list.append(False)
			else:
				print('Failed to interpret gps lock data.',gps_lock_int)

			rocket_state_int = struct.unpack('h',datapacket[62:64])[0]
			# print(rocket_state_int)
			data_list.append(rocket_state_int)

			timestamp = struct.unpack('i',datapacket[64:])[0]
			data_list.append(timestamp)


			write_to_file(csvwriter,data_list)


		csvfile.close()




