import hilsimpacket_pb2

def csv_line_to_protobuf(parsed_csv):
    
    hilsim_packet = hilsimpacket_pb2.HILSIMPacket()

    hilsim_packet.imu_high_ax = parsed_csv['highg_ax']
    hilsim_packet.imu_high_ay = parsed_csv["highg_ay"]
    hilsim_packet.imu_high_az = parsed_csv["highg_az"]
    hilsim_packet.barometer_altitude = parsed_csv["barometer_altitude"]
    hilsim_packet.barometer_temperature = parsed_csv["temperature"]
    hilsim_packet.barometer_pressure = parsed_csv["pressure"]
    hilsim_packet.imu_low_ax = parsed_csv["ax"]
    hilsim_packet.imu_low_ay = parsed_csv["ay"]
    hilsim_packet.imu_low_az = parsed_csv["az"]
    hilsim_packet.imu_low_gx = parsed_csv["gx"]
    hilsim_packet.imu_low_gy = parsed_csv["gy"]
    hilsim_packet.imu_low_gz = parsed_csv["gz"]
    hilsim_packet.mag_x = parsed_csv["mx"]
    hilsim_packet.mag_y = parsed_csv["my"]
    hilsim_packet.mag_z = parsed_csv["mz"]

    return hilsim_packet.SerializeToString()
