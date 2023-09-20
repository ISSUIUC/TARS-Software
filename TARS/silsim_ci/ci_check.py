import csv
import os
import traceback
import ci_config as config
import copy
import pandas as pd
import matplotlib.pyplot as plt
# This script determines if the CI should pass or fail given a simulation_output csv.
# This script will also output a log of all the actions that happened during the simulated launch

# Logging
log_output = "ci_log.txt"

# Array of strings that are the logs
logs = []

# Clears the log_output file to rewrite to before dumping logs
def clear_file():
    f = open(os.path.join(os.path.dirname(__file__), "./" + log_output), "w")
    f.close()

# Inserts text to the log output file
# @param test the text to outut to the file
def insert_file(text):
    f = open(os.path.join(os.path.dirname(__file__), "./" + log_output), "a")
    f.write(text + "\n")
    f.close()

# Writes everything in the log to the log_output file
def insert_logs():
    f = open(os.path.join(os.path.dirname(__file__), "./" + log_output), "a")
    if(len(logs) > 0):
        f.write("=================== LOGFILE ===================\n")
        for log in logs:
            f.write(log + "\n")
        f.write("===================   END   ===================")
        
    else:
        f.write("\n(Process outputted no logs)")
    print("Logs saved to \033[96m" + log_output + "\033[0m")
    
    f.close()

# Logs test in a certain manner to both the log and terminal
# @param logtext text to output to the log
# @param key tag to define the text, default to main
def log(logtext, key="main"):
    logs.append("<" + key + ">  " + str(logtext))
    print("\033[90m[" + key + "]  " + str(logtext) + "\033[0m")

# CI success/fail methods, automatically called if no exceptions
def success():
    print("\n\nSILSIM CI run \033[92mSUCCESS\033[0m")
    clear_file()
    insert_file("RUN SUCCESS")
    insert_file("SILSIM CI Exited with code 0 [No error]")
    insert_logs()
    print("\n")
    exit(0)

# Fails and exiits the program after outputting to file
# @param msg the message to display on fail
def fail(msg=None):
    text_output = ""
    if msg == None or msg == str(None):
        text_output = traceback.format_exc()
    else:
        text_output = msg
        
    log(text_output, "FATAL")
    log("CI Abort", "FATAL")
    print("\033[93m" + str(msg) + "\033[0m")
    print("\n\nSILSIM CI run \033[91mFAIL\033[0m")
    clear_file()
    insert_file("RUN FAIL")
    insert_file("SILSIM CI Exited with code 255 [Generic error]")
    insert_file("\n\nTraceback:\n" + traceback.format_exc() + "\n\n")
    insert_logs()
    print("\n")
    exit(255)

# Actual ci logic
# Turns number of ticks into number if ms
# @param tick the number of ticks
# @return that amout of ticks in milliseconds
def ticks_to_ms(tick):
    return ((tick * 1000) + 10000 - 1)/10000

# add_branch function from https://stackoverflow.com/questions/30880973/parse-a-dot-seperated-string-into-dictionary-variable
# Creates a dictionary in the format as defined below 
# @param tree the original dictionary
# @param vector the current structure to be created in the dictionary
# @param value the value to be stored in the dictionary
def add_branch(tree, vector, value):
    """
    Given a dict, a vector, and a value, insert the value into the dict
    at the tree leaf specified by the vector.  Recursive!

    Params:
        data (dict): The data structure to insert the vector into.
        vector (list): A list of values representing the path to the leaf node.
        value (object): The object to be inserted at the leaf

    Example 1:
    tree = {'a': 'apple'}
    vector = ['b', 'c', 'd']
    value = 'dog'

    tree = add_branch(tree, vector, value)

    Returns:
        tree = { 'a': 'apple', 'b': { 'c': {'d': 'dog'}}}

    Example 2:
    vector2 = ['b', 'c', 'e']
    value2 = 'egg'

    tree = add_branch(tree, vector2, value2)    

    Returns:
        tree = { 'a': 'apple', 'b': { 'c': {'d': 'dog', 'e': 'egg'}}}

    Returns:
        dict: The dict with the value placed at the path specified.

    Algorithm:
        If we're at the leaf, add it as key/value to the tree
        Else: If the subtree doesn't exist, create it.
              Recurse with the subtree and the left shifted vector.
        Return the tree.

    """
    key = vector[0]
    tree[key] = value \
        if len(vector) == 1 \
        else add_branch(tree[key] if key in tree else {},
                        vector[1:],
                        value)
    return tree

# Takes the header and turns it into a dictionary of packet structure
# @param csv_header_line the header line
# @return a tuple of the format dictionary and the subcategories in an array
def construct_packet_format(csv_header_line):
    packet_format = {}
    packet_format_list = []
    for header_str in csv_header_line:
        trace = header_str.split(".")
        packet_format_list.append(trace)
        packet_format = add_branch(packet_format, trace, None)
    return (packet_format, packet_format_list)
    

# Creates a packet dictionary from the format and the data
# @param packet_format the template of the packet to create the format of the packet with
# @param packet_format_list each datastrings structure to add into the dictionary
# @return the packet in the correct format with all the data
def construct_packet_from_format(packet_format, packet_format_list, csv_line):
    packet = packet_format
    cur_token = 0 # Which piece of data are we on in the current line?
    for str in csv_line:
        this_format = packet_format_list[cur_token]
        add_branch(packet, this_format, str)
        cur_token += 1
        
    return packet

# Gets the dictionary key for the timestamp value in a certain sensor's data dictionary
# @param silsim_packet the packet in which to check for the timestamp
# @param datastring current sensor category to get timestamp key in
# @return key in the dictionary that contains timestamp data
def get_timestamp_packet_key(silsim_packet, datastring):
    # Find the timestamp key (they are all different for some reason)
    for key in list(silsim_packet[datastring].keys()):
        if(key.lower().startswith("timestamp")):
            return key

# Gets the difference in timestamps between current stored and the one in the packet
# @param current_timestamp the current store timestamp
# @param silsim_packet the current packet to check timestamp with
# @param datastring current sensor category to get timestamp difference in
# @return the absolute value of the difference between current stored and packet timestamp
def get_packet_timestamp_diff(current_timestamp, silsim_packet, datastring):
    packet_key = get_timestamp_packet_key(silsim_packet, datastring)
    data_timestamp = int(silsim_packet[datastring][packet_key])
    return abs(data_timestamp - current_timestamp)

# Makes sure current stored timestamp and packet timestamp are within ten seconds, fails simulation if not
# @param current_state tuple with current state and timestamp dictionaries
# @param silsim_packet newest packet to update current_state with
# @param datastring sensor category to check timestamp complying in
def check_timestamp_diff_complies(current_state, silsim_packet, datastring):

    if(not test_data_exists(silsim_packet, datastring)):
        return

    current_timestamp = 0

    if config.data_is_processed:
        # If the data is processed, we want to set min timestamp to when sim starts.
        current_timestamp = config.data_start_timestamp

    if(current_state['last_seen'].get(datastring) != None):
        current_timestamp = int(current_state['last_seen'][datastring])
    
    diff = ticks_to_ms(get_packet_timestamp_diff(current_timestamp, silsim_packet, datastring))
    if(int(diff * config.SECONDS_TO_MS) > config.check_timeouts_length):
        print(current_state['last_seen'])
        print(datastring)
        print(silsim_packet[datastring]['timestamp'])
        fail("Packet data timeout for " + datastring + " on timestamp " + str(current_timestamp) + "  (simulation time " + str(ticks_to_ms(current_timestamp) * config.SECONDS_TO_MS) + " seconds)\nPacket timed out with time " + str((diff * config.SECONDS_TO_MS)) + "s, fail condition set to " + str(config.check_timeouts_length) + "s")

# Parses packet for all of the sensor categories
# @param current_state tuple with current state and timestamp dictionaries
# @param silsim_packet newest packet to update current_state with
def parse_packet(current_state, silsim_packet):
    if(config.check_timeouts):
        for category_value in config.all_data_values:
            if category_value != "state_data":
                check_timestamp_diff_complies(current_state, silsim_packet, category_value)

# Checks if a certain datastring has data in a packet
# @param silsim_packet the newest packet to check existence of data
# @param datastring the sensor category to check if data exists
# @return boolean that reuturns true if the data exists and false if not
def test_data_exists(silsim_packet, datastring):
    try:
        return silsim_packet["has_" + datastring] == "True" or silsim_packet["has_" + datastring] == "1" or silsim_packet["has_" + datastring] == True
    except:
        return False
    
# Gets the timestamp from a packet
# @param silsim_packet the newest packet to get the timestamp from
# @param datastring the sensor from which to get the timestamp
# @return the timestamp from the packet in the sensor data or -1 if it cannot be found
def get_timestamp(silsim_packet, datastring):
    if test_data_exists(silsim_packet, datastring):
        timestamp_key = get_timestamp_packet_key(silsim_packet, datastring)
        return silsim_packet[datastring][timestamp_key]
    return -1

# Updates current_state tuple dictionaries with newest state and timestamps
# @param current_state tuple with current state and timestamp dictionaries
# @param silsim_packet newest packet to update current_state with
# @param datastring the sensor in which to update data
def update_current_state_values(current_state, silsim_packet, datastring):
    if test_data_exists(silsim_packet, datastring):
        current_state["loaded_state"][datastring] = silsim_packet[datastring].copy()

    this_seen = get_timestamp(silsim_packet, datastring)
    if(this_seen != -1):
        current_state['last_seen'][datastring] = this_seen

# Converts a timestamp from milliseconds to seconds
# @param timestamp timestamp in milliseconds
# @return timestamp in seconds
def get_timestamp_seconds(timestamp):
    return ticks_to_ms(timestamp) * config.SECONDS_TO_MS

# Returns timestamp with a t infront of it for logging uprposes
# @param timestamp timestamp to return with a t
# @return timestamp with t added infront of it
def get_sim_timestamp_string(timestamp):
    return "t+" + str(round(get_timestamp_seconds(timestamp), 2))

# Updates both the current data and timestamp in current_state from the silsim_packet for eact data sensor
# @param current_state the tuple of state and timestamp dictionary
# @param silsim_packet current packet to update current_state with
def load_packet_into_state(current_state, silsim_packet):
    for data_category in config.all_data_values:
        update_current_state_values(current_state, silsim_packet, data_category)
    

# Checks if first and second values are the same, fails simulation if they are different
# @param assert_text test to be dispalyed to user when fail or pass
# @param real one of the values, got from the state of the rocket
# @param expected the value expected to be had
def assert_equal(assert_text, real, expected):
    if(real != expected):
        fail(assert_text + ": assert_equal failed, expected value '" + str(expected) + "' but got '" + str(real) + "'")
    else:
        log(assert_text + ": PASS", "assert-equal")

# Checks if the Finite State Machine and the success final state value are the same
# @param state a tuple of two dictionaries. The first dictionary has key, value pairs of data name and data
#              the second dictionary has key, value pairs of data name and most recent timestamp
def post_launch(state):
    log("SILSIM parse complete", "post-flight")
    # After-run checks:
    # Check that FSM state is landed.
    if(config.check_fsm):
        fsm_state = rocketstate_to_int(state['loaded_state']['rocketState_data'])
        check_state = config.check_fsm_final_state
        assert_equal("Check FSM state after simulation end", fsm_state, check_state)

# Determine the latest time that the packet might have been sent
# @param silsim_packet the current packet to find the timestamp of
# @return the newest timestamp that is found
def get_timestamp_for_packet(silsim_packet):
    max_timestamp = -1
    for key in silsim_packet:
        if(type(silsim_packet[key]) is dict):
            timestamp_key = get_timestamp_packet_key(silsim_packet, key)
            if(silsim_packet[key][timestamp_key]):
                if(int(silsim_packet[key][timestamp_key]) > max_timestamp):
                    max_timestamp = int(silsim_packet[key][timestamp_key])
    return max_timestamp

def rocketstate_to_int(rocketstate_packet):
    rocketstate_packet_key = "rocketStates"
    if(config.data_is_processed):
        rocketstate_packet_key = "rocketState"
    try:
        evaluated = eval(rocketstate_packet[rocketstate_packet_key])
    except:
        evaluated = rocketstate_packet[rocketstate_packet_key]
    if(type(evaluated) is list):
        st = evaluated[0]
        return config.rocket_states.index(st)
    if(type(evaluated) is int):
        return int(rocketstate_packet[rocketstate_packet_key])
    if(type(evaluated) is str):
        return config.rocket_states.index(evaluated)

# NOT COMPLETE
# Compares previous and current packets and updates data accordingly in log
# @param last the previous data packet
# @param cur the current data packet
def compare_packets(last, cur):
    timestamp = get_timestamp_for_packet(cur)
    timestamp_string = get_sim_timestamp_string(timestamp)

    if(test_data_exists(cur, "rocketState_data")):
        rocket_state = rocketstate_to_int(cur['rocketState_data'])
        
        if(last.get('rocketState_data') == None):
            log("FSM State change from 'None' to " + str(rocket_state), timestamp_string)
        else:
            last_state = rocketstate_to_int(last['rocketState_data'])
            
            if(last_state != int(rocket_state)):
                state_diff = abs(last_state - int(rocket_state))
                log("FSM State change from '" + str(last_state) + "' to '" + str(rocket_state) + "'", timestamp_string)
                if(state_diff > 1):
                    cur_state_str = str(last_state)
                    fail(f"Fail by FSM state jump: State changed from '{cur_state_str}' to '{str(rocket_state)}', but expected '{str(last_state)}' => '{str(last_state+1)}' OR '{str(last_state)}' => '{str(last_state-1)}'")
                

# Mass-map processed packet data to raw data
# @param processed_data_dict list of data fields to add to data_dict, maps processed data => name for unprocessed
# @param data_dict dictionary to add data to
# @param processed_packet the processed packet to retrieve data from
# @return the correct format dictionary for each datastring
def map_processed_data_field(processed_data_dict, processed_packet):
    new_dict = {}

    for data_field_key in list(processed_data_dict.keys()):
        new_dict[processed_data_dict[data_field_key]] = processed_packet[data_field_key]
    
    return new_dict

# Given a packet from a processed data file, return a packet as it would come from SILSIM data.
# @param packet The processed packet
# @return the processed packet into the raw file
def packet_to_raw(packet):
    new_packet = {}
    for datastring in config.all_data_values:
        new_packet["has_" + datastring] = True
        new_packet[datastring] = {}

    new_packet['lowG_data'] = map_processed_data_field({'ax': 'ax', 'ay': 'ay', 'az': 'az', 'gx': 'gx', 'gy': 'gy', 'gz': 'gz', 'timestamp_ms': 'timeStamp_lowG'}, packet)
    new_packet['highG_data'] = map_processed_data_field({'highg_ax': 'ax', 'highg_ay': 'ay', 'highg_az': 'az', 'timestamp_ms': 'timeStamp_highG'}, packet)
    new_packet['gps_data'] = map_processed_data_field({'latitude': 'latitude', 'longitude': 'longitude', 'position_lock': 'posLock', 'satellite_count': 'siv_count', 'timestamp_ms': 'timeStamp_GPS'}, packet)
    new_packet['barometer_data'] = map_processed_data_field({'temperature': 'temperature', 'pressure': 'pressure', 'barometer_altitude': 'altitude', 'timestamp_ms': 'timeStamp_barometer'}, packet)
    new_packet['state_data'] = map_processed_data_field({'kalman_pos_x': 'x', 'kalman_vel_x': 'vx', 'kalman_acc_x': 'ax', 'kalman_apo': 'apo', 'timestamp_ms': 'timeStamp_state'}, packet)
    new_packet['voltage_data'] = map_processed_data_field({'voltage_battery': 'battery_voltage', 'timestamp_ms': 'timestamp'}, packet)
    new_packet['rocketState_data'] = map_processed_data_field({'state': 'rocketState', 'timestamp_ms': 'timestamp'}, packet)

    return new_packet

# Runs through all packets in the csv data and processes them
def main():
    # Load in the file
    csv_rows = None
    log("Loading silsim output")
    try:
        with open(os.path.join(os.path.dirname(__file__), "./silsim_output.csv")) as csv_file:
            csv_rows = csv.reader(csv_file, delimiter=',')
            cur_line = 0
            packet_format = None
            packet_format_list = None

            last_packet = None

            # Initial state
            # Seperate dictionaries for loaded_state and last_seen timestamps to maintain loaded_state with packet structure
            cur_state = {'loaded_state': {}, 'last_seen': {}}


            log("Running silsim parser")

            for row in csv_rows:
                if cur_line == 0:
                    (packet_format, packet_format_list) = construct_packet_format(row)
                    # Line 0 will have all the headers for the csv, we want to construct a dict using the headers.
                    cur_line += 1
                    continue
                
                packet = construct_packet_from_format(packet_format, packet_format_list, row)
                if cur_line == 1:
                    last_packet = copy.deepcopy(cur_state["loaded_state"])

                # This line is only used for converting processed data back into a raw-ish format:
                if config.data_is_processed:
                    packet = packet_to_raw(packet)
                parse_packet(cur_state, packet)
                load_packet_into_state(cur_state, packet)

                compare_packets(last_packet, packet)

                last_packet = copy.deepcopy(cur_state["loaded_state"])
                cur_line += 1
            post_launch(cur_state)
    except SystemExit:
        print("Process exited with error code 255")
        exit(255)
    except:
        fail()


    log("SILSIM CI run complete")
    success()

if __name__ == "__main__":
    main()