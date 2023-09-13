import csv
import os
import traceback
import ci_config as config
# This script determines if the CI should pass or fail given a simulation_output csv.
# This script will also output a log of all the actions that happened during the simulated launch

# Logging
log_output = "ci_log.txt"
logs = []

def clear_file():
    f = open(os.path.join(os.path.dirname(__file__), "./" + log_output), "w")
    f.close()

def insert_file(text):
    f = open(os.path.join(os.path.dirname(__file__), "./" + log_output), "a")
    f.write(text + "\n")
    f.close();

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

def log(logtext, key="main"):
    logs.append("<" + key + ">  " + str(logtext))
    print("\033[90m[" + key + "]  " + str(logtext) + "\033[0m")

# CI success/fail methods
def success():
    print("\n\nSILSIM CI run \033[92mSUCCESS\033[0m")
    clear_file()
    insert_file("RUN SUCCESS")
    insert_file("SILSIM CI Exited with code 0 [No error]")
    insert_logs()
    print("\n")
    exit(0)

def fail(msg=None):
    if msg == None:
        print("\033[93m" + traceback.format_exc() + "\033[0m")
    else:
        print("\033[93m" + msg + "\033[0m")
    print("\n\nSILSIM CI run \033[91mFAIL\033[0m")
    clear_file()
    insert_file("RUN FAIL")
    insert_file("SILSIM CI Exited with code 255 [Generic error]")
    insert_file("\n\nTraceback:\n" + traceback.format_exc() + "\n\n")
    insert_logs()
    print("\n")
    exit(255)

# Actual ci logic

# add_branch function from https://stackoverflow.com/questions/30880973/parse-a-dot-seperated-string-into-dictionary-variable
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

def construct_packet_format(csv_header_line):
    packet_format = {}
    packet_format_list = []
    for header_str in csv_header_line:
        trace = header_str.split(".")
        packet_format_list.append(trace)
        packet_format = add_branch(packet_format, trace, None)
    return (packet_format, packet_format_list)
    
    

def construct_packet_from_format(packet_format, packet_format_list, csv_line):
    packet = packet_format
    cur_token = 0 # Which piece of data are we on in the current line?
    for str in csv_line:
        this_format = packet_format_list[cur_token]
        add_branch(packet, this_format, str)
        cur_token += 1
        
    return packet

def parse_packet(current_state, silsim_packet):
    return

def test_data_exists(silsim_packet, datastring):
    return silsim_packet["has_" + datastring] == "True" or silsim_packet["has_" + datastring] == "1"

def get_timestamp(silsim_packet, datastring):
    if test_data_exists(silsim_packet, datastring):
        return silsim_packet[datastring]['timestamp']
    return -1

def get_lastseen(current_state, silsim_packet, datastring):
    this_seen = get_timestamp(silsim_packet, datastring)
    if(this_seen != -1):
        current_state['lastseen_' + datastring] = this_seen

def load_packet_key(current_state, silsim_packet, datastring):
    if test_data_exists(silsim_packet, datastring):
        current_state[datastring] = silsim_packet[datastring]

def load_packet_into_state(current_state, silsim_packet):
    # Load the last time that the packet was seen
    get_lastseen(current_state, silsim_packet, "lowG_data")
    get_lastseen(current_state, silsim_packet, "highG_data")
    get_lastseen(current_state, silsim_packet, "gps_data")
    get_lastseen(current_state, silsim_packet, "barometer_data")
    get_lastseen(current_state, silsim_packet, "state_data")
    get_lastseen(current_state, silsim_packet, "voltage_data")
    get_lastseen(current_state, silsim_packet, "rocketState_data")
    
    load_packet_key(current_state['loaded_state'], silsim_packet, "lowG_data")
    load_packet_key(current_state['loaded_state'], silsim_packet, "highG_data")
    load_packet_key(current_state['loaded_state'], silsim_packet, "gps_data")
    load_packet_key(current_state['loaded_state'], silsim_packet, "barometer_data")
    load_packet_key(current_state['loaded_state'], silsim_packet, "state_data")
    load_packet_key(current_state['loaded_state'], silsim_packet, "voltage_data")
    load_packet_key(current_state['loaded_state'], silsim_packet, "rocketState_data")

def assert_equal(assert_text, real, expected):
    if(real != expected):
        fail(assert_text + ": assert_equal failed, expected value '" + str(expected) + "' but got '" + str(real) + "'")
    else:
        log(assert_text + ": PASS", "assert_equal")

def post_launch(state):
    log("SILSIM parse complete", "post-flight")
    # After-run checks:
    # Check that FSM state is landed.
    if(config.check_fsm):
        fsm_state = int(state['loaded_state']['rocketState_data']['rocketState'])
        check_state = config.check_fsm_final_state
        assert_equal("Check FSM state after simulation end", fsm_state, check_state)

# Determine the latest time that the packet might have been sent
def get_timestamp_for_packet(silsim_packet):
    max_timestamp = -1
    for key in silsim_packet:
        if(type(key) is dict):
            if(silsim_packet[key]['timestamp']):
                if(int(silsim_packet[key]['timestamp']) > max_timestamp):
                    max_timestamp = int(silsim_packet[key]['timestamp'])
    return max_timestamp


#def run_packet_checks(state, silsim_packet):
#    if(config.check_timeouts):



def main():
    # Load in the file
    csv_rows = None
    log("Loading silsim output")
    try:
        with open(os.path.join(os.path.dirname(__file__), "./flight_computer.csv")) as csv_file:
            csv_rows = csv.reader(csv_file, delimiter=',')
            cur_line = 0
            packet_format = None
            packet_format_list = None

            # Initial state
            cur_state = {'loaded_state': {}}
            log("Running silsim parser")
            for row in csv_rows:
                if cur_line == 0:
                    (packet_format, packet_format_list) = construct_packet_format(row)
                    # Line 0 will have all the headers for the csv, we want to construct a dict using the headers.
                    cur_line += 1
                    continue
                packet = construct_packet_from_format(packet_format, packet_format_list, row)
                parse_packet(cur_state, packet)
                load_packet_into_state(cur_state, packet)

                run_packet_checks(cur_state, packet)
                
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