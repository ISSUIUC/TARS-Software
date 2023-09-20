## SILSIM CI configuration file
# Timestamp conversion
TICKS_TO_MS = 100
SECONDS_TO_MS = 0.001

# Fail conditions
check_timeouts = True
check_timeouts_length = 10

# Pass conditions
check_fsm = True # Check FSM final state
check_fsm_final_state = 6 # State enum which signifies 'BURNOUT_DETECT' (Simulation only goes to apogee)

# Data Values
all_data_values = ["lowG_data", "highG_data", "gps_data", "barometer_data", "state_data", "voltage_data", "rocketState_data"]
rocket_states = ["STATE_UNKNOWN",
    "STATE_INIT",
    "STATE_IDLE",
    "STATE_LAUNCH_DETECT",
    "STATE_BOOST",
    "STATE_BURNOUT_DETECT",
    "STATE_COAST_PREGNC",
    "STATE_COAST_GNC",
    "STATE_APOGEE_DETECT",
    "STATE_APOGEE",
    "STATE_SEPARATION",
    "STATE_DROGUE_DETECT",
    "STATE_DROGUE",
    "STATE_MAIN_DETECT",
    "STATE_MAIN",
    "STATE_LANDED_DETECT",
    "STATE_LANDED",
    "STATE_ABORT"]

# Is the data processed?
data_is_processed = False
data_start_timestamp = 3481810