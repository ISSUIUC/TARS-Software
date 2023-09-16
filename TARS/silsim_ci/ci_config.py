## SILSIM CI configuration file
# Timestamp conversion
TICKS_TO_MS = 100
SECONDS_TO_MS = 0.001

# Fail conditions
check_timeouts = True
check_timeouts_length = 10

# Pass conditions
check_fsm = True # Check FSM final state
check_fsm_final_state = 16 # State enum which signifies 'LANDED'

# Data Values
all_data_vales = ["lowG_data", "highG_data", "gps_data", "barometer_data", "state_data", "voltage_data", "rocketState_data"]
