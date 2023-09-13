## SILSIM CI configuration file
# Timestamp conversion
TICKS_TO_MS = 100

# Fail conditions
check_timeouts = True
check_timeouts_length = 10

# Pass conditions
check_fsm = True # Check FSM final state
check_fsm_final_state = 16 # State enum which signifies 'LANDED'
