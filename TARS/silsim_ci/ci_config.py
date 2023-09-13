## SILSIM CI configuration file
# Fail conditions
check_timeouts = True
check_timeouts_length = 10000

# Pass conditions
check_fsm = True # Check FSM final state
check_fsm_final_state = 3 # State enum which signifies 'LANDED'
