string trial_name
string logging_path
string agent
string error

uint16 k_ep
uint16 k_run

uint8 n_rollouts
float32 h_ceiling

float64[2] mu
float64[2] sigma
float64[2] policy

float32 reward
float32 reward_avg

float64[3] vel_d
bool impact_flag
bool body_contact
uint8 leg_contacts

bool runComplete_flag
bool trialComplete_flag
