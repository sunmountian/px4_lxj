uint64 timestamp
uint64 timestamp_sample

uint8 STATE_INIT = 0
uint8 STATE_WARMUP = 1
uint8 STATE_MONITOR = 2
uint8 STATE_ALERT = 3
uint8 state

uint8 ACTUATOR_MOTORS = 0
uint8 ACTUATOR_OUTPUTS = 1
uint8 actuator_source

bool valid
bool attack_detected
bool direction_valid
bool gps_valid
bool attitude_valid
bool actuator_valid
bool imu_valid

float32 glrt_score_n
float32 glrt_score_e
float32 glrt_score
float32 cost_null_n
float32 cost_null_e
float32 cost_attack_n
float32 cost_attack_e
float32 normalized_score
float32 cusum_score
float32 threshold

float32 direction_n
float32 direction_e
float32 direction_norm
float32 selected_window_s

# V2 normal-model parameter estimates for calibration/diagnostics.
float32 normal_bias_n
float32 normal_bias_e
float32 normal_scale_n
float32 normal_scale_e
float32 normal_dynamic_n
float32 normal_dynamic_e

float32 accel_meas_n
float32 accel_meas_e
float32 accel_actuator_n
float32 accel_actuator_e

uint16 imu_sample_count
uint16 gps_sample_count
uint16 consecutive_count

uint32 computation_time_us

uint32 DATA_QUALITY_NONE = 0
uint32 DATA_QUALITY_TIMESTAMP = 1
uint32 DATA_QUALITY_BUFFER_OVERFLOW = 2
uint32 DATA_QUALITY_IMU_CLIPPING = 4
uint32 DATA_QUALITY_ATTITUDE_ALIGNMENT = 8
uint32 DATA_QUALITY_ACTUATOR_ALIGNMENT = 16
uint32 DATA_QUALITY_WINDOW_COVERAGE = 32
uint32 DATA_QUALITY_SOLVER = 64
uint32 DATA_QUALITY_GPS = 128
uint32 data_quality_flags
uint32 imu_age_us
uint32 gps_age_us
uint32 attitude_age_us
uint32 actuator_age_us
