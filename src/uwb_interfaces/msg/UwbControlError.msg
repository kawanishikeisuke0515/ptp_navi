# Errors from the same calculation that produces the UWB velocity command.
# header.stamp is the control calculation time; no single frame applies to all fields.
std_msgs/Header header
string session_id
bool active
bool inputs_valid
# Metres. Invalid inputs produce NaN in every numeric field.
float64 raw_error_world_x
float64 raw_error_world_y
float64 error_world_x
float64 error_world_y
float64 error_body_x
float64 error_body_y
float64 distance_error_m
# Radians, wrapped to [-pi, pi).
float64 yaw_error
