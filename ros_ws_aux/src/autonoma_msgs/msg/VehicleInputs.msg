std_msgs/Header                 header

# Throttle command (%)
float32 throttle_cmd
uint8 throttle_cmd_count

# Brake pressure command (kPa)
uint16 brake_cmd_front
uint16 brake_cmd_rear
uint8 brake_bias_switch
uint8 brake_cmd_count

# Steering motor angle command (degrees)
float32 steering_cmd
uint8 steering_cmd_count

# Gear command
uint8 gear_cmd