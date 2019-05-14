Header header

# Steering Wheel
int32 steering_wheel_angle # degrees, range -630 to 630
int32 steering_wheel_angle_velocity # deg/s, unused
int32 steering_wheel_torque # Nm, unused

# Status

bool enabled # Enabled
bool override # Driver Override

# Maximum Values
int32 ANGLE_MAX=630 # degrees, maximum angle