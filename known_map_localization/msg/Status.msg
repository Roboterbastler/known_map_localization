# Time stamp of the status update
time stamp

# Status of the known map localization module
# 0 - Initialization
# 1 - No position estimate available
# 2 - Position estimate available, but not validated by GPS
# 3 - Validated position estimate available
uint8 status

# Number of supporting GPS-Positions (only used when status = 3)
uint32 n_supporting_positions
