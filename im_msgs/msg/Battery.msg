uint8 DISCHARGING = 0
uint8 CHARGING    = 1
uint8 CHARGED     = 2

std_msgs/Header header
float32 voltage 				# Voltage in Volts
float32 current 				# Negative when discharging (A)
float32 charge					# Current charge in Ah
float32 capacity				# Capacity in Ah
float32 temperature 		# Temperature
uint8 charge_state			# enum
