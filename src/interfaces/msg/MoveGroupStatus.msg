uint8 IDLE=0
uint8 PLANNING=1
uint8 MOVING=2
uint8 FAILED=3

std_msgs/Header header
uint8 status
string status_text
string active_target