#This message contains the counter reading of the encoder at the datagram time (timestamp in header).

Header header           # same information as in laserscan and Pointcloud2 message

uint32 enc_position     # number of ticks
uint16 enc_speed         # number of ticks per millimeter
