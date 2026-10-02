uint8 up
uint8 state

# Bit errors
uint32 txerr
uint32 rxerr

uint32 tx_packets
uint32 tx_packet_errs
uint32 tx_packet_drops
uint32 rx_packets
uint32 rx_packet_errs
uint32 rx_packet_drops

uint32 restarts
uint32 bus_error
uint32 arbitration_lost
uint32 error_warning
uint32 error_passive
uint32 bus_off

uint8 CAN_STATE_ERROR_ACTIVE  = 0
uint8 CAN_STATE_ERROR_WARNING = 1
uint8 CAN_STATE_ERROR_PASSIVE = 2
uint8 CAN_STATE_BUS_OFF       = 3
uint8 CAN_STATE_STOPPED       = 4
uint8 CAN_STATE_SLEEPING      = 5