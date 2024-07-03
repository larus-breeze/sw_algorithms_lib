#ifndef CAN_GATEWAY_H_
#define CAN_GATEWAY_H_

#include "generic_CAN_driver.h"

bool CAN_gateway_poll( CANpacket &p, unsigned max_wait);

#endif /* CAN_GATEWAY_H_ */
