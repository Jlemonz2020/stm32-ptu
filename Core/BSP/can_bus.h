#ifndef CAN_BUS_H
#define CAN_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define CAN_BUS_STATUS_ID       (0x201U)
#define CAN_BUS_COMMAND_ID      (0x200U)
#define CAN_BUS_HEARTBEAT_ID    (0x202U)

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} can_bus_frame_t;

HAL_StatusTypeDef can_bus_init(void);
HAL_StatusTypeDef can_bus_send_status(void);
HAL_StatusTypeDef can_bus_send_frame(uint32_t id, const uint8_t *data, uint8_t len);
void can_bus_process_rx(void);
void can_bus_on_rx_pending(void);

#ifdef __cplusplus
}
#endif

#endif
