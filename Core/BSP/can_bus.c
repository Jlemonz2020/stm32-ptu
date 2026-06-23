#include "can_bus.h"

#include <string.h>

#include "fdcan.h"
#include "foc.h"

#define CAN_BUS_RX_QUEUE_LEN  (8U)

static can_bus_frame_t rx_queue[CAN_BUS_RX_QUEUE_LEN];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;

static uint32_t len_to_fdcan_dlc(uint8_t len)
{
  switch (len) {
    case 0U: return FDCAN_DLC_BYTES_0;
    case 1U: return FDCAN_DLC_BYTES_1;
    case 2U: return FDCAN_DLC_BYTES_2;
    case 3U: return FDCAN_DLC_BYTES_3;
    case 4U: return FDCAN_DLC_BYTES_4;
    case 5U: return FDCAN_DLC_BYTES_5;
    case 6U: return FDCAN_DLC_BYTES_6;
    case 7U: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
  }
}

static uint8_t fdcan_dlc_to_len(uint32_t dlc)
{
  switch (dlc) {
    case FDCAN_DLC_BYTES_0: return 0U;
    case FDCAN_DLC_BYTES_1: return 1U;
    case FDCAN_DLC_BYTES_2: return 2U;
    case FDCAN_DLC_BYTES_3: return 3U;
    case FDCAN_DLC_BYTES_4: return 4U;
    case FDCAN_DLC_BYTES_5: return 5U;
    case FDCAN_DLC_BYTES_6: return 6U;
    case FDCAN_DLC_BYTES_7: return 7U;
    default: return 8U;
  }
}

static int16_t decode_i16(const uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static void encode_i16(uint8_t *data, int16_t value)
{
  data[0] = (uint8_t)((uint16_t)value & 0xFFU);
  data[1] = (uint8_t)(((uint16_t)value >> 8U) & 0xFFU);
}

static void push_rx(const can_bus_frame_t *frame)
{
  uint8_t next_head = (uint8_t)((rx_head + 1U) % CAN_BUS_RX_QUEUE_LEN);

  if (next_head == rx_tail) {
    return;
  }

  rx_queue[rx_head] = *frame;
  rx_head = next_head;
}

static uint8_t pop_rx(can_bus_frame_t *frame)
{
  if ((frame == NULL) || (rx_head == rx_tail)) {
    return 0U;
  }

  *frame = rx_queue[rx_tail];
  rx_tail = (uint8_t)((rx_tail + 1U) % CAN_BUS_RX_QUEUE_LEN);
  return 1U;
}

static void handle_command(const can_bus_frame_t *frame)
{
  if ((frame == NULL) || (frame->id != CAN_BUS_COMMAND_ID) || (frame->dlc < 1U)) {
    return;
  }

  switch (frame->data[0]) {
    case 0x01U:
      (void)foc_enable();
      break;

    case 0x02U:
      foc_disable();
      break;

    case 0x03U:
      if (frame->dlc >= 3U) {
        foc_set_iq_target((float)decode_i16(&frame->data[1]) * 0.01f);
      }
      break;

    case 0x04U:
      if (frame->dlc >= 5U) {
        foc_set_open_loop((float)decode_i16(&frame->data[1]) * 0.01f,
                          (float)decode_i16(&frame->data[3]) * 0.1f);
      }
      break;

    default:
      break;
  }
}

HAL_StatusTypeDef can_bus_init(void)
{
  FDCAN_FilterTypeDef filter;
  HAL_StatusTypeDef status;

  rx_head = 0U;
  rx_tail = 0U;

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000U;
  filter.FilterID2 = 0x000U;

  status = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
  if (status != HAL_OK) {
    return status;
  }

  status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                        FDCAN_ACCEPT_IN_RX_FIFO0,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE,
                                        FDCAN_REJECT_REMOTE);
  if (status != HAL_OK) {
    return status;
  }

  status = HAL_FDCAN_Start(&hfdcan1);
  if (status != HAL_OK) {
    return status;
  }

  return HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
}

HAL_StatusTypeDef can_bus_send_frame(uint32_t id, const uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef header;
  uint8_t payload[8] = {0};

  if (len > 8U) {
    len = 8U;
  }
  if ((data != NULL) && (len > 0U)) {
    (void)memcpy(payload, data, len);
  }

  header.Identifier = id;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = len_to_fdcan_dlc(len);
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, payload);
}

HAL_StatusTypeDef can_bus_send_status(void)
{
  foc_status_t status;
  uint8_t data[8] = {0};
  int16_t iq;
  int16_t target;
  int16_t vbus;

  foc_get_status(&status);
  iq = (int16_t)(status.iq * 100.0f);
  target = (int16_t)(status.iq_target * 100.0f);
  vbus = (int16_t)(status.vbus * 10.0f);

  data[0] = status.enabled;
  data[1] = (uint8_t)status.mode;
  encode_i16(&data[2], iq);
  encode_i16(&data[4], target);
  encode_i16(&data[6], vbus);

  return can_bus_send_frame(CAN_BUS_STATUS_ID, data, 8U);
}

void can_bus_process_rx(void)
{
  can_bus_frame_t frame;

  while (pop_rx(&frame) != 0U) {
    handle_command(&frame);
  }
}

void can_bus_on_rx_pending(void)
{
  FDCAN_RxHeaderTypeDef header;
  can_bus_frame_t frame;

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK) {
      return;
    }

    if (header.IdType == FDCAN_STANDARD_ID) {
      frame.id = header.Identifier;
      frame.dlc = fdcan_dlc_to_len(header.DataLength);
      push_rx(&frame);
    }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((hfdcan->Instance == FDCAN1) &&
      ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)) {
    can_bus_on_rx_pending();
  }
}
