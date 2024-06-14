#include "VescUart.h"
#include "buffer.h"
#include "datatypes.h"
#include <cstddef>
#include <stdint.h>

VescUart::VescUart(uint32_t timeout_ms) : _TIMEOUT(timeout_ms) {
  nunchuck.valueX = 127;
  nunchuck.valueY = 127;
  nunchuck.lowerButton = false;
  nunchuck.upperButton = false;
}

void VescUart::setSerialPort(Stream *port) { serialPort = port; }

void VescUart::setDebugPort(Stream *port) { debugPort = port; }

void VescUart::receiveUartMessageReady(uint16_t expectedLen) {
  rxState = 0;
  rxExpectedLen = expectedLen;
  rxStart = millis();
}
int VescUart::receiveUartMessageFsm() {
  uint32_t timeout =
      millis() +
      _TIMEOUT; // Defining the timestamp for timeout (100ms before timeout)

  if (millis() < timeout) {
    while (serialPort->available()) {

      rxBuffer[rxState++] = serialPort->read();

      if (rxState == 2) {

        switch (rxBuffer[0]) {
        case 2:
          rxExpectedLen =
              rxBuffer[1] + 5; // Payload size + 2 for sice + 3 for SRC and End.
          rxPayloadLen = rxBuffer[1];
          break;

        case 3:
          // ToDo: Add Message Handling > 255 (starting with 3)
          if (debugPort != NULL) {
            debugPort->println(
                "Message is larger than 256 bytes - not supported");
          }
          break;

        default:
          if (debugPort != NULL) {
            debugPort->println("Unvalid start bit");
          }
          break;
        }
      }

      if (rxState >= sizeof(rxBuffer)) {
        break;
      }

      if (rxState == rxExpectedLen && rxBuffer[rxExpectedLen - 1] == 3) {
        rxBuffer[rxExpectedLen] = 0;
        if (debugPort != NULL) {
          debugPort->println("End of message reached!");
        }

        int len =
            unpackPayload(rxBuffer, rxExpectedLen, NULL) ? rxPayloadLen : 0;
        return processReadPacket(&rxBuffer[2]);
      }
    }
  } else {
    rxState = 0;
    rxPayloadLen = 0;
    rxExpectedLen = 0;
    // Flush rx
    while (serialPort->available()) {
      serialPort->read();
    }
  }
  return 0;
}
int VescUart::receiveUartMessageBlocking(uint8_t *payloadReceived) {

  // Messages <= 255 starts with "2", 2nd byte is length
  // Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st
  // >>8 and then &0xFF

  // Makes no sense to run this function if no serialPort is defined.
  if (serialPort == NULL)
    return -1;

  uint16_t counter = 0;
  uint16_t endMessage = 256;
  bool messageRead = false;
  uint8_t messageReceived[256];
  uint16_t lenPayload = 0;

  uint32_t timeout =
      millis() +
      _TIMEOUT; // Defining the timestamp for timeout (100ms before timeout)

  while (millis() < timeout && messageRead == false) {

    while (serialPort->available()) {

      messageReceived[counter++] = serialPort->read();

      if (counter == 2) {

        switch (messageReceived[0]) {
        case 2:
          endMessage = messageReceived[1] +
                       5; // Payload size + 2 for sice + 3 for SRC and End.
          lenPayload = messageReceived[1];
          break;

        case 3:
          // ToDo: Add Message Handling > 255 (starting with 3)
          if (debugPort != NULL) {
            debugPort->println(
                "Message is larger than 256 bytes - not supported");
          }
          break;

        default:
          if (debugPort != NULL) {
            debugPort->println("Unvalid start bit");
          }
          break;
        }
      }

      if (counter >= sizeof(messageReceived)) {
        break;
      }

      if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
        messageReceived[endMessage] = 0;
        if (debugPort != NULL) {
          debugPort->println("End of message reached!");
        }
        messageRead = true;
        break; // Exit if end of message is reached, even if there is still more
               // data in the buffer.
      }
    }
  }
  if (messageRead == false && debugPort != NULL) {
    debugPort->println("Timeout");
  }

  bool unpacked = false;

  if (messageRead) {
    unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
  }

  if (unpacked) {
    // Message was read
    return lenPayload;
  } else {
    // No Message Read
    return 0;
  }
}

bool VescUart::unpackPayload(uint8_t *message, int lenMes, uint8_t *payload) {

  uint16_t crcMessage = 0;
  uint16_t crcPayload = 0;

  // Rebuild crc:
  crcMessage = message[lenMes - 3] << 8;
  crcMessage &= 0xFF00;
  crcMessage += message[lenMes - 2];

  if (debugPort != NULL) {
    debugPort->print("SRC received: ");
    debugPort->println(crcMessage);
  }

  // Extract payload:
  if (payload)
    memcpy(payload, &message[2], message[1]);
  else
    payload = &message[2];

  crcPayload = crc16(payload, message[1]);

  if (debugPort != NULL) {
    debugPort->print("SRC calc: ");
    debugPort->println(crcPayload);
  }

  if (crcPayload == crcMessage) {
    if (debugPort != NULL) {
      debugPort->print("Received: ");
      serialPrint(message, lenMes);
      debugPort->println();

      debugPort->print("Payload :      ");
      serialPrint(payload, message[1] - 1);
      debugPort->println();
    }

    return true;
  } else {
    return false;
  }
}

int VescUart::packSendPayload(uint8_t *payload, int lenPay) {

  uint16_t crcPayload = crc16(payload, lenPay);
  int count = 0;
  uint8_t messageSend[256];

  if (lenPay <= 256) {
    messageSend[count++] = 2;
    messageSend[count++] = lenPay;
  } else {
    messageSend[count++] = 3;
    messageSend[count++] = (uint8_t)(lenPay >> 8);
    messageSend[count++] = (uint8_t)(lenPay & 0xFF);
  }

  memcpy(messageSend + count, payload, lenPay);
  count += lenPay;

  messageSend[count++] = (uint8_t)(crcPayload >> 8);
  messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
  messageSend[count++] = 3;
  // messageSend[count] = NULL;

  if (debugPort != NULL) {
    debugPort->print("Package to send: ");
    serialPrint(messageSend, count);
  }

  // Sending package
  if (serialPort != NULL)
    serialPort->write(messageSend, count);

  // Returns number of send bytes
  return count;
}

bool VescUart::processReadPacket(uint8_t *buffer) {

  COMM_PACKET_ID packetId;
  int32_t index = 0;

  packetId = (COMM_PACKET_ID)buffer[0];
  buffer++; // Removes the packetId from the actual message (payload)

  switch (packetId) {
  case COMM_FW_VERSION: // Structure defined here:
                        // https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

    fw_version.major = buffer[index++];
    fw_version.minor = buffer[index++];
    return true;
  case COMM_GET_VALUES: // Structure defined here:
                        // https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

  case COMM_GET_VALUES_SELECTIVE: {
    uint32_t mask = 0xFFFFFFFF;
    if (packetId == COMM_GET_VALUES_SELECTIVE) {
      uint32_t mask = buffer_get_uint32(buffer, &index);
    }
    dataLast = data;
    if (mask & VAL_MASK_TEMP_MOSFET) {
      data.tempMosfet = buffer_get_float16(buffer, 10.0, &index);
    }
    if (mask & VAL_MASK_TEMP_MOTOR) {
      data.tempMotor = buffer_get_float16(buffer, 10.0, &index);
    }
    if (mask & VAL_MASK_AVG_MOTOR_CURRENT) {
      data.avgMotorCurrent = buffer_get_float32(buffer, 100.0, &index);
    }
    if (mask & VAL_MASK_AVG_INPUT_CURRENT) {
      data.avgInputCurrent = buffer_get_float32(buffer, 100.0, &index);
    }
    if (mask & VAL_MASK_AVG_IQ) {
      index += 4;
    }
    if (mask & VAL_MASK_AVG_ID) {
      index += 4;
    }
    if (mask & VAL_MASK_DUTY) {
      data.dutyCycleNow = buffer_get_float16(buffer, 1000.0, &index);
    }
    if (mask & VAL_MASK_RPM) {
      data.rpm = buffer_get_float32(buffer, 1.0, &index);
    }
    if (mask & VAL_MASK_INP_VOLTAGE) {
      data.inpVoltage = buffer_get_float16(buffer, 10.0, &index);
    }
    if (mask & VAL_MASK_AMP_HOURS) {
      data.ampHours = buffer_get_float32(buffer, 10000.0, &index);
    }
    if (mask & VAL_MASK_AMP_HOURS_CHARGED) {
      data.ampHoursCharged = buffer_get_float32(buffer, 10000.0, &index);
    }
    if (mask & VAL_MASK_WATT_HOURS) {
      data.wattHours = buffer_get_float32(buffer, 10000.0, &index);
    }
    if (mask & VAL_MASK_WATT_HOURS_CHARGED) {
      data.wattHoursCharged = buffer_get_float32(buffer, 10000.0, &index);
    }
    if (mask & VAL_MASK_TACHOMETER) {
      data.tachometer = buffer_get_int32(buffer, &index);
    }
    if (mask & VAL_MASK_TACHOMETER_ABS) {
      data.tachometerAbs = buffer_get_int32(buffer, &index);
    }
    if (mask & VAL_MASK_ERROR) {
      data.error = (mc_fault_code)buffer_get_int32(buffer, &index);
    }
    if (mask & VAL_MASK_PID_POS) {
      data.pidPos = buffer_get_float32(buffer, 1000000.0, &index);
    }
    if (mask & VAL_MASK_ID) {
      data.id = buffer[index++];
    }
    // TODO: there is some more I ignore
    return true;
  }; break;

  default:
    return false;
    break;
  }
}

bool VescUart::getFWversion(void) { return getFWversion(0); }

bool VescUart::getFWversion(uint8_t canId) {

  int32_t index = 0;
  int payloadSize = (canId == 0 ? 1 : 3);
  uint8_t payload[payloadSize];

  if (canId != 0) {
    payload[index++] = {COMM_FORWARD_CAN};
    payload[index++] = canId;
  }
  payload[index++] = {COMM_FW_VERSION};

  packSendPayload(payload, payloadSize);

  uint8_t message[256];
  int messageLength = receiveUartMessageBlocking(message);
  if (messageLength > 0) {
    return processReadPacket(message);
  }
  return false;
}

bool VescUart::getValues(void) { return getValues(0); }

bool VescUart::getValues(uint8_t canId = 0) {
  return getValuesMasked(0xFFFFFFFF, canId);
}

bool VescUart::getValuesMasked(uint32_t mask) {
  return getValuesMasked(mask, 0);
}

bool VescUart::getValuesMasked(uint32_t mask, uint8_t canId) {

  if (debugPort != NULL) {
    debugPort->println("Command: COMM_GET_VALUES " + String(canId));
  }

  int32_t index = 0;
  int payloadSize = (canId == 0 ? 1 : 3);
  uint8_t payload[payloadSize];
  if (canId != 0) {
    payload[index++] = {COMM_FORWARD_CAN};
    payload[index++] = canId;
  }
  if (~mask != 0) {
    payload[index++] = {COMM_GET_VALUES_SELECTIVE};
    buffer_append_int32(payload, mask, &index);
  } else {
    payload[index++] = {COMM_GET_VALUES};
  }

  packSendPayload(payload, payloadSize);

  // int messageLength = receiveUartMessageBlocking(rxBuffer);
  // if (messageLength > 55) {
  //   return processReadPacket(rxBuffer);
  // }
  receiveUartMessageReady(55);
  return true;
}

void VescUart::setNunchuckValues() { return setNunchuckValues(0); }

void VescUart::setNunchuckValues(uint8_t canId) {

  if (debugPort != NULL) {
    debugPort->println("Command: COMM_SET_CHUCK_DATA " + String(canId));
  }
  int32_t index = 0;
  int payloadSize = (canId == 0 ? 11 : 13);
  uint8_t payload[payloadSize];

  if (canId != 0) {
    payload[index++] = {COMM_FORWARD_CAN};
    payload[index++] = canId;
  }
  payload[index++] = {COMM_SET_CHUCK_DATA};
  payload[index++] = nunchuck.valueX;
  payload[index++] = nunchuck.valueY;
  buffer_append_bool(payload, nunchuck.lowerButton, &index);
  buffer_append_bool(payload, nunchuck.upperButton, &index);

  // Acceleration Data. Not used, Int16 (2 byte)
  payload[index++] = 0;
  payload[index++] = 0;
  payload[index++] = 0;
  payload[index++] = 0;
  payload[index++] = 0;
  payload[index++] = 0;

  if (debugPort != NULL) {
    debugPort->println("Nunchuck Values:");
    debugPort->print("x=");
    debugPort->print(nunchuck.valueX);
    debugPort->print(" y=");
    debugPort->print(nunchuck.valueY);
    debugPort->print(" LBTN=");
    debugPort->print(nunchuck.lowerButton);
    debugPort->print(" UBTN=");
    debugPort->println(nunchuck.upperButton);
  }

  packSendPayload(payload, payloadSize);
}

void VescUart::setCurrent(float current) { return setCurrent(current, 0); }

void VescUart::setCurrent(float current, uint8_t canId) {
  int32_t index = 0;
  int payloadSize = (canId == 0 ? 5 : 7);
  uint8_t payload[payloadSize];
  if (canId != 0) {
    payload[index++] = {COMM_FORWARD_CAN};
    payload[index++] = canId;
  }
  payload[index++] = {COMM_SET_CURRENT};
  buffer_append_int32(payload, (int32_t)(current * 1000), &index);
  packSendPayload(payload, payloadSize);
}

void VescUart::setBrakeCurrent(float brakeCurrent) {
  return setBrakeCurrent(brakeCurrent, 0);
}

void VescUart::setBrakeCurrent(float brakeCurrent, uint8_t canId) {
  int32_t index = 0;
  int payloadSize = (canId == 0 ? 5 : 7);
  uint8_t payload[payloadSize];
  if (canId != 0) {
    payload[index++] = {COMM_FORWARD_CAN};
    payload[index++] = canId;
  }

  payload[index++] = {COMM_SET_CURRENT_BRAKE};
  buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

  packSendPayload(payload, payloadSize);
}

void VescUart::setRPM(float rpm) { return setRPM(rpm, 0); }

void VescUart::setRPM(float rpm, uint8_t canId) {
  int32_t index = 0;
  int payloadSize = (canId == 0 ? 5 : 7);
  uint8_t payload[payloadSize];
  if (canId != 0) {
    payload[index++] = {COMM_FORWARD_CAN};
    payload[index++] = canId;
  }
  payload[index++] = {COMM_SET_RPM};
  buffer_append_int32(payload, (int32_t)(rpm), &index);
  packSendPayload(payload, payloadSize);
}

void VescUart::setPos(float pos) { return setPos(pos, 0); }

void VescUart::setPos(float pos, uint8_t canId) {
  int32_t index = 0;
  int payloadSize = (canId == 0 ? 5 : 7);
  uint8_t payload[payloadSize];
  if (canId != 0) {
    payload[index++] = {COMM_FORWARD_CAN};
    payload[index++] = canId;
  }
  payload[index++] = {COMM_SET_POS};
  buffer_append_int32(payload, (int32_t)(pos), &index);
  packSendPayload(payload, payloadSize);
}

void VescUart::setDuty(float duty) { return setDuty(duty, 0); }

void VescUart::setDuty(float duty, uint8_t canId) {
  int32_t index = 0;
  int payloadSize = (canId == 0 ? 5 : 7);
  uint8_t payload[payloadSize];
  if (canId != 0) {
    payload[index++] = {COMM_FORWARD_CAN};
    payload[index++] = canId;
  }
  payload[index++] = {COMM_SET_DUTY};
  buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

  packSendPayload(payload, payloadSize);
}

void VescUart::sendKeepalive(void) { return sendKeepalive(0); }

void VescUart::sendKeepalive(uint8_t canId) {
  int32_t index = 0;
  int payloadSize = (canId == 0 ? 1 : 3);
  uint8_t payload[payloadSize];
  if (canId != 0) {
    payload[index++] = {COMM_FORWARD_CAN};
    payload[index++] = canId;
  }
  payload[index++] = {COMM_ALIVE};
  packSendPayload(payload, payloadSize);
}

void VescUart::serialPrint(uint8_t *data, int len) {
  if (debugPort != NULL) {
    for (int i = 0; i <= len; i++) {
      debugPort->print(data[i]);
      debugPort->print(" ");
    }
    debugPort->println("");
  }
}

void VescUart::printVescValues() {
  if (debugPort != NULL) {
    debugPort->print("avgMotorCurrent: ");
    debugPort->println(data.avgMotorCurrent);
    debugPort->print("avgInputCurrent: ");
    debugPort->println(data.avgInputCurrent);
    debugPort->print("dutyCycleNow: ");
    debugPort->println(data.dutyCycleNow);
    debugPort->print("rpm: ");
    debugPort->println(data.rpm);
    debugPort->print("inputVoltage: ");
    debugPort->println(data.inpVoltage);
    debugPort->print("ampHours: ");
    debugPort->println(data.ampHours);
    debugPort->print("ampHoursCharged: ");
    debugPort->println(data.ampHoursCharged);
    debugPort->print("wattHours: ");
    debugPort->println(data.wattHours);
    debugPort->print("wattHoursCharged: ");
    debugPort->println(data.wattHoursCharged);
    debugPort->print("tachometer: ");
    debugPort->println(data.tachometer);
    debugPort->print("tachometerAbs: ");
    debugPort->println(data.tachometerAbs);
    debugPort->print("tempMosfet: ");
    debugPort->println(data.tempMosfet);
    debugPort->print("tempMotor: ");
    debugPort->println(data.tempMotor);
    debugPort->print("error: ");
    debugPort->println(data.error);
  }
}
