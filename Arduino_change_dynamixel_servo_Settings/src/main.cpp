#include <DynamixelShield.h>
#include <Arduino.h>

DynamixelShield dxl;

using namespace ControlTableItem;

const float DXL_PROTOCOL_VERSION = 2.0;

// What you want to set
const uint8_t NEW_ID = 2;
const uint32_t NEW_BAUDRATE = 1000000;  // 1 Mbps

// Search range
const uint8_t ID_START = 0;
const uint8_t ID_END = 20;
const uint32_t baudrates[] = {57600, 115200, 250000, 500000, 1000000, 2000000, 3000000};
const uint8_t numBaudrates = sizeof(baudrates) / sizeof(baudrates[0]);

void setup() {
  Serial1.begin(1000000);  // USB Serial1
  while (!Serial1);

  Serial1.println("Starting Dynamixel auto-scan and reconfiguration...\n");

  for (uint8_t b = 0; b < numBaudrates; b++) {
    uint32_t test_baud = baudrates[b];
    Serial1.print("Scanning at baudrate: ");
    Serial1.println(test_baud);

    dxl.begin(test_baud);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    for (uint8_t id = ID_START; id <= ID_END; id++) {
      if (dxl.ping(id)) {
        Serial1.println("Device found!");
        Serial1.print("\tID: "); Serial1.println(id);
        Serial1.print("\tBaudrate: "); Serial1.println(test_baud);
        Serial1.print("\tModel Number: "); Serial1.println(dxl.getModelNumber(id));

        // Disable torque to change settings
        dxl.torqueOff(id);
        delay(100);

        // Change ID if needed
        if (id != NEW_ID) {
          if (dxl.setID(id, NEW_ID)) {
            Serial1.print("\tID changed to: ");
            Serial1.println(NEW_ID);
            id = NEW_ID;  // Update current ID
          } else {
            Serial1.println("\tFailed to change ID");
            continue;
          }
        }

        // Change baudrate if needed
        if (test_baud != NEW_BAUDRATE) {
          if (dxl.setBaudrate(id, NEW_BAUDRATE)) {
            Serial1.print("\tBaudrate changed to: ");
            Serial1.println(NEW_BAUDRATE);

            // Reconnect with new baudrate
            dxl.begin(NEW_BAUDRATE);
          } else {
            Serial1.println("\tFailed to change baudrate");
          }
        }

        Serial1.println("Reconfiguration complete!\n");
        return;  // Exit after finding and configuring the first one
      }
    }
    Serial1.println("No device found at this baudrate.\n");
  }

  Serial1.println("No servos found in scan range.");
}

void loop() {
  // Nothing to do
}
