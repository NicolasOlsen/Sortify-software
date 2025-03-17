#include <Arduino.h>
#include "System_status.h"
#include "UART_communication.h"

using namespace UART_communication;

void sendRequest(MainCommand command);

void setup() {
    System_status::currentStatus = StatusCode::INITIALIZING;
    Serial.begin(115200);  // Debugging output to PC
    UART_init(1000000);  // Initialize UART1

    Serial.println("Master Initialized.");
    Serial.print("\nRequesting Status...");
    sendRequest(MainCommand::REQUEST_STATUS);
}

void loop() {
    System_status::currentStatus = StatusCode::IDLE;
    // Process response if available
    while (Serial1.available()) {
        receiveUARTData();  // Read and process response
    }
    Serial.println("Loop");
    delay(100);
}

// Function to send a request packet to the Slave
void sendRequest(MainCommand command) {
    sendPacket(command, nullptr, 0);
}
