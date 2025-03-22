#include <Arduino.h>
#include "System_status.h"
#include "Debug.h"
#include "UART_communication.h"

using namespace Com_code;
using namespace UART_communication;

unsigned long previousTime = millis();
const unsigned long interval = 200;

void sendRequest(MainCommand command);

void setup() {
    System_status::currentStatus = StatusCode::INITIALIZING;
    Debug::init(250000);  // Debugging output to PC
    UART_init(1000000);  // Initialize UART1

    Debug::infoln("Slave Initialized.");

    System_status::currentStatus = StatusCode::IDLE;
}

void loop() {
    unsigned long currentTime = millis();

    if (currentTime - previousTime >= interval) {
        previousTime += interval;

        // Debug::infoln("Requesting Status...");
        // sendRequest(MainCommand::REQUEST_STATUS);

        receiveUARTData();  // Read and process response

        Debug::infoln("Tick");
    }
}

// Function to send a request packet to the Slave
void sendRequest(MainCommand command) {
    sendPacket(command, nullptr, 0);
}
