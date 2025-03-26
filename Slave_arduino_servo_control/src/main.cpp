#include <Arduino.h>
#include "System_status.h"
#include "Debug.h"
#include "UART_communication.h"

using namespace Com_code;
using namespace UART_communication;

unsigned long previousRequestTime = millis();
unsigned long previousRecieveTime = millis();
const unsigned long intervalRequest = 200;
const unsigned long intervalRecieve = 50;

void sendRequest(MainCommand command);

void setup() {
    System_status::currentStatus = StatusCode::INITIALIZING;
    Debug::init(250000);  // Debugging output to PC
    UART_init(1000000);  // Initialize UART

    Debug::infoln("Initialized.");

    System_status::currentStatus = StatusCode::IDLE;
}

void loop() {
    unsigned long currentTime = millis();

    if (currentTime - previousRequestTime >= intervalRequest) {
        previousRequestTime += intervalRequest;

        Debug::infoln("Requesting Status...");
        sendRequest(MainCommand::REQUEST_STATUS);
    }

    if (currentTime - previousRecieveTime >= intervalRecieve) {
        previousRecieveTime += intervalRecieve;

        receiveUARTData();  // Read and process response
    }
}

// Function to send a request packet to the Slave
void sendRequest(MainCommand command) {
    sendPacket(command, nullptr, 0);
}
