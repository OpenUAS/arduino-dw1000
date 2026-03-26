/*
 * ConnectedRanging.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: steven <stevenhelm@live.nl>
 */


#include "ConnectedRanging.h"


ConnectedRangingClass ConnectedRanging;


// data buffer. Format of data buffer is like this: {from_address, to_address_1, message_type, additional_data, to_address_2, message_type, additional_data,.... to_address_n, message_type, additional data}
// Every element takes up exactly 1 byte, except for additional_data, which takes up 15 bytes in the case of a RANGE message, and 4 bytes in the case of RANGE_REPORT
// Example: byte _data[] = {2, 1, POLL, 3, RANGE, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 4, RANGE_REPORT, 1, 2, 3, 4, 5, POLL_ACK};
byte ConnectedRangingClass::_data[MAX_LEN_DATA];

// pins on the arduino used to communicate with DW1000
uint8_t ConnectedRangingClass::_RST;
uint8_t ConnectedRangingClass::_SS;
uint8_t ConnectedRangingClass::_IRQ;

// addresses of current DW1000
byte ConnectedRangingClass::_longAddress[LEN_EUI];
byte ConnectedRangingClass::_veryShortAddress;

// message sent/received state
volatile boolean ConnectedRangingClass::_sentAck     = false;
volatile boolean ConnectedRangingClass::_receivedAck = false;

// keeping track of send times
uint32_t ConnectedRangingClass::_lastSent = 0;

// nodes (_numNodes includes the current device, but _networkNodes does not)
DW1000Node ConnectedRangingClass::_networkNodes[MAX_NODES];
DW1000Node* ConnectedRangingClass::_lastNode;
uint8_t ConnectedRangingClass::_numNodes = 0;
DW1000Node ConnectedRangingClass::_selfNode;

// when it is time to send
boolean ConnectedRangingClass::_timeToSend = false;

// remembering future time in case RANGE message is sent
boolean ConnectedRangingClass::_rangeSent = false;
DW1000Time ConnectedRangingClass::_rangeTime;

// extended frame
boolean ConnectedRangingClass::_extendedFrame = false;

uint16_t ConnectedRangingClass::protTimes = 0;

// last noted activity
uint32_t ConnectedRangingClass::_lastActivity;

uint16_t ConnectedRangingClass::_maxLenData;



// Handlers
void (* ConnectedRangingClass::_handleNewRange)(void) = 0;



// initialization function
void ConnectedRangingClass::ConnectedRangingClass::init(char longAddress[], uint8_t numNodes)
{
    SerialCoder.attachStateHandle(handleNewSelfStateValue);
    SerialCoder.attachPackedStateHandle(handleNewSelfPackedState);
    // nodes to range to
    if (numNodes <= MAX_NODES) {
        _numNodes = numNodes;
    } else {
        Serial.println(F("The desired number of nodes exceeds MAX_NODES"));
        _numNodes = MAX_NODES;
    }
    DW1000.convertToByte(longAddress, _longAddress);
    initDecawave(_longAddress, numNodes);
    initNodes();
    setSelfState(0, 0, 0, 0, 0, 0);
    _lastSent = millis();
    _lastActivity = millis();
}

// initialization function
void ConnectedRangingClass::init(uint8_t veryShortAddress, uint8_t numNodes)
{
    SerialCoder.attachStateHandle(handleNewSelfStateValue);
    SerialCoder.attachPackedStateHandle(handleNewSelfPackedState);
    // nodes to range to
    if (numNodes <= MAX_NODES) {
        _numNodes = numNodes;
    } else {
        Serial.println("The desired number of nodes exceeds MAX_NODES");
        _numNodes = MAX_NODES;
    }
    for (int i = 0; i < LEN_EUI; i++) {
        _longAddress[i] = veryShortAddress;
    }
    initDecawave(_longAddress, numNodes);
    initNodes();
    setSelfState(0, 0, 0, 0, 0, 0);

    _lastSent = millis();
    _lastActivity = millis();



    Serial.print(F("Initialization complete, this device's Short Address is: "));
    Serial.print(_veryShortAddress, HEX);
    Serial.print(F(" Long Address: "));

    for (int i = 0; i < LEN_EUI; i++) {
        Serial.print(_longAddress[i]);
    }
    Serial.println(F(" "));
    Serial.println(F("The distant devices in memory are: "));
    for (int i = 0; i < _numNodes - 1; i++) {
        _networkNodes[i].printNode();
    }



}

// initialization function
void ConnectedRangingClass::initNodes()
{
    byte address[LEN_EUI];
    byte shortaddress[2];
    uint8_t index = 0;
    uint8_t remoteShortAddress = 1;
    for (int i = 0; i < _numNodes; i++) {
        for (int j = 0; j < LEN_EUI; j++) {
            address[j] = remoteShortAddress;
        }
        for (int j = 0; j < 2; j++) {
            shortaddress[j] = remoteShortAddress;
        }
        DW1000Node temp = DW1000Node(address, shortaddress);

        if (remoteShortAddress == _veryShortAddress) {
            _selfNode = temp;
        } else if (remoteShortAddress != _veryShortAddress) {
            _networkNodes[index] = temp;
            index++;
        }
        remoteShortAddress++;

    }
}

// initialization function
void ConnectedRangingClass::initDecawave(byte longAddress[], uint8_t numNodes, const byte mode[], uint16_t networkID, uint8_t myRST, uint8_t mySS, uint8_t myIRQ)
{
    _RST = myRST;
    _SS = mySS;
    _IRQ = myIRQ;
    DW1000.begin(myIRQ, myRST);
    DW1000.select(mySS);
    //Save the address
    _veryShortAddress = _longAddress[0];
    //Write the address on the DW1000 chip
    DW1000.setEUI(_longAddress);
    //Serial.print(F("very short device address: ")); Serial.println(_veryShortAddress);


    //Setup DW1000 configuration
    DW1000.newConfiguration();
    DW1000.setDefaults(_extendedFrame);
    DW1000.setDeviceAddress((uint16_t)_veryShortAddress);
    DW1000.setNetworkId(networkID);
    DW1000.enableMode(mode);
    DW1000.commitConfiguration();
    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);
    receiver();

    _maxLenData = (numNodes - 1) * RANGE_SIZE + numNodes + STATE_SIZE; // Worst case scenario: range message to all DW1000's means this message size
    _extendedFrame = (MAX_LEN_DATA > 125) ? true : false;
    Serial.print(F("Maximum data length is: "));
    Serial.println(_maxLenData);

}

// Main function that should be called from arduino
void ConnectedRangingClass::loop()
{
    SerialCoder.getSerialData();
    checkForReset();
    if (_sentAck) {
        noteActivity();
        _sentAck = false;
        updateSentTimes();
    }
    if (_receivedAck) {
        noteActivity();
        _receivedAck = false;
        //we read the datas from the modules:
        // get message and parse
        DW1000.getData(_data, _maxLenData);
        handleReceivedData();
    }
    if (_veryShortAddress == 1 && millis() - _lastSent > DEFAULT_RESET_TIME) {
        _timeToSend = true;
    }
    if (_timeToSend) {
        _timeToSend = false;
        _lastSent = millis();
        transmitInit();
        produceMessage();

        if (_rangeSent) {
            _rangeSent = false;
            transmitData(_data);
        } else {
            DW1000Time delay = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW1000Time::MICROSECONDS);
            transmitData(_data, delay);
        }
    }

}

// Set DW1000 in receiving mode
void ConnectedRangingClass::receiver()
{
    DW1000.newReceive();
    DW1000.setDefaults(_extendedFrame);
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

// Initialize a transmit
void ConnectedRangingClass::transmitInit()
{
    DW1000.newTransmit();
    DW1000.setDefaults(_extendedFrame);
}

// Transmit a byte array
void ConnectedRangingClass::transmitData(byte datas[])
{
    DW1000.setData(datas, _maxLenData);
    DW1000.startTransmit();
}

// Transmit a byte array with a delay
void ConnectedRangingClass::transmitData(byte datas[], DW1000Time timeDelay)
{
    DW1000.setDelay(timeDelay);
    DW1000.setData(datas, _maxLenData);
    DW1000.startTransmit();
}

// Transmit a char array
void ConnectedRangingClass::transmitData(char datas[])
{
    DW1000.convertCharsToBytes(datas, _data, _maxLenData);
    DW1000.setData(_data, _maxLenData);
    DW1000.startTransmit();
}

// Transmit a char array and specify the length n
void ConnectedRangingClass::transmitData(char datas[], uint16_t n)
{
    DW1000.convertCharsToBytes(datas, _data, n);
    DW1000.setData(_data, n);
    DW1000.startTransmit();
}

// Handle a successful sent message
void ConnectedRangingClass::handleSent()
{
    _sentAck = true;
}

// Handle a successful received message
void ConnectedRangingClass::handleReceived()
{
    _receivedAck = true;
}

// Handle the received data
void ConnectedRangingClass::handleReceivedData()
{
    uint8_t messagefrom = _data[0];
    uint8_t nodeIndex = messagefrom - 1 - (uint8_t)(_veryShortAddress < messagefrom);
    _lastNode = &_networkNodes[nodeIndex];

    // Nodes transmit in ascending order, so this device will transmit if the previous device's address is one less than this device's address
    if (messagefrom == _veryShortAddress - 1) {
        _timeToSend = true;
    } else if (messagefrom == _numNodes && _veryShortAddress == 1) {
        _timeToSend = true;
    } else {
        _timeToSend = false;
    }

    uint16_t datapointer = 1;
    //Serial.println(F("Got into handleReceivedData, timetosend is: "));Serial.println(_timeToSend);


    retrieveState(&datapointer); // Get the state information from the sending node (stored at start of message)


    // Decode the message to extract the part of _data meant for this device
    uint8_t toDevice;
    for (int i = 0; i < _numNodes - 1; i++) {
        toDevice = _data[datapointer];
        if (toDevice != _veryShortAddress) {
            incrementDataPointer(&datapointer);
        } else if (toDevice == _veryShortAddress) {
            processMessage(messagefrom, &datapointer);
        }

    }

}

void ConnectedRangingClass::handleRanges()
{
    for (int i = 0; i < _numNodes; i++) {
        _lastNode = &_networkNodes[i];
        if (_lastNode->getStatus() == RANGE_RECEIVED || _lastNode->getStatus() == RANGE_REPORT_RECEIVED) {
            _handleNewRange();
        }
    }
}

void ConnectedRangingClass::retrieveState(uint16_t *ptr)
{
    float vx;
    float vy;
    float z;
    float ax;
    float ay;
    float yawr;
    memcpy(&vx, _data + *ptr, FLOAT_SIZE);
    *ptr += 4;
    memcpy(&vy, _data + *ptr, FLOAT_SIZE);
    *ptr += 4;
    memcpy(&z, _data + *ptr, FLOAT_SIZE);
    *ptr += 4;
    memcpy(&ax, _data + *ptr, FLOAT_SIZE);
    *ptr += 4;
    memcpy(&ay, _data + *ptr, FLOAT_SIZE);
    *ptr += 4;
    memcpy(&yawr, _data + *ptr, FLOAT_SIZE);
    *ptr += 4;
    _lastNode->setState(vx, vy, z, ax, ay, yawr);
}

// If a part of the message was not for the current device, skip the pointer ahead to next block of _data
void ConnectedRangingClass::incrementDataPointer(uint16_t *ptr)
{
    uint8_t msgtype = _data[*ptr + 1];
    *ptr += 1;
    switch (msgtype) {
    case POLL :
        *ptr += POLL_SIZE;
        break;
    case POLL_ACK :
        *ptr += POLL_ACK_SIZE;
        break;
    case RANGE :
        *ptr += RANGE_SIZE;
        break;
    case RANGE_REPORT :
        *ptr += RANGE_REPORT_SIZE;
        break;
    case RECEIVE_FAILED :
        *ptr += RECEIVE_FAILED_SIZE;
        break;
    }

}

// If a part of the message WAS meant for the current device, then extract the information from it and advance the data pointer to the next block of _data (this last step is actually redundant)
void ConnectedRangingClass::processMessage(uint8_t msgfrom, uint16_t *ptr)
{

    uint8_t msgtype = _data[*ptr + 1];
    *ptr += 2;
    //Serial.print(F("Message type is: ")); Serial.println(msgtype);
    //Serial.print(F("Last node is: ")); Serial.println(_lastNode->getVeryShortAddress());

    if (msgtype == POLL) {
        _lastNode->setStatus(POLL_RECEIVED);
        DW1000.getReceiveTimestamp(_lastNode->timePollReceived);
    } else if (msgtype == POLL_ACK) {
        _lastNode->setStatus(POLL_ACK_RECEIVED);
        DW1000.getReceiveTimestamp(_lastNode->timePollAckReceived);
    } else if (msgtype == RANGE) {
        _lastNode->setStatus(RANGE_RECEIVED);
        DW1000.getReceiveTimestamp(_lastNode->timeRangeReceived);
        _lastNode->timePollSent.setTimestamp(_data + *ptr);
        *ptr += 5;
        _lastNode->timePollAckReceived.setTimestamp(_data + *ptr);
        *ptr += 5;
        _lastNode->timeRangeSent.setTimestamp(_data + *ptr);
        *ptr += 5;
        DW1000Time TOF;
        computeRangeAsymmetric(_lastNode, &TOF);
        float distance = TOF.getAsMeters();
        _lastNode->setRange(distance);
        if (_handleNewRange == 0) {
            Serial.print(F(" Range to device "));
            Serial.print(_lastNode->getVeryShortAddress());
            Serial.print(F(" is: "));
            Serial.print(distance);
            Serial.print(F(" m, update frequency is: "));
            Serial.print(_lastNode->getRangeFrequency());
            Serial.println(F(" Hz"));
        } else {
            _handleNewRange();
        }

    } else if (msgtype == RANGE_REPORT) {
        _lastNode->setStatus(RANGE_REPORT_RECEIVED);
        float curRange;
        memcpy(&curRange, _data + *ptr, 4);
        *ptr += 4;
        _lastNode->setRange(curRange);
        if (_handleNewRange == 0) {
            Serial.print(F(" Range to device "));
            Serial.print(_lastNode->getVeryShortAddress());
            Serial.print(F(" is: "));
            Serial.print(curRange);
            Serial.print(F(" m, update frequency is: "));
            Serial.print(_lastNode->getRangeFrequency());
            Serial.println(F(" Hz"));
        } else {
            _handleNewRange();
        }

    } else if (msgtype == RECEIVE_FAILED) {
        _lastNode->setStatus(INIT_STATUS);
    }
}

// Compute range according to assymmetric two way ranging
void ConnectedRangingClass::computeRangeAsymmetric(DW1000Device* myDistantDevice, DW1000Time* myTOF)
{
    DW1000Time round1 = (myDistantDevice->timePollAckReceived - myDistantDevice->timePollSent).wrap();
    DW1000Time reply1 = (myDistantDevice->timePollAckSent - myDistantDevice->timePollReceived).wrap();
    DW1000Time round2 = (myDistantDevice->timeRangeReceived - myDistantDevice->timePollAckSent).wrap();
    DW1000Time reply2 = (myDistantDevice->timeRangeSent - myDistantDevice->timePollAckReceived).wrap();

    myTOF->setTimestamp((round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2));

}

// Acquire transmit timestamps
void ConnectedRangingClass::updateSentTimes()
{
    DW1000Node* distantNode;
    for (int i = 0; i < _numNodes - 1; i++) {
        distantNode = &_networkNodes[i];
        if (distantNode->getStatus() == POLL_SENT) {
            DW1000.getTransmitTimestamp(distantNode->timePollSent);
        } else if (distantNode->getStatus() == POLL_ACK_SENT) {
            DW1000.getTransmitTimestamp(distantNode->timePollAckSent);
        }
    }
}

// Produce the data message to be sent
void ConnectedRangingClass::produceMessage()
{
    uint16_t datapointer = 0;
    memcpy(_data, &_veryShortAddress, 1);
    datapointer++;
    addStateToData(&datapointer);
    for (int i = 0; i < _numNodes - 1; i++) {
        addMessageToData(&datapointer, &_networkNodes[i]);
    }
}

void ConnectedRangingClass::addStateToData(uint16_t *ptr)
{
    State* selfState = _selfNode.getState();
    memcpy(_data + *ptr, &(selfState->vx), FLOAT_SIZE);
    *ptr += FLOAT_SIZE;
    memcpy(_data + *ptr, &(selfState->vy), FLOAT_SIZE);
    *ptr += FLOAT_SIZE;
    memcpy(_data + *ptr, &(selfState->z), FLOAT_SIZE);
    *ptr += FLOAT_SIZE;
    memcpy(_data + *ptr, &(selfState->ax), FLOAT_SIZE);
    *ptr += FLOAT_SIZE;
    memcpy(_data + *ptr, &(selfState->ay), FLOAT_SIZE);
    *ptr += FLOAT_SIZE;
    memcpy(_data + *ptr, &(selfState->yawr), FLOAT_SIZE);
    *ptr += FLOAT_SIZE;
}

void ConnectedRangingClass::addMessageToData(uint16_t *ptr, DW1000Node *distantNode)
{
    switch (distantNode->getStatus()) {
    case INIT_STATUS :
        addPollMessage(ptr, distantNode);
        break;
    case POLL_SENT :
        addReceiveFailedMessage(ptr, distantNode);
        break;
    case POLL_RECEIVED :
        addPollAckMessage(ptr, distantNode);
        break;
    case POLL_ACK_SENT :
        addReceiveFailedMessage(ptr, distantNode);
        break;
    case POLL_ACK_RECEIVED :
        addRangeMessage(ptr, distantNode);
        break;
    case RANGE_SENT :
        addReceiveFailedMessage(ptr, distantNode);
        break;
    case RANGE_RECEIVED :
        addRangeReportMessage(ptr, distantNode);
        break;
    case RANGE_REPORT_SENT :
        addReceiveFailedMessage(ptr, distantNode);
        break;
    case RANGE_REPORT_RECEIVED :
        addPollMessage(ptr, distantNode);
        break;
    }

}

void ConnectedRangingClass::addPollMessage(uint16_t *ptr, DW1000Node *distantNode)
{
    distantNode->setStatus(POLL_SENT);
    byte toSend[2] = {distantNode->getVeryShortAddress(), POLL};
    memcpy(_data + *ptr, &toSend, 2);
    *ptr += 2;
}

void ConnectedRangingClass::addPollAckMessage(uint16_t *ptr, DW1000Node *distantNode)
{
    distantNode->setStatus(POLL_ACK_SENT);
    byte toSend[2] = {distantNode->getVeryShortAddress(), POLL_ACK};
    memcpy(_data + *ptr, &toSend, 2);
    *ptr += 2;
}

void ConnectedRangingClass::addRangeMessage(uint16_t *ptr, DW1000Node *distantNode)
{
    distantNode->setStatus(RANGE_SENT);
    if (!_rangeSent) {
        _rangeSent = true;
        // delay the next message sent because it contains a range message
        DW1000Time deltaTime = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW1000Time::MICROSECONDS);
        _rangeTime = DW1000.setDelay(deltaTime);
    }
    distantNode->timeRangeSent = _rangeTime;
    byte toSend[2] = {distantNode->getVeryShortAddress(), RANGE};
    memcpy(_data + *ptr, &toSend, 2);
    *ptr += 2;
    distantNode->timePollSent.getTimestamp(_data + *ptr);
    *ptr += 5;
    distantNode->timePollAckReceived.getTimestamp(_data + *ptr);
    *ptr += 5;
    distantNode->timeRangeSent.getTimestamp(_data + *ptr);
    *ptr += 5;
}

void ConnectedRangingClass::addRangeReportMessage(uint16_t *ptr, DW1000Node *distantNode)
{
    distantNode->setStatus(RANGE_REPORT_SENT);
    byte toSend[2] = {distantNode->getVeryShortAddress(), RANGE_REPORT};
    memcpy(_data + *ptr, &toSend, 2);
    *ptr += 2;
    float range = distantNode->getRange();
    memcpy(_data + *ptr, &range, 4);
    *ptr += 4;
}

void ConnectedRangingClass::addReceiveFailedMessage(uint16_t *ptr, DW1000Node *distantNode)
{
    distantNode->setStatus(INIT_STATUS);
    byte toSend[2] = {distantNode->getVeryShortAddress(), RECEIVE_FAILED};
    memcpy(_data + *ptr, &toSend, 2);
    *ptr += 2;
}


// Reset functions
void ConnectedRangingClass::checkForReset()
{
    if (millis() - _lastActivity > INACTIVITY_RESET_TIME) {
        resetInactive();
    }
}

void ConnectedRangingClass::resetInactive()
{
    receiver();
    noteActivity();
}

void ConnectedRangingClass::noteActivity()
{
    _lastActivity = millis();
}

void ConnectedRangingClass::setSelfState(float vx, float vy, float z, float ax, float ay, float yawr)
{
    _selfNode.setState(vx, vy, z, ax, ay, yawr);
}

DW1000Node* ConnectedRangingClass::getDistantNode()
{
    return _lastNode;
}

void ConnectedRangingClass::printDataBytes()
{
    Serial.println(F("data bytes: "));
    for (uint16_t i = 0; i < _maxLenData; i++) {
        Serial.print(_data[i]);
        Serial.print(F(" "));
    }
    Serial.println(F(" "));
}

// Handle new self state value
void ConnectedRangingClass::handleNewSelfStateValue(float value, uint8_t type)
{
    _selfNode.setSingleState(value, type);
}

// Handle packed state
void ConnectedRangingClass::handleNewSelfPackedState(float* states, uint8_t type)
{
    if (type == PACKED_ALL) {
        _selfNode.setState(states[0], states[1], states[2], states[3], states[4], states[5]);
    }
}
