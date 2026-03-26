/*
 * DW1000Node.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: steven <stevenhelm@live.nl>
 */



#include "DW1000Node.h"



/**###############################################################################################################
 * ###########################   Constructors and Destructors    #################################################
 * ###############################################################################################################
 */
DW1000Node::DW1000Node() : DW1000Device()
{
    _veryShortAddress = _shortAddress[0];
    _state.vx = 0;
    _state.vy = 0;
    _state.z = 0;
}

DW1000Node::DW1000Node(byte address[], byte shortAddress[])  : DW1000Device(address, shortAddress)
{
    _veryShortAddress = address[0];
    _state.vx = 0;
    _state.vy = 0;
    _state.z = 0;
}

DW1000Node::DW1000Node(byte address[], boolean shortOne) : DW1000Device(address, shortOne)
{
    _veryShortAddress = address[0];
    _state.vx = 0;
    _state.vy = 0;
    _state.z = 0;
}

DW1000Node::~DW1000Node()
{
}

/**###############################################################################################################
 * ###########################              Getters              #################################################
 * ###############################################################################################################
 */
float DW1000Node::getRange()
{
    return _state.r;
}

uint8_t DW1000Node::getStatus()
{
    return _status;
}

byte DW1000Node::getVeryShortAddress()
{
    return _veryShortAddress;
}

State* DW1000Node::getState()
{
    return &_state;
}

/**###############################################################################################################
 * ###########################              Setters              #################################################
 * ###############################################################################################################
 */
void DW1000Node::setRange(float range)
{
    _state.r = range;
}

void DW1000Node::setStatus(uint8_t status)
{
    _status = status;
    return;
}

void DW1000Node::setState(float vx, float vy, float z, float ax, float ay, float yawr, float r)
{
    _state.vx = vx;
    _state.vy = vy;
    _state.z = z;
    _state.ax = ax;
    _state.ay = ay;
    _state.yawr = yawr;
    _state.r = r;
    for (uint8_t i = 0; i < STATE_VAR_SIZE; i++) {
        _state.stateUpdate[i] = true;
    }
}

void DW1000Node::setSingleState(float value, uint8_t type)
{
    switch (type) {
    case VX:
        _state.vx = value;
        _state.stateUpdate[VX] = true;
        break;
    case VY:
        _state.vy = value;
        _state.stateUpdate[VY] = true;
        break;
    case Z:
        _state.z = value;
        _state.stateUpdate[Z] = true;
        break;
    case AX:
        _state.ax = value;
        _state.stateUpdate[AX] = true;
        break;
    case AY:
        _state.ay = value;
        _state.stateUpdate[AY] = true;
        break;
    case YAWR:
        _state.yawr = value;
        _state.stateUpdate[YAWR] = true;
        break;
    case RANGE:
        _state.r = value;
        _state.stateUpdate[RANGE] = true;
        break;
    }
}

boolean DW1000Node::isStateUpdated()
{
    boolean checker = true;
    for (uint8_t i = 0; i < STATE_VAR_SIZE; i++) {
        checker = checker && _state.stateUpdate[i];
    }
    if (checker) {
        for (uint8_t i = 0; i < STATE_VAR_SIZE; i++) {
            _state.stateUpdate[i] = false;
        }
        return true;
    } else {
        return false;
    }
}

/**###############################################################################################################
 * ###########################             Operators             #################################################
 * ###############################################################################################################
 */
boolean DW1000Node::operator==(const uint8_t cmp) const
{
    return _veryShortAddress == cmp;
}

boolean DW1000Node::operator!=(const uint8_t cmp) const
{
    return _veryShortAddress != cmp;
}


/**###############################################################################################################
 * ###########################          Utility Functions        #################################################
 * ###############################################################################################################
 */
void DW1000Node::printNode()
{
    Serial.print(F("Node with Very Short Address: "));
    Serial.print(_veryShortAddress, HEX);
    Serial.print(F(" and Long Address: "));
    for (int j = 0; j < 8; j++) {
        Serial.print(_ownAddress[j], HEX);
    }
    Serial.print(F(" and Status: "));
    Serial.println(_status);

}

float DW1000Node::getRangeFrequency()
{
    if (_successRanges < 10) {
        _successRanges++;
    } else {
        uint32_t newTime = millis();
        _rangeFreq = ((float)_successRanges) / (((float)(newTime - _rangeTimer)) / 1000.0);
        _rangeTimer = newTime;
        _successRanges = 0;
    }
    return _rangeFreq;
}
