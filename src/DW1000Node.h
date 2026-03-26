/*
 * DW1000Node.h
 *
 *  Created on: Aug 18, 2017
 *      Author: steven <stevenhelm@live.nl>
 */

#ifndef _DW1000Node_H_INCLUDED
#define _DW1000Node_H_INCLUDED

#include "DW1000Device.h"
#include "Serial_Coder.h"

// These are the statusses that a node can be set to
#define INIT_STATUS 0
#define POLL_SENT 1
#define POLL_RECEIVED 2
#define POLL_ACK_SENT 3
#define POLL_ACK_RECEIVED 4
#define RANGE_SENT 5
#define RANGE_RECEIVED 6
#define RANGE_REPORT_SENT 7
#define RANGE_REPORT_RECEIVED 8

// Size of the state of a node (should correspond to the number of float variables present in State struct)
#define STATE_VAR_SIZE 7

// State types (corresponding to message types being exchanged over serial)
#define RANGE 0
#define VX 1
#define VY 2
#define Z 3
#define AX 4
#define AY 5
#define YAWR 6
#define PACKED_ALL 7

// Struct that holds the state of a certain node
struct State {
    float vx;
    float vy;
    float z;
    float ax;
    float ay;
    float yawr;
    float r;
    boolean stateUpdate[STATE_VAR_SIZE];
};

/**
 * This class is a helper class for the ranging protocol defined in ConnectedRangingClass.h.
 * This class inherits from the DW1000Device class, since a node in the ConnectedRanging protocol
 * is essential a DW1000Device with some additional functionalities.
 */
class DW1000Node: public DW1000Device
{
public:
    // Constructors and Destructor. Mostly just calls the corresponding DW1000Device constructors and initializes state vars.
    DW1000Node();
    DW1000Node(byte address[], byte shortAddress[]);
    DW1000Node(byte address[], boolean shortOne = true);
    ~DW1000Node();

    // Getters
    uint8_t getStatus();
    byte getVeryShortAddress();
    State* getState();
    float getRange();

    // Setters
    void setStatus(uint8_t status);
    void setState(float vx, float vy, float z, float ax, float ay, float yawr, float r = 0.0); // Default r = 0.0. This allows easy using of the same node class when the node is the current device.
    void setSingleState(float value, uint8_t type);
    void setRange(float range);

    boolean isStateUpdated();

    // Comparisons
    boolean operator==(const uint8_t cmp) const;
    boolean operator!=(const uint8_t cmp) const;

    // Utility functions
    void printNode();
    float getRangeFrequency();

protected:
    /**
     * _status is used when this node is a distant node for the device operating.
     * Depending on what stage in the protocol the distant node is, it gets assigned a status value.
     * This allows the current device to keep track of where in the protocol the different nodes are.
     */
    uint8_t _status = 0;


    /**
     * This class uses a single byte to store the address of a node. This is more than enough since that means it already supports 255
     * devices in principle (first address is 1 and not 0, don't ask why). Since this is way more than I see will ever be needed, I use
     * only a single byte. The downside of course is that this is not in line with the IEEE UWB standard.
     */
    byte _veryShortAddress = 0;

    // Counter used to determine the update frequency to this node
    uint32_t _successRanges = 0;

    // Auxiliary variables used to compute update frequency
    uint32_t _rangeTimer = millis();
    float _rangeFreq = 0;

    /**
     * The state of the current node. In case the current node is a distant node, then the state will have a non-zero range value
     * if the protocol is successful. However, the same node class is also used to represent the device itself, in which case the
     * range value will obviously be 0.
     */
    State _state;

};

#endif
