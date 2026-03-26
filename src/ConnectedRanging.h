/*
 * ConnectedRanging.h
 *
 *  Created on: Aug 18, 2017
 *      Author: steven <stevenhelm@live.nl>
 */

#ifndef _CONNECTEDRANGING_H_INCLUDED
#define _CONNECTEDRANGING_H_INCLUDED




#include "DW1000.h"
#include "DW1000Time.h"
#include "DW1000Node.h"
#include "Serial_Coder.h"



// reset time in ms
#define DEFAULT_RESET_TIME 100
#define INACTIVITY_RESET_TIME 2*DEFAULT_RESET_TIME



// messages used in the ranging protocol
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RECEIVE_FAILED 255

// number of bytes a certain message type takes up
#define POLL_SIZE 1
#define POLL_ACK_SIZE 1
#define RANGE_SIZE 16 // 1 + 15
#define RANGE_REPORT_SIZE 5 // 1 + 4
#define RECEIVE_FAILED_SIZE 1



// reply time in us
#define DEFAULT_REPLY_DELAY_TIME 5000

#define STATE_SIZE 24

#define FLOAT_SIZE 4

// maximum number of nodes that can be connected. Warning: Consumes a lot of SRAM. Need to optimize if going beyond 4 nodes.
#define MAX_NODES 4
#define MAX_LEN_DATA (MAX_NODES-1)*RANGE_SIZE+MAX_NODES+STATE_SIZE

// Serial message types
#define VX 0
#define VY 1
#define Z 2
#define R 3


class ConnectedRangingClass
{

public:
    // data buffer
    static byte _data[MAX_LEN_DATA];

    // initialisation
    static void init(char longAddress[], uint8_t numNodes);
    static void init(uint8_t veryShortAddress, uint8_t numNodes);
    static void initDecawave(byte longAddress[], uint8_t numNodes, const byte mode[] = DW1000.MODE_LONGDATA_RANGE_ACCURACY, uint16_t networkID = 0xDECA, uint8_t myRST = 9, uint8_t mySS = SS, uint8_t myIRQ = 2);

    // set DW1000 in permanent receiving mode
    static void receiver();

    // transmit functions
    static void transmitInit();
    static void transmitData(byte datas[]);
    static void transmitData(char datas[]);
    static void transmitData(char datas[], uint16_t n);
    static void transmitData(byte datas[], DW1000Time timeDelay);

    // main loop
    static void loop();

    // reset functions
    static void checkForReset();
    static void resetInactive();
    static void noteActivity();

    // handlers
    static void handleSent();
    static void handleReceived();
    static void handleRanges();
    static void attachNewRange(void (* handleNewRange)(void))
    {
        _handleNewRange = handleNewRange;
    };

    // received message parsing and handling
    static void handleReceivedData();
    static void incrementDataPointer(uint16_t *ptr);
    static void processMessage(uint8_t msgfrom, uint16_t *ptr);
    static void computeRangeAsymmetric(DW1000Device* myDistantDevice, DW1000Time* myTOF);
    static void retrieveState(uint16_t *ptr);

    // sent message handling
    static void updateSentTimes();


    // producing the transmit message
    static void produceMessage();
    static void addMessageToData(uint16_t *ptr, DW1000Node *distantNode);
    static void addPollMessage(uint16_t *ptr, DW1000Node *distantNode);
    static void addPollAckMessage(uint16_t *ptr, DW1000Node *distantNode);
    static void addRangeMessage(uint16_t *ptr, DW1000Node *distantNode);
    static void addRangeReportMessage(uint16_t *ptr, DW1000Node *distantNode);
    static void addReceiveFailedMessage(uint16_t *ptr, DW1000Node *distantNode);
    static void addStateToData(uint16_t *ptr);

    // Setting state variables when they come in via serial
    static void setSelfState(float vx, float vy, float z, float ax, float ay, float yawr);

    // Getters
    static DW1000Node* getDistantNode();

    static void printDataBytes();

    // Handling new state receive over serial
    static void handleNewSelfStateValue(float value, uint8_t type);
    static void handleNewSelfPackedState(float* states, uint8_t type);







protected:


    // pins on the arduino used to communicate with DW1000
    static uint8_t _RST;
    static uint8_t _SS;
    static uint8_t _IRQ;

    // addresses of current DW1000
    static byte _longAddress[LEN_EUI];
    static byte _veryShortAddress;

    // message sent/received state
    static volatile boolean _sentAck;
    static volatile boolean _receivedAck;

    // keeping track of send times
    static uint32_t _lastSent;

    // nodes to range with
    static DW1000Node _networkNodes[MAX_NODES];
    static DW1000Node* _lastNode;
    static uint8_t _numNodes;

    // self node
    static DW1000Node _selfNode;

    // initializing those nodes
    static void initNodes();

    // when it is time to send
    static boolean _timeToSend;

    // remembering future time in case a RANGE message is sent
    static boolean _rangeSent;
    static DW1000Time _rangeTime;

    // extended frame;
    static boolean _extendedFrame;

    static uint16_t protTimes;

    // reset variable
    static uint32_t _lastActivity;

    static uint16_t _maxLenData;


    // Handlers
    static void (* _handleNewRange)(void);













};

extern ConnectedRangingClass ConnectedRanging;

#endif /* _CONNECTEDRANGING_H_INCLUDED */






