/*
 * Serial_Coder.h
 *
 *  Created on: Aug 28, 2017
 *      Author: steven <stevenhelm@live.nl>
 */

#ifndef SERIAL_CODER_H_
#define SERIAL_CODER_H_

#include <Arduino.h>
#include "DW1000Node.h"


#define MAX_MESSAGE 64
#define IN_MESSAGE_SIZE 4
#define OUT_MESSAGE_SIZE 8
#define END_MARKER 255
#define SPECIAL_BYTE 253
#define START_MARKER 254
#define IN_MESSAGES 3
#define OUT_MESSAGES 1
#define FLOAT_SIZE 4


/**
 * This class is used for encoding, decoding, sending, and receiving messages that will be sent over Arduino's serial communication.
 * The purpose is to make this sending more robust by implementing features like start and end characters to make sure messages are sent and received in whole.
 * Furthermore it will dedicate a byte to indicate the type of message, such that multiple different messages can be sent depending on the indicator byte.
 * A message will therefore look like {<start character> <message type> <message payload> <end character>}
 */
class SerialCoderClass
{

public:
    SerialCoderClass();
    static void getSerialData();
    static void processData();
    static void decodeHighBytes();

    //static float receiveFloat(byte msgtype);


    static void sendFloat(byte thisAddress, byte remoteAddress, byte msgtype, float outfloat);
    static void sendPackedState(byte thisAddress, byte remoteAddress, float* states);
    static void encodeHighBytes(byte* sendData, uint8_t msgSize);

    static void checkBigEndian();

    static void attachStateHandle(void (* handleNewSelfStateValue)(float, uint8_t))
    {
        _handleNewSelfStateValue = handleNewSelfStateValue;
    };
    static void attachPackedStateHandle(void (* handleNewSelfPackedState)(float*, uint8_t))
    {
        _handleNewSelfPackedState = handleNewSelfPackedState;
    };
    //static void updateStateVar(byte msgFrom,byte msgType);
    //static boolean stateUpdated();



protected:
    static byte _bytesRecvd;
    static byte _dataSentNum;
    static byte _dataRecvCount;


    static byte _tempBuffer[MAX_MESSAGE];
    static byte _tempBuffer2[MAX_MESSAGE];
    static byte _recvBuffer[64];

    static byte _dataSendCount;
    static byte _dataTotalSend;

    static boolean _inProgress;
    static boolean _startFound;
    static boolean _allReceived;

    //static MessageIn _receiveMessages[IN_MESSAGES];
    //static selfState _selfState;

    static byte _varByte;

    static boolean _bigEndian;

    // Handlers
    static void (* _handleNewSelfStateValue)(float, uint8_t);
    static void (* _handleNewSelfPackedState)(float*, uint8_t);

};

extern SerialCoderClass SerialCoder;

#endif /* SERIAL_CODER_H_ */
