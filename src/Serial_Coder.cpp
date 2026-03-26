/*
 * Serial_Coder.c
 *
 *  Created on: Aug 28, 2017
 *      Author: steven <stevenhelm@live.nl>
 */

#include "Serial_Coder.h"

SerialCoderClass SerialCoder;

byte SerialCoderClass::_bytesRecvd = 0;
byte SerialCoderClass::_dataSentNum = 0;
byte SerialCoderClass::_dataRecvCount = 0;

byte SerialCoderClass::_dataSendCount = 0;
byte SerialCoderClass::_dataTotalSend = 0;

boolean SerialCoderClass::_inProgress = false;
boolean SerialCoderClass::_startFound = false;
boolean SerialCoderClass::_allReceived = false;

byte SerialCoderClass::_varByte = 0;

//MessageIn SerialCoderClass::_receiveMessages[IN_MESSAGES];
//selfState SerialCoderClass::_selfState;

// Handlers
void (* SerialCoderClass::_handleNewSelfStateValue)(float, uint8_t) = 0;
void (* SerialCoderClass::_handleNewSelfPackedState)(float*, uint8_t) = 0;

byte SerialCoderClass::_tempBuffer[MAX_MESSAGE];
byte SerialCoderClass::_tempBuffer2[MAX_MESSAGE];
byte SerialCoderClass::_recvBuffer[64];

boolean SerialCoderClass::_bigEndian = false;


SerialCoderClass::SerialCoderClass()
{
    /*
    for (uint8_t i=0;i<IN_MESSAGES;i++){
    	_receiveMessages[i].type = i;
    }
    for (uint8_t i=0;i<STATE_SIZE;i++){
    	_selfState.updated[i]=false;
    }
    checkBigEndian();
    */
}


/**
 * Function for receiving serial data.
 * Only receives serial data that is between the start and end markers. Discards all other data.
 * Stores the received data in _tempBuffer, and after decodes the high bytes and copies the final
 * message to the corresponding message in _messages.
 */
void SerialCoderClass::getSerialData()
{
    if (Serial.available() > 0) {
        _varByte = Serial.read();
        //Serial.print(F("read character is: "));
        //Serial.println(_varByte);
        if (_varByte == START_MARKER) {
            _bytesRecvd = 0;
            _inProgress = true;
        }

        if (_inProgress) {
            _tempBuffer[_bytesRecvd] = _varByte;
            _bytesRecvd++;
        }

        if (_varByte == END_MARKER) {
            _inProgress = false;
            _allReceived = true;

            decodeHighBytes();
        }
    }

}

/**
 * Function for decoding the high bytes of received serial data and saving the message.
 * Since the start and end marker could also be regular payload bytes (since they are simply the values
 * 254 and 255, which could also be payload data) the payload values 254 and 255 have been encoded
 * as byte pairs 253 1 and 253 2 respectively. Value 253 itself is encoded as 253 0.
 *  This function will decode these back into values the original payload values.
 */
void SerialCoderClass::decodeHighBytes()
{
    _dataRecvCount = 0;
    byte msgType = _tempBuffer[1];
    for (uint8_t i = 2; i < _bytesRecvd - 1; i++) { // Skip the begin marker (0), message type (1), and end marker (_bytesRecvd-1)
        _varByte = _tempBuffer[i];
        if (_varByte == SPECIAL_BYTE) {
            i++;
            _varByte = _varByte + _tempBuffer[i];
        }
        if (_dataRecvCount < sizeof(_recvBuffer)) {
            _recvBuffer[_dataRecvCount] = _varByte;
        }
        //Serial.print(F("Stored character is: "));
        //Serial.println(_varByte);
        _dataRecvCount++;
    }
    if (msgType == PACKED_ALL) {
        if (_dataRecvCount == 24) { // 6 floats
            float packed_state[6];
            memcpy(packed_state, &_recvBuffer, 24);
            if (_handleNewSelfPackedState) {
                _handleNewSelfPackedState(packed_state, msgType);
            }
        }
    } else if (_dataRecvCount == FLOAT_SIZE) {
        float tempfloat;
        memcpy(&tempfloat, &_recvBuffer, FLOAT_SIZE);
        if (_handleNewSelfStateValue) {
            _handleNewSelfStateValue(tempfloat, msgType);
        }
    }
}

/*
void SerialCoderClass::updateStateVar(byte msgFrom, byte msgType){
	switch(msgType){
	case VX : memcpy(&_selfState.vx,&_recvBuffer,4);break;
	case VY : memcpy(&_selfState.vy,&_recvBuffer,4);break;
	case Z  : memcpy(&_selfState.z,&_recvBuffer,4);break;


	}
	_selfState.updated[msgType]=true;
	if(stateUpdated()){
		_handleNewSelfState();
	}

}

boolean SerialCoderClass::stateUpdated(){

	boolean updated = true;
	for (uint8_t i=0;i<STATE_SIZE;i++){
		updated = updated && _selfState.updated[i];
	}
	return updated;
}*/

/**
 * Function used to receive a float with a certain message ID
 */
/*
float SerialCoderClass::receiveFloat(byte msgtype){
	float tempfloat;
	memcpy(&tempfloat,&_receiveMessages[msgtype].msg,4);
	return tempfloat;
}*/

/**
 * Function that will send a float over serial. The actual message that will be sent will have
 * a start marker, the from address, the message type, 4 bytes for the float, and the end marker.
 */
void SerialCoderClass::sendPackedState(byte thisAddress, byte remoteAddress, float* states)
{
    byte floatbytes[24];
    for (int i = 0; i < 6; i++) {
        memcpy(floatbytes + (i * 4), &states[i], 4);
    }
    encodeHighBytes(floatbytes, 24);
    Serial.write(START_MARKER);
    Serial.write(thisAddress);
    Serial.write(remoteAddress);
    Serial.write(PACKED_ALL);
    Serial.write(_tempBuffer2, _dataTotalSend);
    Serial.write(END_MARKER);
}

void SerialCoderClass::sendFloat(byte thisAddress, byte remoteAddress, byte msgtype, float outfloat)
{
    byte floatbyte[FLOAT_SIZE];
    memcpy(floatbyte, &outfloat, FLOAT_SIZE);
    encodeHighBytes(floatbyte, FLOAT_SIZE);
    Serial.write(START_MARKER);
    Serial.write(thisAddress);
    Serial.write(remoteAddress);
    Serial.write(msgtype);
    Serial.write(_tempBuffer2, _dataTotalSend);
    Serial.write(END_MARKER);

}

/**
 * Function that encodes the high bytes of the serial data to be sent.
 * Start and end markers are reserved values 254 and 255. In order to be able to send these values,
 * the payload values 253, 254, and 255 are encoded as 2 bytes, respectively 253 0, 253 1, and 253 2.
 */
void SerialCoderClass::encodeHighBytes(byte* sendData, uint8_t msgSize)
{
    _dataSendCount = msgSize;
    _dataTotalSend = 0;
    for (uint8_t i = 0; i < _dataSendCount; i++) {
        if (sendData[i] >= SPECIAL_BYTE) {
            _tempBuffer2[_dataTotalSend] = SPECIAL_BYTE;
            _dataTotalSend++;
            _tempBuffer2[_dataTotalSend] = sendData[i] - SPECIAL_BYTE;
        } else {
            _tempBuffer2[_dataTotalSend] = sendData[i];
        }
        _dataTotalSend++;
    }
}

/**
 * Function to check the endianness of the system
 */
void SerialCoderClass::checkBigEndian(void)
{
    union {
        uint32_t i;
        char c[4];
    } un = {0x01020304};

    _bigEndian = un.c[0] == 1;
}
