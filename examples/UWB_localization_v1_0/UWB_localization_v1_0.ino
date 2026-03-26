#include <ConnectedRanging.h>

/**
 * Only variables that need changing. Change numNodes to the number of drones that will be flying. 
 * On each drone, assign a unique veryShortAddress in incremental fashion starting from 1. So the first drone gets address 1, the second 2, etc. up to numNodes.
 * Notes: only tested with 2 drones. This arduino code has been tested to work with up to 4 nodes, but the paparazzi side only up to 2.
 */
uint8_t numNodes = 2;
uint8_t veryShortAddress = 2;


void setup() {
  Serial.begin(57600);
  ConnectedRanging.init(veryShortAddress,numNodes);
  ConnectedRanging.setSelfState(0.0,0.0,0.0,0.0,0.0,0.0);
  ConnectedRanging.attachNewRange(newRange);

}

void loop() {
  ConnectedRanging.loop();  
}

void newRange(){
  DW1000Node* lastNode = ConnectedRanging.getDistantNode();
  uint8_t messageFrom = lastNode->getVeryShortAddress();
  State* remoteState = lastNode->getState();  

  
  SerialCoder.sendFloat(veryShortAddress,messageFrom,R,remoteState->r);
  SerialCoder.sendFloat(veryShortAddress,messageFrom,VX,remoteState->vx);
  SerialCoder.sendFloat(veryShortAddress,messageFrom,VY,remoteState->vy);
  SerialCoder.sendFloat(veryShortAddress,messageFrom,Z,remoteState->z);  
  
  /*
  Serial.print(F(" Range to ")); Serial.print(messageFrom);Serial.print(F(" is: ")); Serial.print(remoteState->r);
  Serial.print(F(" m, state vx, vy, z is: ")); Serial.print(remoteState->vx);Serial.print(F(", "));
  Serial.print(remoteState->vy);Serial.print(F(", "));  Serial.print(remoteState->z);
  Serial.print(F(" m, update frequency is: "));Serial.print(lastNode->getRangeFrequency()); Serial.println(F(" Hz"));
  */
}
