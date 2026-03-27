#include <ConnectedRanging.h>

/*
 * =========================================================================================
 * UWB SWARM LOCALIZATION NODE
 * =========================================================================================
 * This code runs on an Arduino connected to a Decawave DW1000 Ultra-Wideband (UWB) module.
 * It is designed to act as a "Swarm Node" that measures the physical distance (ranging) 
 * between itself and other flying drones in a peer-to-peer network.
 * 
 * SWARM CONFIGURATION LIMITS:
 * - 'numNodes' defines the total number of UWB tags participating in the token ring. 
 * - 'veryShortAddress' is the unique ID of THIS specific drone. 
 * Assign node IDs sequentially starting from 1 (e.g., Drone 1 = 1, Drone 2 = 2, etc.).
 * 
 * NOTE: Both the ConnectedRanging library and Paparazzi's dw1000_arduino C-modules
 * perfectly support tracking up to 8 nodes simultaneously.
 */
uint8_t numNodes = 2;
uint8_t veryShortAddress = 2;


void setup() {
  // We use 115200 baud to ensure the Arduino can push range data to the flight controller
  // faster than the UWB chip can generate new distance measurements. 
  Serial.begin(115200); 

  // Initialize the UWB hardware and join the peer-to-peer network schedule
  ConnectedRanging.init(veryShortAddress, numNodes);
  
  // Initialize the internal state. (vx, vy, z, ax, ay, yawr)
  // This can later be populated with data from the flight controller if we want 
  // to broadcast our velocity to other drones over UWB.
  ConnectedRanging.setSelfState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  
  // Attach the callback function. Whenever the DW1000 successfully calculates 
  // a distance to another drone, it will automatically fire "newRange()".
  ConnectedRanging.attachNewRange(newRange);
}

void loop() {
  // Drives the internal Two-Way Ranging (TWR) state machine. It handles listening
  // for radio pulses, responding at exact microsecond timings, and computing the math.
  ConnectedRanging.loop();  
}

/*
 * Packages the raw distance into a strict byte array format that Paparazzi's 
 * flight controller (dw1000_arduino.c) knows how to decode.
 * Packet Structure (8 bytes): 
 * [0xFE Header] [2 bytes ID] [4 bytes Float Distance] [1 byte Checksum]
 */
inline void sendToPaparazzi(uint16_t id, float distance) {
  // If the serial transmit buffer is full, drop the packet completely. 
  // We cannot afford to "wait" for the buffer to clear, because a blocked 
  // Arduino will miss its highly sensitive microsecond UWB transmit window 
  // and crash the entire Swarm token ring.
  if (Serial.availableForWrite() < 8) {
    return; 
  }

  uint8_t packet[8];
  packet[0] = 0xFE; // Sync byte telling Paparazzi a new message is starting

  // Copy ID and distance efficiently into the packet buffer.
  // Little-endian memory mapping matches Paparazzi's native struct extraction.
  memcpy(&packet[1], &id, sizeof(uint16_t));
  memcpy(&packet[3], &distance, sizeof(float));

  // Compute a simple rolling checksum so the flight controller knows the UART signal wasn't corrupted.
  // Unrolling this mathematically is faster than using a for-loop.
  uint8_t ck = packet[1] + packet[2] + packet[3] + packet[4] + packet[5] + packet[6];
  packet[7] = ck;

  // Single batch write operation ensures UART interrupts don't sequence-break strings
  Serial.write(packet, 8);
}

/*
 * This is the callback triggered every time a distance measurement finishes.
 */
void newRange() {
  // Get the pointer to the drone we just finished ranging with
  DW1000Node* lastNode = ConnectedRanging.getDistantNode();
  
  // Hardware/RF dropouts happen. Prevent null-pointer dereferencing.
  if (lastNode == nullptr) {
    return;
  }

  State* remoteState = lastNode->getState();  
  if (remoteState == nullptr) {
    return;
  }

  // Extract the computed distance constraint in meters
  float range = remoteState->r;

  // Real-world UWB signals can bounce off walls or drone frames (multipath interference/ghosting),
  // which sometimes results in crazy math outputs mathematically.
  // This bounds filter prevents NaN, Infinity, negative values, and literal 0s 
  // from poisoning the Extended Kalman Filter (EKF) on the flight controller.
  if (isnan(range) || isinf(range) || range < 0.01f || range > 1000.0f) {
    return;
  }

  // Grab the ID of the drone we measured the distance to
  uint16_t messageFrom = (uint16_t)lastNode->getVeryShortAddress();

  // Route the strictly validated telemetry directly out to the Paparazzi flight controller
  sendToPaparazzi(messageFrom, range);
}

