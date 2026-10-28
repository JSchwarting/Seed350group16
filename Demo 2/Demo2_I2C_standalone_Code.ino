#include <Wire.h>
#define Arduino_Addr 0x08

void setup() {
  Wire.begin(Arduino_Addr);
  Wire.onReceive(CV_Info);
}

// 1st byte of info -> stores when we detect the beacon (for test 1 and 2) and what direction the arrow it sees is (for test 2)
// Idea for how to set up this first byte could be to have bit 0 be whether we detected a marker, while bits 1 and 2 are for the arrow
// TT for bits 1 and 2: 00 -> doesn't detect an arrow, 01 -> left facing arrow, 10 -> right facing arrow (essentially flip bit 1 if we need to go left, flip bit 2 if we need to go right)
// note: in order to get test 1 to work, should add a flag that prevents us from entering the turn in the arduino once we reach the beacon
volatile bool beacon_detected = 0;
// 0 means we need to go left, 1 means we need to go right
volatile bool arrow_direction = 0;
// 0 means we need to turn, 1 means don't turn
volatile bool no_turn = 1;
// Bytes 2-5 of info -> stores how far away the ardunio is from the beacon (note units between the Pi and Arduino)
volatile float distance_away = 0;
// bytes 6-9 of info -> stores the angle the displacement of the marker once the robot detects the marker
volatile float angle_away = 0;
// Flag that checks if we got new data, used in conjuction with turning interrupts off when we store the volatile data into a stable data type so it can't change on us mid-read
volatile bool new_Data = 0;
// note: distance_away and anlge_away are transferred as floats which automatically account for negative values, not sure what the #.# format is, look it up to make sure we code it correctly on the Pi side

void CV_Info(int numBytes) {
  // checks how many bytes we got, if it != 9, then drain then get rid of the data (quickly read it without storing it anywhere)
  if (numBytes != 9) {
    while (Wire.available()) Wire.read();
    return;
  }

  // checking individual bits for the values we need (5 extra bits for more information to carry in future if needed)
  byte beacon_info = Wire.read();
  beacon_detected = bitRead(beacon_info, 0);
  if (bitRead(beacon_info, 1) == 1) {
    arrow_direction = 0;
    no_turn = 0;
  } else if (bitRead(beacon_info, 2) == 1) {
    arrow_direction = 1;
    no_turn = 0;
  } else {
    no_turn = 1;
  }

  // complicated since we are communicating floating point values (4 bytes for each float value, sign is transferred automatically)
  float d, a;
  uint8_t geo_values[8];
  for (int i = 0; i < 8; i++) geo_values[i] = Wire.read();
  memcpy(&d, &geo_values[0], sizeof(float));
  memcpy(&a, &geo_values[4], sizeof(float));
  distance_away = d;
  angle_away = a;
  new_Data = 1;
}

void loop() {
  // I2C code to extract values for our data
  if (new_Data) {
    // Copy with interrupts off so a new packet can't change values mid-read
    noInterrupts();
    bool b_det = beacon_detected;
    bool n_turn = no_turn;
    bool a_dir = arrow_direction;
    float dist = distance_away;
    float  ang  = angle_away;
    new_Data = 0;
    interrupts();
  }
}