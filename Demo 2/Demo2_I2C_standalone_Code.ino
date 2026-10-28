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
// 2nd byte of info -> stores how far away the ardunio is from the beacon (not units between the Pi and Arduino), max value is 255
volatile int distance_away = 0;
// 3rd byte of info -> stores the angle the displacement of the marker once the robot detects the marker, max value is -128 to 127 degrees which should be plenty for our camera FOV
// Probably best to use 2's complement, but we can transfer the negative number in a few different ways if that would be easier on the Pi side
volatile int angle_away = 0;
// Flag that checks if we got new data, used in conjuction with turning interrupts off when we store the volatile data into a stable data type so it can't change on us mid-read
volatile bool new_Data = 0;

// note: all the max values above are based on the type casting below, if we need higher numbers we can just read in two bytes of data and add the values to get the larger numbers
// note: for distance and angle, so far I have been writing it assuming whole numbers (ints), will need to figure out how to do floating point

void CV_Info(int numBytes) {
  // checks how many bytes we got, if it != 3, then drain then get rid of the data (quickly read it without storing it anywhere)
  if (numBytes != 3) {
    while (Wire.available()) Wire.read();
    return;
  }

  byte beacon_info = (uint8_t)Wire.read(); // type casted to specify that it is 1 byte wide and an unsigned integer
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
  distance_away = (uint8_t)Wire.read(); // type casted to specify that it is 1 byte wide and an unsigned integer
  angle_away = (int8_t)Wire.read(); // type casted to specify that it is 1 byte wide and a signed integer
  new_Data = 1;
}

void loop() {
  if (newData) {
    // Copy with interrupts off so a new packet can't change values mid-read
    noInterrupts();
    bool b_det = beacon_detected;
    bool n_turn = no_turn;
    bool a_dir = arrow_direction;
    int dist = distance;
    int  ang  = angle;
    newData = 0;
    interrupts();
  }
}