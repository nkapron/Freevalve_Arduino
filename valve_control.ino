// This code is based on Wesley Kagan's work here:
// https://github.com/Tamaren1/Freevalve_Arduino/blob/main/ArduinoHallEffect_good_code.ino
// His work is documented on YouTube here: https://youtu.be/k_MrPylnsDg

// This will code is written with the assumption that your timing wheel's gap is aligned 
// with Top Dead Center (TDC), and (for now) the engine has to be at TDC (or close) when
// you reset the arduino.

#include <limits.h> // We need this to reference ULONG_MAX

//Configuration
const byte hall_pin = 2;
const byte exhaust_pin = 12;
const byte intake_pin = 13;
const unsigned int degrees_per_pulse = 6; 
const byte pulses_per_rev = 59; //Number of magnets on your timing wheel
const bool debug_print = true;

// These values are based on your comments in github,
// but this doesn't make a whole lot of sense to me.
// Where is your compression cycle? You open the exhaust
// valve right as you close the intake...? That would explain why there's
// what looks like unburnt fuel coming out of the engine when running
// (and also probably why it doesn't rev)
// Additionally, the exhaust ovelaps the intake (102 > 30) so you're not
// pulling all the air through the carb. 

//const unsigned int valve_timing[2][] = {
//  {30, 252}, //Intake open, close
//  {252, 102} //Exhaust open, close
//};

// Expected generic timing. Note the intake is the first rotation, exhaust is second.
// Note: I've added the compression and combustion cycles in the timing table as comments
// for clarity, but do not uncomment them.
const unsigned int valve_timing[2][2] = {
// First rotation: intake and compression
  {0, 180}, // Intake open, close
//{181, 365}, // Compression cycle, no valves open
// Second rotation: combustion and exhaust
//{0, 180}, // Combustion cylce, no valves open  
  {180, 365 } //Exhaust open, close
};

// Communication variables from interrupt to main loop.
volatile unsigned long current_timestamp = 0;
volatile unsigned int pulse_count = 0;

// Other global variables... 

unsigned long processing_timestamp = 0;
unsigned int processing_pulse_count = 0;
unsigned long revolutions = 0;
unsigned long last_timestamp = 0;
unsigned long elapsed_time;
unsigned long last_elapsed_time;
static unsigned int rotation_pulse_count = 0;

unsigned int degree;
unsigned int state;

bool exhaust_state = false;

bool intake_open = false;
bool exhaust_open = false;
bool state_printed = false;
bool debug = true;

void setup() {
  pinMode(hall_pin, INPUT); // sets the digital pin 2 as an input
  pinMode(exhaust_pin, OUTPUT); // sets the digital pin 12 as an output
  pinMode(intake_pin, OUTPUT); // sets the digital pin 13 as an output

  // This could be changed from RISING to CHANGE for more resolution.
  // Doing so would cause callbacks on both the rising edge and falling edge
  // of the pulse produced by the hall effect sensor, theoretically changing
  // the elapsed degrees from 6* to 3*, but some measurement would need to be
  // done to determine if the sensor measures symetrically, or if it
  // releases slower/faster than it 'latches'
  attachInterrupt(digitalPinToInterrupt(hall_pin), pulse_detect_callback, RISING); //Initialize the interrupt pin digital pin 2
  
  Serial.begin(115200);
  
  degree = 0;
  Serial.print("Init complete");
}

// This is called every time there is (currently) a rising edge on the hall effect sensor.
// More resolution could be added by changing RISING to CHANGE in setup()->attachInterrupt,
// would need to change the math for degrees and whatnot.

// Interrupts need to be held as short as possible- no printing to serial, etc.
// For more reading, check out https://roboticsbackend.com/arduino-interrupts/
// Changed millis() to micros() for more resolution/timestamp accuracy.
void pulse_detect_callback() {
  //We want this to be the first thing that happens
  current_timestamp = micros();
  
  // Instead of just setting a boolean flag, we increase a counter.
  // This way, we can detect a processing overrun condition where two or more pulses occured
  // before we could process them in loop()
  pulse_count++;
}

bool wait_for_pulse() {
  static unsigned int last_pulse_count = 0;
  while( last_pulse_count == pulse_count) {
    //Could delay here, but we 
    //delay();

    // could add a timeout here and return false...
    // TODO: check for timeout
    //return false;
  }
  if(pulse_count != last_pulse_count+1) {
    Serial.println("Skipped pulse");
  }
  last_pulse_count = pulse_count;
  return true;
}

void loop() {
  // One thing that is not clear to me is how you are accounting for the ignition.
  // Is is just luck when it runs (or you have to set the engine to TDC, reset the arduino, then pull the starter?)
  // Seems like it needs knowledge or control of the spark.

  if(wait_for_pulse()) {
    //Pulse detected within the timeout

    //First, we want to create a copy of the timestamp since it can be changed at any time by the ISR.
    processing_timestamp = current_timestamp;
    processing_pulse_count = pulse_count;

    //Have to check for timer overflow. We assume it is only a single overflow...
    if(processing_timestamp < last_timestamp) {
      elapsed_time = ULONG_MAX - last_timestamp;
      elapsed_time += processing_timestamp;
    } else {
      elapsed_time = processing_timestamp - last_timestamp;
    }
    
    // Checking for the long pulse (what should be TDC, but I'm not sure if it is TDC on your timing gear or not...)
    // This was >= 2 in your code, but I'd think theres some variability, so this chunk of code is essentially > 1.7
    // To avoid floating point math (slow), we multiply by 10
    bool val_overflow = false;
    unsigned long big_elapsed = elapsed_time * 10;
    if (big_elapsed < elapsed_time) {
      //Overflow...
      val_overflow = true;
      Serial.println("big_elapsed overflow");
    }
    // To avoid floating point math, we multiply by 17
    unsigned long big_last_elapsed = last_elapsed_time * 17;
    if (big_last_elapsed < last_elapsed_time) {
      //Overflow...
      val_overflow = true;
      Serial.println("big_last_elapsed overflow");
    }
    if (!val_overflow) {
      if(big_elapsed > big_last_elapsed) {
        revolutions++;
        if(revolutions == 0) {
          //We're (theoretically) running and the counter overflowed, so skip 0 since that will enter initialization logic.
          revolutions++;
        }
        if( (revolutions != 0) && (rotation_pulse_count != pulses_per_rev)) {
          Serial.print("Unexpected number of pulses: ");
          Serial.print(rotation_pulse_count, DEC);
          Serial.println();
        }
        rotation_pulse_count = 1;
        exhaust_state = !exhaust_state;
      }
    }

    unsigned int rotation_degrees = rotation_pulse_count * degrees_per_pulse;
    if (exhaust_state) {
      //First, close the intake if it is still open
      if(intake_open) {
        digitalWrite(intake_pin, LOW);  //Close the intake valve
        intake_open = false;
      }
      //Now check the timing table to see if we should open the exhaust valve
      if((rotation_degrees >= valve_timing[exhaust_state][0]) &&
        (rotation_degrees < valve_timing[exhaust_state][1])) {
          if(!exhaust_open) {
            digitalWrite(exhaust_pin, HIGH);  //Open the exhaust valve
            exhaust_open = true;
          }
        } else if (exhaust_open) {
            digitalWrite(exhaust_pin, HIGH);  //Close the exhaust valve
            exhaust_open = false;
        }
    } else {
      //Intake state
      if( exhaust_open) {
        //First, close the exhaust if it is still open
        digitalWrite(exhaust_pin, LOW);  //Close the exhaust valve
        exhaust_open = false;
      }
      //Now check the timing table to see if we should open the intake valve
      if((rotation_degrees >= valve_timing[exhaust_state][0]) &&
        (rotation_degrees < valve_timing[exhaust_state][1])) {
          if(!intake_open) {
            digitalWrite(intake_pin, HIGH);  //Open the intake valve
            intake_open = true;
          }
        } else if (intake_open) {
            digitalWrite(intake_pin, HIGH);  //Close the intake valve
            intake_open = false;
        }
    }
    
    //Update the state
    last_timestamp = processing_timestamp;
    last_elapsed_time = elapsed_time;
    state_printed = false;
  } else {
    if((debug_print) && (!state_printed)) {
      Serial.print("No pulse detected");
      state_printed = true;
    }
  }
}
