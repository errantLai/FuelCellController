#include <math.h>

// FC States
#define FC_INITIAL 0
#define FC_STANDBY 1
#define FC_STARTUP 2
#define FC_RUN 3
#define FC_ALARM 4
#define FC_SHUTDOWN 5

//FC Substates
#define FC_STARTUP_FAN_SPOOLUP 6
#define FC_STARTUP_STARTUP_PURGE 7 
#define FC_STARTUP_END 8

//FC Thermistor Pin
#define StackThermistorPin 1
#define StackTempFixedResistance 10000

int stackTempPos = 0, stackTempArraySize = 1000;
int[] stackTempArray = new int[stackTempArraySize];

int FC_State = FC_INITIAL; // initial state perhaps we could enumerate these
int FC_SubState = FC_STARTUP_FAN_SPOOLUP;

boolean fc_on = false;
boolean fc_alarm = false;

int CurrentRequest = 0;

void setup() {
  // set all of our variables to their initial values if needed, initiate measuring capabilities as needed
  purgeLastCallTime = millis();
}

void loop() {
  // we iterate through this switch case block infinitely 
  Check_Alarms();
  System();
  FC();
}

double Steinhart_Hart(int resistance, double a, double b, double c) {
  // Returns the temperature of a thermistor, based on the model's coefficients
  // These double values can be hard coded, or left as inputs for other temp readings
  // In arduino, log() is for the natural log implementation.
  double logR = log(resistance);
  double model = a + b*logR + c*(logR * logR * logR);
  return (1/model) - 273.15; // Account for Kelvin to Celsius
}

void updateStackTemperature() {
  /* C20
  Reads the voltage, which is used to approximate the resistance, since the input voltage is
  the controlled 5V output from the arduino. Steinhart_Hart then used to approximate temperature
  Expected Input: Voltage from [0, 5V], or [0, 1024)
  */
  int vTemp = analogRead(StackThermistorPin);
  stackTempArray[stackTempPos] = vTemp;
  stackTempPos = (stackTempPos + 1)%stackTempArraySize;
}

double getAverageIntArray(int[] a) {
  int size = sizeof(a)/sizeof(int);
  double total = 0;
  for (int i = 0; i<size; i++) {
    total += a[i];
  }
  return total / size;
}

double getStackTemperature() {
  double vTemp = getAverageIntArray(stackTempArray);
  double stackResistance = StackTempFixedResistance*(1023.0 / double(Vo) - 1.0);
  return Steinhart_Hart(stackResistance);
}

void Check_Alarms() {
  // Check all alarm properties to make sure they're within the safe range
  // If not, set fc_alarm to true. 
  // Temperature, Pressure, Current, Voltage are checked
  updateStackTemperature();
  if (!secondCounter % 5) {
      double stackTemp = getStackTemperature();
  }
}

void System() {
  // wait for input to set fc_on to true, set CurrentRequest
  if (true) { //
    fc_on = true;
  }
  
  CurrentRequest = 0; // need some logic to know how to set CurrentRequest
}

void FC() {
  AutomaticPurgeControl();
  AutomaticFanControl();
  StackCurrentRampControl();
  
  // FSM
  switch(FC_State) {
     case(FC_INITIAL):
       break;
     case(FC_STANDBY):
       break;
     case(FC_STARTUP):
       break;
     case(FC_RUN):
       break;
     case(FC_SHUTDOWN):
       break;
     case(FC_ALARM):
       // once we enter the FC_ALARM state it cannot be exited without a system reset
       break;
     default:
       fc_alarm = true; //If we somehow enter an invalid state something is wrong so we should transfer to the FC_Alarm
       break;
  }
}

void transition(int fromState, int toState) {
  
 // not sure if we need specific transitions 

 // at the end of the transition we change the state
 FC_State = toState;
 // and then we return to the loop segment
}

float AutomaticFanControl() {
  float OptTemp;
  float UpdatedFanCmd;
  //does the fan control thing 
  
  return [OptTemp, UpdatedFanCmd]; // can't actually do this
}

float StackCurrentRampControl() {
  float CurrentCmd;
  
  return CurrentCmd;
}