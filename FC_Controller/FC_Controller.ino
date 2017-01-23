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

//Other
#define STACK_TEMP_ARRAY_SIZE 100

// flags
boolean fc_on = false;
boolean fc_alarm = false;
boolean fc_fan_time_set = false;
boolean timer_time_set = false;


// need to be decided
long FAN_SPOOLUP_TIME = 40000;
long STARTUP_PURGE_LOOP_COUNT = 40000;
long Current_Time = 676767667;
long STANDBY_DELAY_TIME = 12387979;
long SHUTDOWN_DELAY_TIME = 123213;

/*
** Counter Declaration
*/
long fan_start_time = 0;
long purge_counter = 1;
long timer_start_time = 0;
long purgeLastCallTime;
long secondCounter;

// not sure of the type for this yet
int CurrentRequest = 0;

int stackTempPos = 0;
int stackTempArray[STACK_TEMP_ARRAY_SIZE];

int FC_State = FC_INITIAL; // initial state perhaps we could enumerate these
int FC_SubState = FC_STARTUP_FAN_SPOOLUP;

/* 
** What are these for?
*/
double Vo = 1;


/* 
** Actual Code
**
**
*/

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


/*
** Main Functions
**
**
*/

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
  
  if (true) { //logic
    fc_on = true;
  }
  
  CurrentRequest = 0; // need some logic to know how to set CurrentRequest
}

/* FC()
** Controls the fuel cell.
*/
void FC() {    
  if (fc_alarm)
    stateTransition(FC_State, FC_ALARM);

  // FSM
  switch(FC_State) {
    case(FC_INITIAL):
      stateTransition(FC_INITIAL, FC_STANDBY);
      break;

    case(FC_STANDBY):
      // The stack is not consuming reactant or delivering power
      // and all stack BOP actuators are in their safe state
      // The system remains in FC_STANDBY for STANDBY_DELAY_TIME
      if (!timer_time_set) {
        timer_start_time = Current_Time;
        timer_time_set = true;
      }
      
      // probably need to set the default state of all the things in the fuel cell
      
      if (STANDBY_DELAY_TIME <= Current_Time - timer_start_time && fc_on) {
        timer_time_set = false;
        stateTransition(FC_STANDBY, FC_STARTUP);
      }

      break;

    case(FC_STARTUP):
      // The stack goes from FC_STANDBY to a state where current can
      // safely be drawn from the stack
      
      FCStartup();
      break;

    case(FC_RUN):
      // ?? manual info is copied from FC_STANDBY
      if (!fc_on)
        stateTransition(FC_RUN, FC_SHUTDOWN);
      
      AutomaticPurgeControl();
      AutomaticFanControl();
      StackCurrentRampControl(); // controlled using Current_Request, I would imagine
      break;

    case(FC_SHUTDOWN):
      // The stack goes from FC_RUN to FC_STANDBY. The system remains in
      // FC_SHUTDOWN for SHUTDOWN_DELAY_TIME
      if (!timer_time_set) {
        timer_start_time = Current_Time;
        timer_time_set = true;
      }
      
      // do something to shut down the stack?
      
      if (SHUTDOWN_DELAY_TIME <= Current_Time - timer_start_time) {
        timer_time_set = false;
        stateTransition(FC_SHUTDOWN, FC_STANDBY);
      }
      break;

    case(FC_ALARM):
      // The stack is shut down because an alarm was triggered.
      // All actuators are in their safe states
      
      // seems like we need to do some work to make sure everything is in the right state 
      // in the event of an alarm... we could specify the alert via a Serial.println if we really
      // wanted but we could also build a visual representation of the system that failed
      Serial.println("AN ALARM HAS BEEN TRIGGERED");
      break;

    default:
      fc_alarm = true; //If we somehow enter an invalid state something is wrong so we should transfer to the FC_Alarm
      break;
  }
  
  // eventually can add a delay here depending on what kind of timing resolution we need
}

/*
** Helper Functions
**
**
*/

void stateTransition(int fromState, int toState) {
  FC_State = toState;
}

void subStateTransition(int fromState, int toState) {
  FC_SubState = toState;
}

double Steinhart_Hart(int resistance, double a, double b, double c) {
  // Returns the temperature of a thermistor, based on the model's coefficients
  // These double values can be hard coded, or left as inputs for other temp readings
  // In arduino, log() is for the natural log implementation.
  double logR = log(resistance);
  double model = a + b*logR + c*(logR * logR * logR);
  return (1/model) - 273.15; // Account for Kelvin to Celsius
}

double getStackTemperature() {
  double vTemp = getAverageIntArray(stackTempArray);
  double stackResistance = StackTempFixedResistance*(1023.0 / double(Vo) - 1.0);
  return Steinhart_Hart(stackResistance, 1, 1, 1);
}

float AutomaticFanControl() {
  float OptTemp;
  float UpdatedFanCmd;
  //does the fan control thing 
  
  return OptTemp;//, UpdatedFanCmd]; // can't actually do this
}

boolean AutomaticPurgeControl() {
  boolean OpenPurgeValve = false;

  return OpenPurgeValve;
}

float StackCurrentRampControl() {
  float CurrentCmd;
  
  return CurrentCmd;
}

void updateStackTemperature() {
  /* C20
  Reads the voltage, which is used to approximate the resistance, since the input voltage is
  the controlled 5V output from the arduino. Steinhart_Hart then used to approximate temperature
  Expected Input: Voltage from [0, 5V], or [0, 1024)
  */
  int vTemp = analogRead(StackThermistorPin);
  stackTempArray[stackTempPos] = vTemp;
  stackTempPos = int((stackTempPos + 1) % STACK_TEMP_ARRAY_SIZE);
}

double getAverageIntArray(int* a) {
  int sizeOfArray = sizeof(a)/sizeof(int);
  double total = 0;
  for (int i = 0; i<sizeOfArray; i++) {
    total += a[i];
  }
  return total / sizeOfArray;
}

void FCStartup() {
  switch(FC_SubState) {
    case(FC_STARTUP_FAN_SPOOLUP):
      // fan is turned on and given time to spool up
      // TODO: how are we going to implement the fan?
      if (!fc_fan_time_set) {
        fan_start_time = millis();
        fc_fan_time_set = true; 
      }
      if (FAN_SPOOLUP_TIME <= Current_Time - fan_start_time) {
        subStateTransition(FC_SubState, FC_STARTUP_STARTUP_PURGE);
        return;
      }
      break;

    case(FC_STARTUP_STARTUP_PURGE):
      // purge valve and supply valves are opened simultaneously
      // for the start-up purge and the start-up resistor is applied
      // across the stack to limit voltage
      
      if (STARTUP_PURGE_LOOP_COUNT <= purge_counter){
        subStateTransition(FC_SubState, FC_STARTUP_STARTUP_PURGE);
        return;
      }
      
      purge_counter++;
      break;

    case(FC_STARTUP_END):
      // close purge valve
      // fan is minimum
      fc_fan_time_set = false;
      purge_counter = 1;
      stateTransition(FC_State, FC_RUN);
      subStateTransition(FC_SubState, FC_STARTUP_FAN_SPOOLUP); // switch back to startup purge 
      break;
  }
}
