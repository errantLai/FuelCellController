// FC States
#define FC_INITIAL 0
#define FC_STANDBY 1
#define FC_STARTUP 2
#define FC_RUN 3
#define FC_ALARM 4
#define FC_SHUTDOWN 5

// FC Startup Substates
#define FC_STARTUP_STARTUP_PURGE 6
#define FC_STARTUP_END 7

// Analog Pins
#define THEMRMISTOR_PIN 0
#define CURRENT_PIN 1
#define VOLTAGE_PIN_ONE 2
#define VOLTAGE_PIN_TWO 3
#define HYDROGEN_PIN 4

// Digital Pins
#define SYSTEM_ON_PIN 2
#define PURGE_PIN 3
#define SUPPLY_PIN 4
#define FC_RELAY_PIN 5
#define FC_FAN_RELAY_PIN 14

//Digital State LED Pins
#define STACK_ON_LED 13
#define STANDBY_LED 12
#define STARTUP_LED 11
#define STARTUP_PURGE_LED 10
#define STARTUP_END_LED 9
#define RUN_LED 8
#define SHUTDOWN_LED 7
#define ALARM_LED 6

// Valve, Relay States (note that this may need to change)
#define OPEN 0
#define CLOSED 1

// Other
#define ARRAY_SIZE 300 // will need to be reduced for testing on an uno
#define StackTempFixedResistance 10000

// Alarm Thresholds TODO: set threshes
#define FC_MIN_CURRENT 0 // A
#define FC_MAX_CURRENT 30 // A
#define FC_STANDBY_MIN_VOLTAGE 0 // V
#define FC_RUN_MIN_VOLTAGE 13 // V
#define FC_MAX_VOLTAGE 55 // V
#define FC_MAX_H2_READ 10 // ppm
#define FC_MIN_TEMP 15 // deg C
#define FC_MAX_TEMP 70 // deg C

// Constant System Parameters
#define PURGE_THRESHOLD 1026.0 //A*s
#define LOOP_TIME 0.002

// Delays (number of loops @ 2 ms per loop)
#define STARTUP_PURGE_LOOP_COUNT 500 // 1 s
#define GENERAL_PURGE_TIME 100 // 0.2 s
#define STANDBY_DELAY_TIME 1000 // 2 s
#define SHUTDOWN_DELAY_TIME 1500 // 3 s
#define START_DELAY_TIME 5000 // 10 s

// flags
boolean fc_on = false;
boolean fc_alarm = false;
boolean timer_time_set = false;
boolean arrays_filled = false;
boolean fc_purge_time_set = false;

// global timer 
unsigned long Current_Time = 0; // would overflow after 25 days if left running forever (hopefully)

//purge amp-sec counter
float AmpSecSincePurge = 0;
int purge_time = 0;

/*
** Counter Declaration
*/
long purge_counter = 1;
long timer_start_time = 0;
long purgeLastCallTime;
long fc_on_counter = 0;
long fc_start_counter = 0;
long fc_off_counter = 0;
int arrayIndex = 0;


// input value arrays
int stackTempArray[ARRAY_SIZE];
int stackCurrentArray[ARRAY_SIZE];
int stackVoltageArray[ARRAY_SIZE];
int stackH2Array[ARRAY_SIZE];

// Set up initial states
int FC_State = FC_INITIAL;
int FC_SubState = FC_STARTUP_STARTUP_PURGE;

/*
** Actual Code
**
**
*/

// Currently, one iteration takes 2 ms (4/29/17)
void setup() {
  // set all of our variables to their initial values if needed, initiate measuring capabilities as needed
  Serial.begin(115200);
  pinMode(SYSTEM_ON_PIN, INPUT);
  pinMode(PURGE_PIN, OUTPUT);
  pinMode(SUPPLY_PIN, OUTPUT);
  pinMode(FC_RELAY_PIN, OUTPUT);
  pinMode(FC_FAN_RELAY_PIN, OUTPUT);
  delay(100);

  // set valves and relay to default values
  setAllActuatorsSafe();
  delay(100);

  // setup LED pins and flash pins to indicate setup complete 
  setupPins();
  flashOn();
}

void loop() {
  // we iterate through this switch case block infinitely
  Check_Alarms();
  System();
  FC();
  delay(1);
}


/*
** Main Functions
**
**
*/

void Check_Alarms() {
  // Check all alarm properties to make sure they're within the safe range
  // If not, set fc_alarm to true.
  // Temperature, Pressure, Current, Voltage, Hydrogen are checked
  arrayIndex++;
  
  // Take measurements TODO: FIX
  stackTempArray[arrayIndex] = 20;analogRead(A0);
  stackCurrentArray[arrayIndex] = 20;analogRead(A1);
  stackVoltageArray[arrayIndex] = 20;abs(analogRead(A2) - analogRead(A3));
  stackH2Array[arrayIndex] = analogRead(A4);

  if (arrayIndex < ARRAY_SIZE && !arrays_filled) {
    return; // wait until we have enough measurements to start triggering alarms
  }
  else {
    arrays_filled = true;
  }
  
  if (arrayIndex >= ARRAY_SIZE)
    arrayIndex = 0;
  
  // check all thresholds to ensure maxes are not exceeded 
  checkThreshold(average(stackH2Array), 0, FC_MAX_H2_READ, "HYDROGEN");
  checkThreshold(Steinhart_Hart(average(stackTempArray)), FC_MIN_TEMP, FC_MAX_TEMP, "TEMPERATURE");
  checkThreshold(convertToAmperage(average(stackCurrentArray)), FC_MIN_CURRENT, FC_MAX_CURRENT, "CURRENT");
  
  if (FC_State == FC_RUN) //TODO: timing may need to be delayed or tweaked to ensure FC voltage is high enough before testing
    checkThreshold(convertToVoltage(average(stackVoltageArray)), FC_RUN_MIN_VOLTAGE, FC_MAX_VOLTAGE, "VOLTAGE");
  else
    checkThreshold(convertToVoltage(average(stackVoltageArray)), FC_STANDBY_MIN_VOLTAGE, FC_MAX_VOLTAGE, "VOLTAGE");
}

/*
** Turns the fc on if the on button is help for START_DELAY_TIME
** 
*/
void System() {
  if (!fc_on || !fc_alarm) {
    if (digitalRead(SYSTEM_ON_PIN) == HIGH) {
      fc_on_counter++;
      if (fc_on_counter >= START_DELAY_TIME) {
    	digitalWrite(STACK_ON_LED, HIGH);
    	fc_on = true;
        fc_on_counter = 0;
      }
    }
    else
      fc_on_counter = 0;
  }
}

/* FC()
** Controls the fuel cell.
*/
void FC() {
  if (fc_alarm)
    stateTransition(FC_State, FC_ALARM);

  // FSM
  switch (FC_State) {
    case (FC_INITIAL):
      setAllActuatorsSafe();
      
      fc_start_counter++;
      Serial.println(fc_start_counter);
      if (fc_start_counter > 2000) {
        stateTransition(FC_INITIAL, FC_STANDBY);
        fc_start_counter = 0;
      }
      break;
    case (FC_STANDBY):
      // The stack is not consuming reactant or delivering power
      // and all stack BOP actuators are in their safe state
      // The system remains in FC_STANDBY for STANDBY_DELAY_TIME
      digitalWrite(STANDBY_LED, HIGH);
      setAllActuatorsSafe();

      if (!timer_time_set) {
        timer_start_time = Current_Time;
        timer_time_set = true;
      }

      // probably need to set the default state of all the things in the fuel cell
      
      if (STANDBY_DELAY_TIME <= Current_Time - timer_start_time && fc_on) {
        timer_time_set = false;
        digitalWrite(STANDBY_LED, LOW);
        stateTransition(FC_STANDBY, FC_STARTUP);
      }

      break;

    case (FC_STARTUP):
      // The stack goes from FC_STANDBY to a state where current can
      // safely be drawn from the stack
      digitalWrite(STARTUP_LED, HIGH);
      FCStartup();
      break;

    case (FC_RUN):
      // ?? manual info is copied from FC_STANDBY
      digitalWrite(RUN_LED, HIGH);
      runStateActuatorStates();
      
      // On/off button by driver
      if (digitalRead(SYSTEM_ON_PIN) == HIGH) {
        fc_off_counter++;
        if (fc_off_counter >= START_DELAY_TIME) {
          digitalWrite(RUN_LED, LOW);
    	  digitalWrite(STACK_ON_LED, LOW);
          fc_off_counter = 0;
          stateTransition(FC_RUN, FC_SHUTDOWN);
    	  fc_on = false;
        }
      }
      else
        fc_off_counter = 0;

      AutomaticPurgeControl();
      break;

    case (FC_SHUTDOWN):
      digitalWrite(SHUTDOWN_LED, HIGH);
      shutdownActuatorStates();
      
      // The stack goes from FC_RUN to FC_STANDBY. The system remains in
      // FC_SHUTDOWN for SHUTDOWN_DELAY_TIME
      if (!timer_time_set) {
        timer_start_time = Current_Time;
        timer_time_set = true;
      }

      // do something to shut down the stack?

      if (SHUTDOWN_DELAY_TIME <= Current_Time - timer_start_time) {
        timer_time_set = false;
        digitalWrite(SHUTDOWN_LED, LOW);
        stateTransition(FC_SHUTDOWN, FC_STANDBY);
      }
      
      break;

    case (FC_ALARM):
      // The stack is shut down because an alarm was triggered.
      // All actuators are in their safe states

      // seems like we need to do some work to make sure everything is in the right state
      // in the event of an alarm... we could specify the alert via a Serial.println if we really
      // wanted but we could also build a visual representation of the system that failed
      digitalWrite(ALARM_LED, HIGH);
      digitalWrite(STACK_ON_LED, LOW);

      // put valves and relay in safe states
      setAllActuatorsSafe();

      // we should really have a sound/visual indicator
      break;

    default:
      fc_alarm = true; //If we somehow enter an invalid state something is wrong so we should transfer to the FC_Alarm
      break;
      
  }
   
   Current_Time++;
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

/*
** FC_STARTUP state machine within a state machine
*/

void FCStartup() {
  switch (FC_SubState) {
    case (FC_STARTUP_STARTUP_PURGE):
      // purge valve and supply valves are opened simultaneously
      // for the start-up purge across the stack to limit voltage
      digitalWrite(STARTUP_PURGE_LED, HIGH);
      startupPurgeActuatorStates();
      if (!fc_purge_time_set) {
        purge_counter = Current_Time;
        fc_purge_time_set = true;
      }

      if (STARTUP_PURGE_LOOP_COUNT <= Current_Time - purge_counter) {
        fc_purge_time_set = false;
        digitalWrite(STARTUP_PURGE_LED, LOW);
        subStateTransition(FC_SubState, FC_STARTUP_END);
        return;
      }
      break;

    case (FC_STARTUP_END):
      // close purge valve
      startupEndActuatorStates();
      digitalWrite(STARTUP_END_LED, HIGH);
      delay(1000);
      digitalWrite(STARTUP_END_LED, LOW);
      purge_counter = 0;
      digitalWrite(STARTUP_LED, LOW);
      stateTransition(FC_State, FC_RUN);
      subStateTransition(FC_SubState, FC_STARTUP_STARTUP_PURGE); // switch back to startup purge
      break;
  }
}

/*
** Conversion Functions TODO: ALL NEED CALIBRATION
*/
int convertToAmperage(int val) {
  return val;
}

int convertToVoltage(int val) {
  return val;
}

int Steinhart_Hart(int val) {
  // Returns the temperature of a thermistor, based on the model's coefficients
  // These double values can be hard coded, or left as inputs for other temp readings
  // In arduino, log() is for the natural log implementation.
  
  double a = 0; //TODO: calibrate
  double b = 0;
  double model = a + b * val;
  return val;
  //return int((1 / model) - 273.15); // Account for Kelvin to Celsius
}

double getStackTemperature() {
  double vTemp = average(stackTempArray);
  double stackResistance = StackTempFixedResistance * (1023.0 / double(vTemp) - 1.0);
  return Steinhart_Hart(stackResistance);
}

/*
** Automatic Purge Control
*/
void AutomaticPurgeControl() {
  if (AmpSecSincePurge < PURGE_THRESHOLD) {
    AmpSecSincePurge = AmpSecSincePurge + LOOP_TIME*stackCurrentArray[arrayIndex];
    return;
  }
  
  if (purge_time < GENERAL_PURGE_TIME)
    purge_time++;
  else {
    AmpSecSincePurge = 0.0;
    purge_time = 0;
  }
}

/*
** This will not be implemented until we better understand how the motor controller works
*/

int average(int* arr) {
  long sum = 0;
  for (int i = 0; i < ARRAY_SIZE; i++)
    sum += arr[i];

  return sum / ARRAY_SIZE;
}

void checkThreshold(int input, int lowerLimit, int upperLimit, String inputType) {
  if (input < lowerLimit)
    Serial.println("AN ALARM HAS BEEN TRIGGERED DUE TO: LOW " + inputType);
  else if (input > upperLimit)
    Serial.println("AN ALARM HAS BEEN TRIGGERED DUE TO: HIGH " + inputType);
  else
    return;
    
  // if we get here there's an alarm
  fc_alarm = true;
}

/*
** Fuel Cell Controller Control Functions
*/

void setPurgeState(int state) {
  digitalWrite(PURGE_PIN, state);
}
void setSupplyState(int state) {
  digitalWrite(SUPPLY_PIN, state);
}

void setMotorRelayState(int state) {
  digitalWrite(FC_RELAY_PIN, state);
}

void setFanRelayState(int state) {
  digitalWrite(FC_RELAY_PIN, state);
}

/*
** Shortcut Relay/Valve States
*/

void setAllActuatorsSafe() { // for startup and initial states
  setPurgeState(LOW); // closed
  setSupplyState(LOW); // closed
  setMotorRelayState(LOW); // not connected
  setFanRelayState(LOW); // not connected
}

void startupPurgeActuatorStates() { // for startup-purge substate
  setPurgeState(HIGH); // open
  setSupplyState(HIGH); // open
  setMotorRelayState(LOW); // not connected
  setFanRelayState(LOW); // not connected
}

void startupEndActuatorStates() { // for startup-end substate
  setPurgeState(HIGH); // closed
  setSupplyState(HIGH); // open
  setMotorRelayState(LOW); // not connected
  setFanRelayState(HIGH); // connected
}

void runStateActuatorStates() { // for FC_RUN state
  // set purge in automatic purge control
  setSupplyState(HIGH);
  setMotorRelayState(HIGH);
  setFanRelayState(HIGH);
}

void shutdownActuatorStates() { // for FC_SHUTDOWN
  setPurgeState(HIGH); // open
  setSupplyState(LOW); // closed
  setMotorRelayState(LOW); // not connected
  setFanRelayState(LOW); // not connected
}

/*
 STATE OUTPUT TEST
*/
void setupPins() {
  pinMode(STACK_ON_LED, OUTPUT);
  pinMode(STANDBY_LED, OUTPUT);
  pinMode(STARTUP_LED, OUTPUT);
  pinMode(STARTUP_PURGE_LED, OUTPUT);
  pinMode(STARTUP_END_LED, OUTPUT);
  pinMode(RUN_LED, OUTPUT);
  pinMode(SHUTDOWN_LED, OUTPUT);
  pinMode(ALARM_LED, OUTPUT);
  
  // wait a bit so everything gets set up internally
  delay(100);
  digitalWrite(STACK_ON_LED, LOW); // does not get turned off by turnAllOff()
  turnAllOff();
}

void turnAllOff() {
  digitalWrite(STANDBY_LED, LOW);
  digitalWrite(STARTUP_LED, LOW);
  digitalWrite(STARTUP_PURGE_LED, LOW);
  digitalWrite(STARTUP_END_LED, LOW);
  digitalWrite(RUN_LED, LOW);
  digitalWrite(SHUTDOWN_LED, LOW);
  digitalWrite(ALARM_LED, LOW); 
}

void flashOn() {
  digitalWrite(STACK_ON_LED, HIGH);
  digitalWrite(STANDBY_LED, HIGH);
  digitalWrite(STARTUP_LED, HIGH);
  digitalWrite(STARTUP_PURGE_LED, HIGH);
  digitalWrite(STARTUP_END_LED, HIGH);
  digitalWrite(RUN_LED, HIGH);
  digitalWrite(SHUTDOWN_LED, HIGH);
  digitalWrite(ALARM_LED, HIGH); 
  delay(1000);
  digitalWrite(STACK_ON_LED, LOW); // does not get turned off by turnAllOff()
  turnAllOff();
}

