// FC States
#define FC_INITIAL 0
#define FC_STANDBY 1
#define FC_STARTUP 2
#define FC_RUN 3
#define FC_ALARM 4
#define FC_SHUTDOWN 5

// FC Substates
#define FC_STARTUP_FAN_SPOOLUP 6
#define FC_STARTUP_STARTUP_PURGE 7
#define FC_STARTUP_END 8

// Analog Pins
#define THEMRMISTOR_PIN 0
#define PRESSURE_PIN 1
#define CURRENT_PIN 2
#define VOLTAGE_PIN 3
#define HYDROGEN_PIN 4

// Fan States
#define FAN_OFF 0
#define FAN_ON_LOW 1
#define FAN_ON_MED 2
#define FAN_ON_HIGH 3

// Digital Pins
#define SYSTEM_ON_PIN 12
#define PURGE_PIN 2
#define SUPPLY_PIN 3
#define RESISTOR_PIN 4
#define FC_RELAY_PIN 5

// Valve, Resistor, Relay States (note that this may need to change
#define OPEN 0
#define CLOSED 1

// Other
#define ARRAY_SIZE 300 // will need to be reduced for testing on an uno
#define StackTempFixedResistance 10000
#define FC_MIN_CURRENT 0 // will need to be decided by testing

// flags
boolean fc_on = false;
boolean fc_alarm = false;
boolean fc_fan_time_set = false;
boolean timer_time_set = false;
boolean arrays_filled = false;

// Program constants, all must be determined with testing
long FAN_SPOOLUP_TIME = 5000;
long STARTUP_PURGE_LOOP_COUNT = 5000;
unsigned long Current_Time = 0; // would overflow after 25 days if left running forever (hopefully)
long STANDBY_DELAY_TIME = 5000;
long SHUTDOWN_DELAY_TIME = 5000;

/*
** Counter Declaration
*/
long fan_start_time = 0;
long purge_counter = 1;
long timer_start_time = 0;
long purgeLastCallTime;
long secondCounter;
int arrayIndex = 0;

/* Fake counters*/
int count = 0;
unsigned long current_time = 0;

// not sure of the type for this yet
int CurrentRequest = 0;

int stackTempPos = 0;
int stackTempArray[ARRAY_SIZE];
int stackPressureArray[ARRAY_SIZE];
int stackCurrentArray[ARRAY_SIZE];
int stackVoltageArray[ARRAY_SIZE];

int FC_State = FC_INITIAL; // initial state perhaps we could enumerate these
int FC_SubState = FC_STARTUP_FAN_SPOOLUP;


/*
** Actual Code
**
**
*/

// Currently, one iteration takes 0.5 ms (3/14/17)
void setup() {
  // set all of our variables to their initial values if needed, initiate measuring capabilities as needed
  Serial.begin(115200);
  pinMode(SYSTEM_ON_PIN, INPUT);
  pinMode(PURGE_PIN, OUTPUT);
  pinMode(SUPPLY_PIN, OUTPUT);
  pinMode(RESISTOR_PIN, OUTPUT);
  pinMode(FC_RELAY_PIN, OUTPUT);
  delay(100);
  digitalWrite(PURGE_PIN, CLOSED);
  digitalWrite(SUPPLY_PIN, CLOSED);
  digitalWrite(RESISTOR_PIN, OPEN);
  digitalWrite(FC_RELAY_PIN, OPEN);
  delay(100);
  setupPins();//remove
  flashOn();//remove
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
  // Temperature, Pressure, Current, Voltage, Hydrogen are checked

  // Take measurements
  int fc_temp = analogRead(THEMRMISTOR_PIN);
  int fc_pressure = analogRead(PRESSURE_PIN);
  int fc_current = analogRead(CURRENT_PIN);
  int fc_voltage = analogRead(VOLTAGE_PIN);
  int fc_hydrogen = analogRead(HYDROGEN_PIN);

  if (fc_temp > 500)
    fc_alarm = true;

  // will need to fill arrays here 

  if (arrayIndex < ARRAY_SIZE || !arrays_filled) {
    arrayIndex++;
    return; // wait until we have enough measurements to start triggering alarms
  }
  else {
    arrayIndex = 0;
  }

  updateStackTemperature();


  // if out of threshhold, set fc_alarm to true

}

void System() {
  // wait for input to set fc_on to true, set CurrentRequest
  Serial.println(digitalRead(SYSTEM_ON_PIN));
  if (digitalRead(SYSTEM_ON_PIN) == HIGH) {
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
  switch (FC_State) {
    case (FC_INITIAL):
      digitalWrite(13, HIGH);
      if (count > 1000){
        stateTransition(FC_INITIAL, FC_STANDBY);
        digitalWrite(13, LOW);//remove
        count = 0;
      }
      break;
    case (FC_STANDBY):
      // The stack is not consuming reactant or delivering power
      // and all stack BOP actuators are in their safe state
      // The system remains in FC_STANDBY for STANDBY_DELAY_TIME
      digitalWrite(11, HIGH);//remove

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

    case (FC_STARTUP):
      // The stack goes from FC_STANDBY to a state where current can
      // safely be drawn from the stack
      digitalWrite(10, HIGH);//remove
      FCStartup();
      break;

    case (FC_RUN):
      // ?? manual info is copied from FC_STANDBY
      digitalWrite(9, HIGH);//remove
      if (!fc_on)
        stateTransition(FC_RUN, FC_SHUTDOWN);

      AutomaticPurgeControl();
      AutomaticFanControl();
      StackCurrentRampControl(); // controlled using Current_Request, I would imagine
      break;

    case (FC_SHUTDOWN):
      digitalWrite(8, HIGH);//remove
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

    case (FC_ALARM):
      // The stack is shut down because an alarm was triggered.
      // All actuators are in their safe states

      // seems like we need to do some work to make sure everything is in the right state
      // in the event of an alarm... we could specify the alert via a Serial.println if we really
      // wanted but we could also build a visual representation of the system that failed
      digitalWrite(7, HIGH);//remove
      Serial.println("AN ALARM HAS BEEN TRIGGERED");
      break;

    default:
      fc_alarm = true; //If we somehow enter an invalid state something is wrong so we should transfer to the FC_Alarm
      break;
      
  }

  // eventually can add a delay here depending on what kind of timing resolution we need
  if (count > 1000){
    count = 0;
    Serial.println(FC_State); //remove
    if (FC_State == FC_STARTUP)
      Serial.println(FC_SubState);
    turnAllOff();
   }
   count++;
   
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
    case (FC_STARTUP_FAN_SPOOLUP):
      // fan is turned on and given time to spool up
      // TODO: how are we going to implement the fan?
      if (!fc_fan_time_set) {
        fan_start_time = Current_Time;
        fc_fan_time_set = true;
      }
      
      if (FAN_SPOOLUP_TIME <= Current_Time - fan_start_time) {
        fc_fan_time_set = false;
        subStateTransition(FC_SubState, FC_STARTUP_STARTUP_PURGE);
        return;
      }
      break;

    case (FC_STARTUP_STARTUP_PURGE):
      // purge valve and supply valves are opened simultaneously
      // for the start-up purge and the start-up resistor is applied
      // across the stack to limit voltage
      
      if (!fc_fan_time_set) {
        purge_counter = Current_Time;
        fc_fan_time_set = true;
      }

      if (STARTUP_PURGE_LOOP_COUNT <= Current_Time - purge_counter) {
        fc_fan_time_set = false;
        subStateTransition(FC_SubState, FC_STARTUP_END);
        return;
      }
      break;

    case (FC_STARTUP_END):
      // close purge valve
      // fan is minimum
      fc_fan_time_set = false;
      purge_counter = 1;
      stateTransition(FC_State, FC_RUN);
      subStateTransition(FC_SubState, FC_STARTUP_FAN_SPOOLUP); // switch back to startup purge
      break;
  }
}

double Steinhart_Hart(int resistance, double a, double b, double c) {
  // Returns the temperature of a thermistor, based on the model's coefficients
  // These double values can be hard coded, or left as inputs for other temp readings
  // In arduino, log() is for the natural log implementation.
  double logR = log(resistance);
  double model = a + b * logR + c * (logR * logR * logR);
  return (1 / model) - 273.15; // Account for Kelvin to Celsius
}

double getStackTemperature() {

  double vTemp = getAverageIntArray(stackTempArray);
  double stackResistance = StackTempFixedResistance * (1023.0 / double(vTemp) - 1.0);
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

/*
** This will not be implemented until we better understand how the motor controller works
*/

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
  int vTemp = analogRead(THEMRMISTOR_PIN);
  stackTempArray[stackTempPos] = vTemp;
  stackTempPos = int((stackTempPos + 1) % ARRAY_SIZE);
}

double getAverageIntArray(int* a) {
  int sizeOfArray = sizeof(a) / sizeof(int);
  double total = 0;
  for (int i = 0; i < sizeOfArray; i++) {
    total += a[i];
  }
  return total / sizeOfArray;
}

int average(int* arr) {
  long sum = 0;
  for (int i; i < ARRAY_SIZE; i++)
    sum += arr[i];

  return sum / ARRAY_SIZE;
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
void setFanState(int state) {
  // will need to use a switch-case 
}

void setRelayState(int state) {
  digitalWrite(FC_RELAY_PIN, state);
}

void setResistorState(int state) {
  digitalWrite(RESISTOR_PIN, state);
}


/*
 PHYSICAL OUTPUT TEST REMOVE ALL
*/
void setupPins() {
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT); 
  delay(100);
  digitalWrite(13, LOW);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);
  digitalWrite(7, LOW); 
}

void turnAllOff() {
  digitalWrite(13, LOW);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, LOW); 
  digitalWrite(7, LOW); 
}

void flashOn() {
  digitalWrite(13, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(8, HIGH); 
  digitalWrite(7, HIGH); 
  delay(1000);
  turnAllOff();
}

