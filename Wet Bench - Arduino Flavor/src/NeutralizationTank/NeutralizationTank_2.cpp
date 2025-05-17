#include <Arduino.h>
#include <SoftwareSerial.h>       // Software-based serial communication
#include <Wire.h>                 // I2C communication library
#include <LiquidCrystal_I2C.h>    // LCD library for I2C displays
#include <math.h>                 // Math library for calculations




// ──────────────────────────────────────────── Pin Definition Config ───────────────────────────────────────────────────
// Define pin numbers for various relays and sensors
#define MIXING_RELAY_PIN     2      // Pin for mixing relay
#define VALVE_RELAY_PIN      3      // Pin for valve relay
#define ACID_RELAY_PIN       4      // Pin for acid relay
#define BASE_RELAY_PIN       5      // Pin for base relay
#define PUMP_RELAY_PIN       6      // Pin for pump relay
#define OVERFLOW_SENSOR_PIN  7      // Overflow sensor pin
#define TANK_LOW_SENSOR_PIN  9      // Low-level sensor pin (tank empty)
#define TANK_HIGH_SENSOR_PIN 10     // High-level sensor pin (tank full)
#define PH_SENSOR_PIN        A5     // Analog input for pH sensor
#define FLUSH_BUTTON_PIN     13     // Button input pin for flushing/toggling neutralization
#define PH_SENSOR_RX_PIN     11     // SoftwareSerial RX pin for pH sensor
#define PH_SENSOR_TX_PIN     12     // SoftwareSerial TX pin for pH sensor
// ──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


// ───────────────────────────────────────── Forward Declerations ──────────────────────────────────────────────────────
// Forward declerations: for platform.io, could be in a header

// ──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


// ─────────────────────────────────────── PH Probe Variable Config ──────────────────────────────────────────────────────

// Create a SoftwareSerial object for communication with the Atlas Scientific pH sensor
SoftwareSerial phSensorSerial(PH_SENSOR_RX_PIN, PH_SENSOR_TX_PIN);

// Create a SoftwareSerial object for communication with the Atlas Scientific pH sensor
SoftwareSerial phSensorSerial(PH_SENSOR_RX_PIN, PH_SENSOR_TX_PIN);

// Global variables for system state, sensor readings, and commands
// bool isPHAcceptable = true;            // Flag indicating if the current pH is within acceptable range
// bool tankFlushed = false;              // Flag indicating whether the tank flush process has been completed
String phReadCommand = "R";            // Command to request a pH reading from the sensor
String pcInputBuffer = "";             // Buffer for incoming data from the PC (not used)
String phSensorResponseBuffer = "";    // Buffer for data received from the pH sensor
float currentPH = 7;                   // Default pH value
bool isPHUpdated = false;              // Flag to indicate if the pH value was updated (currently not used)
bool shouldExitLoop = false;           // Flag to exit the main loop (currently not used)
// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


// ───────────────────────────────────────── Display Variable Config ───────────────────────────────────────────────────
// Initialize the LCD at I2C address 0x27 with 20 columns and 4 rows
LiquidCrystal_I2C lcd(0x27, 20, 4);
// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


// ──────────────────────────────────── Neutralization Ctrl Variable Config ────────────────────────────────────────────
// Global variables for neutralization toggle control
bool neutralizationEnabled = true;           // Neutralization is enabled by default
unsigned long neutralizationDisabledTime = 0;  // Timestamp when neutralization was toggled off
// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────



void setup() {
  // Configure pin modes for relays (outputs) and sensors (inputs)
  pinMode(MIXING_RELAY_PIN, OUTPUT);
  pinMode(BASE_RELAY_PIN, OUTPUT);
  pinMode(ACID_RELAY_PIN, OUTPUT);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(VALVE_RELAY_PIN, OUTPUT);
  pinMode(TANK_LOW_SENSOR_PIN, INPUT);
  pinMode(TANK_HIGH_SENSOR_PIN, INPUT);
  pinMode(OVERFLOW_SENSOR_PIN, INPUT);
  pinMode(PH_SENSOR_PIN, INPUT);
  // Use internal pull-up resistor for flush button
  pinMode(FLUSH_BUTTON_PIN, INPUT_PULLUP); // DIFF 1
  
  delay(400);  // Delay for stabilization
  
  // Initialize relay states to LOW (off)
  digitalWrite(MIXING_RELAY_PIN, LOW);
  digitalWrite(BASE_RELAY_PIN, LOW);
  digitalWrite(ACID_RELAY_PIN, LOW);
  digitalWrite(PUMP_RELAY_PIN, LOW); 
  digitalWrite(VALVE_RELAY_PIN, LOW); 
  
  // Initialize serial communications for debugging and sensor communication
  Serial.begin(9600);
  phSensorSerial.begin(9600);                               
  // Reserve memory for the sensor response buffer to optimize dynamic memory usage
  phSensorResponseBuffer.reserve(30);
  
  // Initialize LCD once in setup
  lcd.init();
  lcd.backlight();


  Serial.println("WDT Enabled");
  Serial.println("Begin");
  
  // Send initialization commands to the pH sensor
  phSensorSerial.print("*OK,0 \r");
  phSensorSerial.print("R \r");
}

enum SYSTEM_STATE {IDLE, NEUTRALIZING, FLUSHING}; //System state enums for the entire neutralization system

SYSTEM_STATE systemState = IDLE;
NEUTRALIZING_STATE neutralizingState = MIXING;

void loop(){
    //=- TODO -=//
    // Handle watchdog timer
    // Handle OTA
    // Handle LCD update
    // Handle system disable timeout
    // Handle pH value (regardles of reading in ISR or if system is active) determine system state 

    //Run system
    if(neutralizationEnabled){
        switch(systemState){
            case IDLE:
                //prolly better way of doing this (ensure neutralizing vars are reset for next round)
                mixCount = 0;
                initialMixComplete = false; //ensure initial Mix time is first mix
                neutralizingState = MIXING; //always make sure we start with mixing first

                break;

            case NEUTRALIZING:
                neutralizeSystem();
                break;

            case FLUSHING:
                //prolly better way of doing this (ensure neutralizing vars are reset for next round)
                mixCount = 0;
                initialMixComplete = false;
                neutralizingState = MIXING;
                flushTank();
                break;
        }
    }
}


int initialMix_MAX_CNT_SEC = 25;  //initial mixingtime before chemicals are applied. Represented in seconds
int systemMix_MAX_CNT_SEC = 45;  //mixing time between adding more chemicals assuming PH has not been restored.

int mixCount = 0; //Tracking variable for mixing timing
bool initialMixComplete = false;

struct pH_caseAdjusments{
    double pH_trigger; //point at which range search is started. If pH is >7, range adds. If ph <7 range subtracts
    double range; //range from which the pH case adjustment parameters apply
    double chemDep_Cnt_Sec; //amount of time to dump acid/base for given case can do half second incraments. Note: chem choice dependent on pH_trigger range (ie >7 acid is used)
};

pH_caseAdjusments selectedCaseAdjustment;

const int totalNumCases = 9;
pH_caseAdjusments caseArray[totalNumCases] = {
    //---- Base Case Adjustments (we are adding acid)
    {9.5, 1, 2},           //Base of 9.5 - 10.5, add acid for 2 seconds
    {10.5, 0.5, 3},      //Base of 10.5 - 11, add acid for 3 seconds
    {11, 3, 10},          //Base of 11 -14, add acid for 10 seconds ????

    //---- Acid Case Adjustments (we are adding base)
    {5.5, 0.5, 1},         //Acid of 5.5 - 5, add base for 1 seconds
    {5, 0.5, 1.5},       //Acid of 5 - 4.5, add base for 1.5 seconds
    {4.5, 0.5, 2},   //Acid of 4.5 - 4, add base for 2 seconds
    {4, 0.5, 3},           //Acid of 4 - 3.5, add base for 3 seconds
    {3.5, 0.5, 4},        //Acid of 3.5 - 3, add base for 4 seconds
    {3, 2, 8}        //Acid of 3 - 1, add base for 8 seconds
};
 


void neutralizeSystem(){
    switch(neutralizingState){
        case MIXING:
            //TODO
            //if timer is disabled enable timer
            
            //if initialMixComplete == true, we compare against systemMix, otherweise we compare against initial mix
            //If mix counter is complete, move to chemical deposition, note, we must first return to the main loop
            if(mixCount > (initialMixComplete? systemMix_MAX_CNT_SEC : initialMix_MAX_CNT_SEC)){
                if(!initialMixComplete) {initialMixComplete = true;} //ensure if this was the initial mix to change timing, could just always set to true (latch that bitch)
                mixCount = 0;
                selectedCaseAdjustment = getCaseAdjustment(100); //TODO pass the currentPH
                //TODO disable timer

                neutralizingState = CHEM_DEP;
            }
            break;

        case CHEM_DEP:
            //TODO ensure timer is enabled
            //TODO add back up timeStamp
            int chemcialRelayPin = (selectedCaseAdjustment.pH_trigger > 7)? BASE_RELAY_PIN : ACID_RELAY_PIN; //determine the type of chem we are correcting with (BASE or ACID)
            digitalWrite(chemcialRelayPin, HIGH); //ensure chemical pin (either ACID or BASE) is on and dumping until cnt is over

            if(mixCount > selectedCaseAdjustment.chemDep_Cnt_Sec){
                digitalWrite(chemcialRelayPin, LOW); //make sure the relay gets turned off
                mixCount = 0;
                //TODO disable timer
                neutralizingState = MIXING;
            }
            break;
    }
}

pH_caseAdjusments getCaseAdjustment(double currentPH){
    double upperRange, lowerRange;
    for(int i = 0; i < totalNumCases; i++){
        upperRange = (caseArray[i].pH_trigger > 7) ? (caseArray[i].pH_trigger + caseArray[i].range) : caseArray[i].pH_trigger;
        lowerRange = (caseArray[i].pH_trigger > 7) ? caseArray[i].pH_trigger : (caseArray[i].pH_trigger + caseArray[i].range);

        if(lowerRange >= currentPH && currentPH < upperRange){
            return caseArray[i];
        }
    }
    //TODO throw error i can't find case
}

bool tankReadyForFlush = false;
enum NEUTRALIZING_STATE {MIXING, CHEM_DEP};   //Neutralizing specific state variables (basically sub states), default state should be mixing 
enum FLUSHING_STATE {IDLE_, FILLING_TANK, EMPTYING_TANK};
FLUSHING_STATE flushingState = FILLING_TANK;

void flushTank(){
    //for saftey, ensure acid and base pumps are off
    digitalWrite(ACID_RELAY_PIN, LOW);
    digitalWrite(BASE_RELAY_PIN, LOW);

    switch(flushingState){
        case FILLING_TANK:
            if((!digitalRead(TANK_HIGH_SENSOR_PIN)) && !digitalRead(OVERFLOW_SENSOR_PIN)){
                //Ensure valve is open and pump is off - lets fill the tank 
                digitalWrite(PUMP_RELAY_PIN, LOW); 
                digitalWrite(VALVE_RELAY_PIN, HIGH);
            }
            if((digitalRead(TANK_HIGH_SENSOR_PIN)) || digitalRead(OVERFLOW_SENSOR_PIN)){
                //close valve immediatly and start emptying tank
                digitalWrite(VALVE_RELAY_PIN, LOW);
                flushingState = EMPTYING_TANK;
            }
            break;
        case EMPTYING_TANK:
            break;
    }
    //Make sure the tank is full first, we can skip if overflowing
    if((!digitalRead(TANK_HIGH_SENSOR_PIN)) && !digitalRead(OVERFLOW_SENSOR_PIN)){
        //Ensure valve is open and pump is off
        digitalWrite(PUMP_RELAY_PIN, LOW); //
        digitalWrite(VALVE_RELAY_PIN, HIGH);
    }

    if(digitalRead(TANK_HIGH_SENSOR_PIN) || digitalRead(OVERFLOW_SENSOR_PIN)){
        tankReadyForFlush = true;
        digitalWrite(VALVE_RELAY_PIN, LOW); //ensure valve is closed for flush procedure
    }



     
}

