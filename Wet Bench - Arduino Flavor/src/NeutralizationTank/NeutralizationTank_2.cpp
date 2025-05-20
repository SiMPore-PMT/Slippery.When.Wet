//#include "NeutralizationTank.h"
#include <Arduino.h>
#include <Wire.h>                 // I2C communication library
#include <LiquidCrystal_I2C.h>    // LCD library for I2C displays
#include <math.h>                 // Math library for calculations
#include "FspTimer.h"             // FSP library for uno R4
#include <movingAvg.h>            // Moving average filter for PH probve
#include <WDT.h>                  // Watchdog timer
// #include <ArduinoOTA.h>           // Over the air updates
// #include <WiFiS3.h> 
// #include <WiFiUdp.h>
// #include <PubSubClient.h>         // Pubsub client for connection/communication with mqtt

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
#define PH_SENSOR_RX_PIN     0 //11     // HardwareSerial RX pin for pH sensor 
#define PH_SENSOR_TX_PIN     1 //12    // HardwareSerial TX pin for pH sensor
// ──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


// ────────────────────────────────────────── Forward Declerations ──────────────────────────────────────────────────────
void    flushButton_ISR();
void    overflowSensor_ISR();
void    highTankSensor_ISR();
void    timer_100ms_callback(timer_callback_args_t *args);

bool    neutralizeSystem(double currentPH);
bool    flushTank();
void    updateDisplay(double pH);

double  update_pH();
bool    safe_pH_range(double pH);
struct pH_caseAdjusments getCaseAdjustment(double currentPH);
void    ensureNeutralSystemState();
void    ensureNeutralFlushingState();


// ──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


// ───────────────────────────────────────── Network Variable Config ─────────────────────────────────────────────────────
// const char* ssid = "SiMPore Co";                                                                    //Wifi network SSID
// const char* password = "%$1mp0r3*";                                                                 //Wifi password

// const char* mqtt_server = "simporeco.local";//"192.168.117.185";                                    //MQTT servers IP address
// const int mqtt_port = 1883;                                                                         // MQTT port
// const char* mqtt_user = "homeassistant";    // change to wet_bench i think this is the name                                  
// const char* mqtt_password = "uushiyeugaelei9ahshogeuS6aishahSai3AhtoDee5Siep8ohSha2ooch7vieje";

// const char* mqttBroadCastName = "wet_bench";
// WiFiClient espClient;
// PubSubClient client(espClient);
// ──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


// ─────────────────────────────────────── PH Probe Variable Config ──────────────────────────────────────────────────────

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
enum DisplayState { NORMAL_DISPLAY, POPUP_DISPLAY };
DisplayState displayState = NORMAL_DISPLAY;

unsigned long popupDuration = 4500;  // ms
unsigned long popupStartTime = 0;
String popupMessage = "";
String subMessage = "";
int subMessageShift = 0;

bool enableSubMessage = false;
// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


// ──────────────────────────────────────── Pushbutton ISR Variable Config ──────────────────────────────────────────────
constexpr uint32_t DEBOUNCE_DELAY   = 50;            // ms debounce window
constexpr uint32_t LONG_PRESS_TIME  = 3000;         // ms threshold for long press

// Internal ISR timing
volatile uint32_t lastIntTime = 0;                  // timestamp of last valid edge
volatile uint32_t pressTime   = 0;                  // timestamp when button went LOW (pressed)

volatile bool toggleNeutralizationSystem = false;   //So we can do more than just toggle things (remove workload from isr)
volatile bool attemptNeutralizationSystemFlush = false;
// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


// ──────────────────────────────────────── PH adjustment Variable Config ──────────────────────────────────────────────
enum ph_ReadSensorState { IDLE_s, WAITING_s };
ph_ReadSensorState phState = IDLE_s;
double   lastPH      = 0.0;
String  buf         = "";
unsigned long lastReqTime = 0, startTime = 0;

struct pH_caseAdjusments{
    double pH_trigger;      //point at which range search is started. If pH is >7, range adds. If ph <7 range subtracts
    double range;           //range from which the pH case adjustment parameters apply
    double chemDep_Cnt_Sec; //amount of time to dump acid/base for given case can do .1s (100ms) incraments. Note: chem choice dependent on pH_trigger range (ie >7 acid is used)
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
// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


//  ──────────────────────────────────────── Full System Variable Config ───────────────────────────────────────────────
enum SYSTEM_STATE {IDLE, NEUTRALIZING, FLUSHING};           //System state enums for the entire neutralization system
SYSTEM_STATE systemState = IDLE;

bool neutralizationEnabled = true;                          // Neutralization is enabled by default
volatile unsigned long neutralizationDisabledTime = 0;      // Timestamp when neutralization was toggled off

movingAvg phFilter(11);                                     //Create a 11 sample moving average filter
// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


//  ────────────────────────────────────── Flushing System Variable Config ─────────────────────────────────────────────
enum FLUSHING_STATE {FILLING_TANK, EMPTYING_TANK};          //Flushing specific state variables (more sub states), defualt state should be filling
FLUSHING_STATE flushingState = FILLING_TANK;
// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


//  ────────────────────────────────── Neutralization System Variable Config ───────────────────────────────────────────
enum NEUTRALIZING_STATE {MIXING, CHEM_DEP};                 // Neutralizing specific state variables (basically sub states), default state should be mixing 
NEUTRALIZING_STATE neutralizingState = MIXING;

double initialMix_MAX_CNT_SEC = 25.0;                       // initial mixingtime before chemicals are applied. Represented in seconds
double systemMix_MAX_CNT_SEC = 45.0;                        // mixing time between adding more chemicals assuming PH has not been restored.

volatile double mixCount = 0;                               // Tracking variable for mixing timing
bool initialMixComplete = false;

FspTimer timer_100ms;                                       // ISR timer for ensure accurate counts
// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────






/**
 * Handles flush button presses.
 * - A short press (< 3 seconds) toggles neutralization mode on/off.
 *   When toggled from paused to active the first press displays a "resumed" message.
 * - A long press (>= 3 seconds) triggers an immediate tank flush.
 *   However, if neutralization is paused, a flush attempt will display a warning.
 */
void flushButton_ISR() {
  uint32_t now = millis();
  // simple debounce
  if (now - lastIntTime < DEBOUNCE_DELAY) return;
  lastIntTime = now;

  int state = digitalRead(FLUSH_BUTTON_PIN);
  if (state == LOW) {  // button pressed (INPUT_PULLUP → LOW)
    pressTime = now;
  } else {
    // button released
    uint32_t duration = now - pressTime;
    if (duration >= LONG_PRESS_TIME) {
      //LONG PRESS
      attemptNeutralizationSystemFlush = true;
    } else {
      //SHORT PRESS
      toggleNeutralizationSystem = true;
    }
  }
}


void overflowSensor_ISR(){
    //If overflow sensor is triggered, we gotta move to flushing state regardless of PH
    //Also just double check we arent currently flushhing
    if(systemState != FLUSHING) {
        systemState = FLUSHING;

        //Technically this should not be in the ISR, though nothing else is more important,
        //So we can stick around for half a second and not worry about anything
        displayState = POPUP_DISPLAY;
        popupDuration = 5000; 
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("OVERFLOW");
        lcd.setCursor(0, 1);
        lcd.print("ERROR");
    }
}

void highTankSensor_ISR(){
    //At a full tank and we need to flush, must be neutralized though first
    if((systemState != FLUSHING) && safe_pH_range(phFilter.getAvg())){
        systemState = FLUSHING;
    }
}

void timer_100ms_callback(timer_callback_args_t *args){
    mixCount += 0.1; //incramnt counter
}

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
  
  delay(200);  // Delay for stabilization
  
  // Initialize relay states to LOW (off)
  digitalWrite(MIXING_RELAY_PIN, LOW);
  digitalWrite(BASE_RELAY_PIN, LOW);
  digitalWrite(ACID_RELAY_PIN, LOW);
  digitalWrite(PUMP_RELAY_PIN, LOW); 
  digitalWrite(VALVE_RELAY_PIN, LOW); 
  
  // Initialize serial communications for debugging and sensor communication
  Serial.begin(115200);   // For direct terminal coms (ie serial.println)
  Serial1.begin(115200);  // For PH probe print outs - changed this to higher baud rate, may not be acceptable with pH probe

  //Setup ISRs
  attachInterrupt(OVERFLOW_SENSOR_PIN, overflowSensor_ISR, RISING); //Overflow sensor ISR   
  attachInterrupt(TANK_HIGH_SENSOR_PIN, highTankSensor_ISR, RISING); //High tank sesnsor ISR
  attachInterrupt(digitalPinToInterrupt(FLUSH_BUTTON_PIN),
                  flushButton_ISR,
                  RISING);

  //Setup timer
  timer_100ms.begin(
    TIMER_MODE_PERIODIC,        // mode: perodic interrupt
    GPT_TIMER,                  // type: general PWM timer
    0,                          // channel: 0
    10.0,                       // frequency(in HZ): 10hz ==> 100ms 
    50.0,                       // duty % (not used for perodic)
    timer_100ms_callback        // Callback function
  );

  timer_100ms.setup_overflow_irq();
  timer_100ms.start(); // ensure timer isn't running
  timer_100ms.disable_overflow_irq();

  // Initialize LCD once in setup
  lcd.init();
  lcd.backlight();


  Serial.println("WDT Enabled");
  Serial.println("Begin");
  
  // Send initialization commands to the pH sensor
  Serial1.print("*OK,0 \r");
  Serial1.print("R \r");

  //Setup watchdog timer (5s timeout):
  if (!WDT.begin(5000)) {
    Serial.println("Error: Watchdog failed to start");
  } else {
    Serial.println("Watchdog started with 5s timeout");
  }

  //Setup OTA updates:
//   // connect to Wi-Fi
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) delay(100);
//   Serial.print("IP: "); Serial.println(WiFi.localIP());

//   // configure OTA
//   ArduinoOTA.setHostname("WetBench");
//   // For a password‐protected upload (optional):
//   ArduinoOTA.setPassword("helloWorld");

//   // start OTA service
//   if (!ArduinoOTA.begin(WiFi.localIP(), "WetBench", "helloWorld", InternalStorage)) {
//     Serial.println("OTA init failed!");
//   } else {
//     Serial.println("OTA ready");
//   }
}

void loop(){
    // Handle watchdog timer
    WDT.refresh();

    // Handle OTA
    // ArduinoOTA.handle();
    
    // Handle PH value update - NON BLOCKING (ie, don't fucking average it from here)
    double currentPH = update_pH(); 

    // Handle display update - NON BLOCKING
    updateDisplay(currentPH);
    
    //temp debug, serial print the current ph once every x seconds 
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 30000UL) {
        Serial.print("PH: ");
        Serial.println(currentPH);
        lastDebugTime = millis();
    }

    //Handle pushButton isr triggers
    if(attemptNeutralizationSystemFlush){
        attemptNeutralizationSystemFlush = false;
        if(safe_pH_range(currentPH)){ 
            //good to flush, PH is at safe level
            systemState = FLUSHING; 

            Serial.println("Push Button - Long Press: PH SAFE, flushing tank");
        
            displayState = POPUP_DISPLAY;
            popupDuration = 3000; 
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("FLUSHING!");
            lcd.setCursor(0,1);
            lcd.print("Safe to flush");
        }
        else{
            Serial.println("Push Button - Long Press: PH UNSAFE, ignoring press");

            //ignore flush request, PH needs adjustment
            displayState = POPUP_DISPLAY;
            popupDuration = 3000; 

            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Flush Canceled");
            lcd.setCursor(0,1);
            lcd.print("PH Not Safe!");
        }
    }

    if(toggleNeutralizationSystem){
        //if neutralization system is being disabled, save time stamp
        if(neutralizationEnabled) {neutralizationDisabledTime = millis();}
        neutralizationEnabled = !neutralizationEnabled;
        toggleNeutralizationSystem = false;

        Serial.print("Push Button - Short Press: Toggling Neut System - ");
        Serial.println(neutralizationEnabled? "ENABLED" : "DISABLED");

        displayState = POPUP_DISPLAY;
        popupDuration = 4500; 
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("NEUTRALIZATION:");
        lcd.setCursor(0,1);
        lcd.print(neutralizationEnabled? "ENABLED" : "DISABLED");
    }
    
    //Handle last system disable time out
    if(millis() - neutralizationDisabledTime > 3600000UL){
        Serial.println("Time Out Triggered - System Deactivated for >1hr");

        displayState = POPUP_DISPLAY;
        popupDuration = 5000; 
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("RENABLING SYSTEM");
        lcd.setCursor(0,1);
        lcd.print("- TIMED OUT -");
    }
    
    //Handle Run system - 
    if(neutralizationEnabled){
        //TODO if highTank trigger, and swap to flush, ensure we are in a safe neut range
        switch(systemState){
            case IDLE:
                //TODO pass in pH
                //check to see if tank neut/flush is needed (can only be allowed while we know we are idling (flushing not occuring))
                systemState =  (safe_pH_range(currentPH))? IDLE : NEUTRALIZING;
                
                
                if(systemState == NEUTRALIZING){Serial.println("System State Change - GOING TO NEUTRALIZING - PH is bad");} //debug print out - if we change system state we finish off the IDLE case first before moving. Alows us to only print this once

                ensureNeutralSystemState(); //ensure we aren't dumping chemicals or mixing while idling
                ensureNeutralFlushingState(); //esnure valve is closed and pump is off
                break;

            case NEUTRALIZING:
                ensureNeutralFlushingState(); //ensure valve is closed and pump is off

                //System state remains in neutralizing until the neutralizeSystem returns true (neutralize done)
                systemState = neutralizeSystem(currentPH) ? IDLE : NEUTRALIZING;

                if(systemState == IDLE){Serial.println("System State Change - GOING TO IDLE - Neutralizing complete");} //debug print out - if we change system state we finish off the IDLE case first before moving. Alows us to only print this once

                //TODO check if we want to do some kind of flush after any neutralizing
                break;

            case FLUSHING:
                ensureNeutralSystemState(); //just ensure while we are flushign the tank we are not mixing, or dumping chems

                //System state remains in flushing until the tank is empty, then and only then allowed ot return to idle
                systemState = flushTank() ? IDLE : FLUSHING;

                if(systemState == IDLE){Serial.println("System State Change - GOING TO IDLE - Flushing complete");} //debug print out - if we change system state we finish off the IDLE case first before moving. Alows us to only print this once

                break;
        }
    }
}

bool neutralizeSystem(double currentPH){
    //If we are inside the neutralizing state, as saftey ensure required valves and pumps are closed:
    switch(neutralizingState){
        case NEUTRALIZING_STATE::MIXING:
            digitalWrite(MIXING_RELAY_PIN, HIGH); //Start up the mixer, we in the mixing state
            
            //add message to display that we are mixing
            enableSubMessage = true;
            subMessageShift = initialMixComplete? 2 : 0;
            subMessage = initialMixComplete? "- Mixing -" : "- Initial Mix -";

            //if timer is disabled enable timer
            timer_100ms.enable_overflow_irq();
            
            //if initialMixComplete == true, we compare against systemMix, otherweise we compare against initial mix
            //If mix counter is complete, move to chemical deposition, note, we must first return to the main loop
            if(mixCount > (initialMixComplete? systemMix_MAX_CNT_SEC : initialMix_MAX_CNT_SEC)){
                if(!initialMixComplete) {initialMixComplete = true;} //ensure if this was the initial mix to change timing, could just always set to true (latch that bitch)
                
                selectedCaseAdjustment = getCaseAdjustment(currentPH); //TODO pass the currentPH
                //TODO disable timer and ensure counter reset.
                mixCount = 0;
                timer_100ms.disable_overflow_irq();

                neutralizingState = NEUTRALIZING_STATE::CHEM_DEP;

                if(neutralizingState == CHEM_DEP){Serial.println("Neutralizing State Change - Chemical Adjusting");} //debug print out - if we change system state we finish off the IDLE case first before moving. Alows us to only print this once

                enableSubMessage = false; //Make sure we stop showing the subMessage on the LCD
                return false; //We aren't done with neutralizing yet, so keep returning false
            }
            return false;

        case NEUTRALIZING_STATE::CHEM_DEP:
            digitalWrite(MIXING_RELAY_PIN, LOW); //ensure we aren't mixing while dumping chems

            //TODO ensure timer is enabled
            timer_100ms.enable_overflow_irq(); 

            //add message to display that we are dumping chemicals
            enableSubMessage = true;
            subMessageShift =  0;
            subMessage = "- Adjusting PH -";

            //TODO add back up timeStamp
            int chemcialRelayPin = (selectedCaseAdjustment.pH_trigger > 7)? BASE_RELAY_PIN : ACID_RELAY_PIN; //determine the type of chem we are correcting with (BASE or ACID)
            digitalWrite(chemcialRelayPin, HIGH); //ensure chemical pin (either ACID or BASE) is on and dumping until cnt is over

            if(mixCount > selectedCaseAdjustment.chemDep_Cnt_Sec){
                digitalWrite(chemcialRelayPin, LOW); //make sure the relay gets turned off
                //disable timer and reset count
                timer_100ms.disable_overflow_irq();
                mixCount = 0;
                neutralizingState = NEUTRALIZING_STATE::MIXING;
                enableSubMessage = false; //make sure we stop showing the subMessage on the LCD

                if(neutralizingState == CHEM_DEP){Serial.println("Neutralizing State Change - Mixing");} //debug print out - if we change system state we finish off the IDLE case first before moving. Alows us to only print this once

                return true; //We are finally done with the neutralizing, notify main state machine
            }
            return false;
    }
}

/**
 * Flush tank is responsible for first filling the tank to max volume (if not already) and then flushing
 * entirety. Passes back a boolean on state completion, and internal sub STATE CHANGES are only allowed on
 * init, intnternal to the function, and on emergency reset.
 */
bool flushTank(){
    switch(flushingState){
        case FILLING_TANK:
            enableSubMessage = true;
            subMessageShift =  0;
            subMessage = "-Fill to Flush-";

            if((!digitalRead(TANK_HIGH_SENSOR_PIN)) && !digitalRead(OVERFLOW_SENSOR_PIN)){
                //Ensure valve is open and pump is off - lets fill the tank 
                digitalWrite(PUMP_RELAY_PIN, LOW); 
                digitalWrite(VALVE_RELAY_PIN, HIGH);
            }
            if((digitalRead(TANK_HIGH_SENSOR_PIN)) || digitalRead(OVERFLOW_SENSOR_PIN)){
                if(digitalRead(OVERFLOW_SENSOR_PIN)){

                }

                //close valve immediatly and start emptying tank, we are full and maybe even overflowing
                digitalWrite(VALVE_RELAY_PIN, LOW);
                flushingState = EMPTYING_TANK;

                if(flushingState == CHEM_DEP){Serial.println("Flushing State Change - Dumping Tank");} //debug print out - if we change system state we finish off the IDLE case first before moving. Alows us to only print this once

                enableSubMessage = false;
                return false; //we are done filling, but not with flush process, do not return true yet
            }
            return false;

        case EMPTYING_TANK:
            enableSubMessage = true;
            subMessageShift =  0;
            subMessage = "-Flushing Tank-";

            digitalWrite(VALVE_RELAY_PIN, LOW); //saftey, make sure valve is closed
            digitalWrite(PUMP_RELAY_PIN, HIGH); //make sure the pup is active and running
           
            if(!digitalRead(TANK_LOW_SENSOR_PIN)){
                //if and when tank low sensor pin is released, the emptying process is done
                digitalWrite(PUMP_RELAY_PIN, LOW);
                flushingState = FILLING_TANK;

                Serial.println("Flushing State Change - Completed, tank emptied");

                return true; //we are done with entire flush process, notify the main STATE loop.
            }
            return false;
    } 
}



void updateDisplay(double pH){
    unsigned long now = millis();
    static unsigned long lastUpdate = 0;
    const unsigned long interval = 250;//250ms delay to avoid screen flicker

    switch(displayState){
        case POPUP_DISPLAY:
            if (now - popupStartTime > popupDuration) {
                // popup expired → go back to normal
                displayState = NORMAL_DISPLAY;
                lcd.clear();
            }
            return;

        case NORMAL_DISPLAY:
            if(now - lastUpdate > interval){
                //update the display with a normal PH level, no pop up below.
                lastUpdate = now;
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("PH: ");
                lcd.setCursor(4,0);
                lcd.print(pH);

                if(enableSubMessage){
                    lcd.setCursor(subMessageShift, 1);
                    lcd.print(subMessage);
                }
            }
            return;
    }
}



double update_pH() {
  unsigned long now = millis();
  const unsigned long REQUEST_INTERVAL = 250;  // ms between requests
  const unsigned long TIMEOUT_MS       = 100;  // ms before giving up

  switch (phState) {
    case IDLE_s:
      // time to kick off a new measurement?
      if (now - lastReqTime >= REQUEST_INTERVAL) {
        buf          = "";
        Serial1.print("R\r");
        startTime    = now;
        lastReqTime  = now;
        phState      = WAITING_s;
      }
      break;

    case WAITING_s:
      // read *one* char if available
      if (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\r') {
          // complete packet, add to movingAverage and return
          lastPH = phFilter.reading(buf.toFloat());   
          phState   = IDLE_s;
        }
        else {
          buf += c;
        }
      }
      // or time out this request
      else if (now - startTime >= TIMEOUT_MS) {
        phState = IDLE_s;
      }
      break;
  }

  return lastPH;
}

struct pH_caseAdjusments getCaseAdjustment(double currentPH){
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

bool safe_pH_range(double pH){
    double pH_UpperSafeRange = 9.5;
    double pH_LowerSafeRange = 5.5;

    return (pH <= pH_UpperSafeRange) && (pH >= pH_LowerSafeRange);
}

/**
 * Helper/saftey funciton to ensure outputs/vars that are needed to properly handle mixing and neutralizing
 * are in their "Neutral" Sate
 */
void ensureNeutralSystemState(){
    digitalWrite(ACID_RELAY_PIN, LOW);
    digitalWrite(BASE_RELAY_PIN, LOW);
    digitalWrite(MIXING_RELAY_PIN, LOW);
    mixCount = 0;
    initialMixComplete = false; //ensure initial Mix time is first mix
    neutralizingState = MIXING; //always make sure we start with mixing first               
}

void ensureNeutralFlushingState(){
    digitalWrite(VALVE_RELAY_PIN, LOW); //ensure vlave is off 
    digitalWrite(PUMP_RELAY_PIN, LOW); //ensure pump is off
    timer_100ms.disable_overflow_irq(); //ensure clock isr is off. we aren't timing shit right now
}