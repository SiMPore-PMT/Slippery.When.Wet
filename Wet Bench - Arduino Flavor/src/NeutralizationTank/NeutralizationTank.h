// #include <Arduino.h>
// #include <Wire.h>                 // I2C communication library
// #include <LiquidCrystal_I2C.h>    // LCD library for I2C displays
// #include <math.h>                 // Math library for calculations
// #include "FspTimer.h"             // FSP library for uno R4
// #include <movingAvg.h>            // Moving average filter for PH probve


// // ──────────────────────────────────────────── Pin Definition Config ───────────────────────────────────────────────────
// // Define pin numbers for various relays and sensors
// #define MIXING_RELAY_PIN     2      // Pin for mixing relay
// #define VALVE_RELAY_PIN      3      // Pin for valve relay
// #define ACID_RELAY_PIN       4      // Pin for acid relay
// #define BASE_RELAY_PIN       5      // Pin for base relay
// #define PUMP_RELAY_PIN       6      // Pin for pump relay
// #define OVERFLOW_SENSOR_PIN  7      // Overflow sensor pin
// #define TANK_LOW_SENSOR_PIN  9      // Low-level sensor pin (tank empty)
// #define TANK_HIGH_SENSOR_PIN 10     // High-level sensor pin (tank full)
// #define PH_SENSOR_PIN        A5     // Analog input for pH sensor
// #define FLUSH_BUTTON_PIN     13     // Button input pin for flushing/toggling neutralization
// #define PH_SENSOR_RX_PIN     0 //11     // HardwareSerial RX pin for pH sensor 
// #define PH_SENSOR_TX_PIN     1 //12    // HardwareSerial TX pin for pH sensor
// // ──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────



// // ─────────────────────────────────────── PH Probe Variable Config ──────────────────────────────────────────────────────

// // Global variables for system state, sensor readings, and commands
// // bool isPHAcceptable = true;            // Flag indicating if the current pH is within acceptable range
// // bool tankFlushed = false;              // Flag indicating whether the tank flush process has been completed
// String phReadCommand = "R";            // Command to request a pH reading from the sensor
// String pcInputBuffer = "";             // Buffer for incoming data from the PC (not used)
// String phSensorResponseBuffer = "";    // Buffer for data received from the pH sensor
// float currentPH = 7;                   // Default pH value
// bool isPHUpdated = false;              // Flag to indicate if the pH value was updated (currently not used)
// bool shouldExitLoop = false;           // Flag to exit the main loop (currently not used)
// // ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


// // ───────────────────────────────────────── Display Variable Config ───────────────────────────────────────────────────
// // Initialize the LCD at I2C address 0x27 with 20 columns and 4 rows
// LiquidCrystal_I2C lcd(0x27, 20, 4);
// // ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────


// FspTimer timer_100ms;

// movingAvg phFilter(11); //Create a 11 sample moving average filter


// // ──────────────────────────────────── Neutralization Ctrl Variable Config ────────────────────────────────────────────
// // Global variables for neutralization toggle control
// bool neutralizationEnabled = true;           // Neutralization is enabled by default

// volatile unsigned long neutralizationDisabledTime = 0;  // Timestamp when neutralization was toggled off
// // ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

// constexpr uint32_t DEBOUNCE_DELAY   = 50;     // ms debounce window
// constexpr uint32_t LONG_PRESS_TIME  = 3000;   // ms threshold for long press

// // Internal ISR timing
// volatile uint32_t lastIntTime = 0;    // timestamp of last valid edge
// volatile uint32_t pressTime   = 0;    // timestamp when button went LOW (pressed)

// volatile bool toggleNeutralizationSystem = false; //So we can do more than just toggle things (remove workload from isr)
// volatile bool attemptNeutralizationSystemFlush = false;

// enum NEUTRALIZING_STATE {MIXING, CHEM_DEP};         //Neutralizing specific state variables (basically sub states), default state should be mixing 
// enum FLUSHING_STATE {FILLING_TANK, EMPTYING_TANK};  //Flushing specific state variables (more sub states), defualt state should be filling
// FLUSHING_STATE flushingState = FILLING_TANK;


// enum SYSTEM_STATE {IDLE, NEUTRALIZING, FLUSHING}; //System state enums for the entire neutralization system

// SYSTEM_STATE systemState = IDLE;
// NEUTRALIZING_STATE neutralizingState = MIXING;

// double initialMix_MAX_CNT_SEC = 25.0;  //initial mixingtime before chemicals are applied. Represented in seconds
// double systemMix_MAX_CNT_SEC = 45.0;  //mixing time between adding more chemicals assuming PH has not been restored.

// volatile double mixCount = 0; //Tracking variable for mixing timing
// bool initialMixComplete = false;

// struct pH_caseAdjusments{
//     double pH_trigger;      //point at which range search is started. If pH is >7, range adds. If ph <7 range subtracts
//     double range;           //range from which the pH case adjustment parameters apply
//     double chemDep_Cnt_Sec; //amount of time to dump acid/base for given case can do .1s (100ms) incraments. Note: chem choice dependent on pH_trigger range (ie >7 acid is used)
// };

// pH_caseAdjusments selectedCaseAdjustment;

// const int totalNumCases = 9;
// pH_caseAdjusments caseArray[totalNumCases] = {
//     //---- Base Case Adjustments (we are adding acid)
//     {9.5, 1, 2},           //Base of 9.5 - 10.5, add acid for 2 seconds
//     {10.5, 0.5, 3},      //Base of 10.5 - 11, add acid for 3 seconds
//     {11, 3, 10},          //Base of 11 -14, add acid for 10 seconds ????

//     //---- Acid Case Adjustments (we are adding base)
//     {5.5, 0.5, 1},         //Acid of 5.5 - 5, add base for 1 seconds
//     {5, 0.5, 1.5},       //Acid of 5 - 4.5, add base for 1.5 seconds
//     {4.5, 0.5, 2},   //Acid of 4.5 - 4, add base for 2 seconds
//     {4, 0.5, 3},           //Acid of 4 - 3.5, add base for 3 seconds
//     {3.5, 0.5, 4},        //Acid of 3.5 - 3, add base for 4 seconds
//     {3, 2, 8}        //Acid of 3 - 1, add base for 8 seconds
// };

// enum ph_ReadSensorState { IDLE_s, WAITING_s };
// ph_ReadSensorState phState = IDLE_s;
// double   lastPH      = 0.0;
// String  buf         = "";
// unsigned long lastReqTime = 0, startTime = 0;



// void flushButton_ISR() {}
// void overflowSensor_ISR(){}
// void highTankSensor_ISR(){}
// void timer_100ms_callback(timer_callback_args_t *args){}

// void setup() {}
// void loop(){}

// bool neutralizeSystem(double currentPH){}
// bool flushTank(){}
// double update_pH() {}

// pH_caseAdjusments getCaseAdjustment(double currentPH){}
// bool safe_pH_range(double pH){}
// void ensureNeutralSystemState(){}
// void ensureNeutralFlushingState(){}