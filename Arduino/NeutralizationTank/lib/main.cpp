#include <Arduino.h>
#include <SoftwareSerial.h>       // Software-based serial communication
#include <Wire.h>                 // I2C communication library
#include <LiquidCrystal_I2C.h>    // LCD library for I2C displays
#include <math.h>                 // Math library for calculations



#if defined(__AVR__)
  #include <avr/interrupt.h>        // AVR interrupt support
  #include <avr/io.h>
  #include <avr/wdt.h>
#elif defined(ARDUINO_UNOR4_WIFI)
  // Renesas-specific includes or Arduino fallback
#endif

// Global volatile flag (currently not used)
volatile bool timerFlag = false;

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

// Create a SoftwareSerial object for communication with the Atlas Scientific pH sensor
SoftwareSerial phSensorSerial(PH_SENSOR_RX_PIN, PH_SENSOR_TX_PIN);

// Global variables for system state, sensor readings, and commands
bool isPHAcceptable = true;            // Flag indicating if the current pH is within acceptable range
bool tankFlushed = false;              // Flag indicating whether the tank flush process has been completed
int unusedPHValue;                     // Unused variable for pH value (legacy)
float sensorVoltage;                   // Unused variable for sensor voltage
String phReadCommand = "R";            // Command to request a pH reading from the sensor
String pcInputBuffer = "";             // Buffer for incoming data from the PC (not used)
String phSensorResponseBuffer = "";    // Buffer for data received from the pH sensor
float currentPH = 7;                   // Default pH value
bool isPHUpdated = false;              // Flag to indicate if the pH value was updated (currently not used)
bool shouldExitLoop = false;           // Flag to exit the main loop (currently not used)

// Global variables for neutralization toggle control
bool neutralizationEnabled = true;           // Neutralization is enabled by default
unsigned long neutralizationDisabledTime = 0;  // Timestamp when neutralization was toggled off

// Initialize the LCD at I2C address 0x27 with 20 columns and 4 rows
LiquidCrystal_I2C lcd(0x27, 20, 4);



/**
 * pauseableDelay - delays for the given time (ms) in short increments,
 * periodically checking if neutralization has been paused via the flush button.
 *
 * @param ms: number of milliseconds to delay.
 * @return true if full delay elapsed; false if interrupted by pausing.
 */
bool pauseableDelay(unsigned long ms) {
  unsigned long startTime = millis();
  while (millis() - startTime < ms) {
    // Check for a flush button press to update neutralizationEnabled
    handleFlushButton();
    if (!neutralizationEnabled) {
      return false;
    }
    delay(10);
    #if defined(__AVR__)
      wdt_reset();
    #endif
  }
  return true;
}


/**
 * Checks if the current pH is within acceptable bounds (5.5 < pH < 9.5)
 * and updates the global flag accordingly.
 */
void checkAcceptablePH() {
  delay(500);
  Serial.println("Read");
  phSensorSerial.println("R");  // Request a new pH reading from the sensor
  delay(500);
  
  if (currentPH < 9.5 && currentPH > 5.5) { // pH is acceptable
    isPHAcceptable = true;
    Serial.println("pH is Good");
  } else if (currentPH <= 5.5 || currentPH >= 9.5) { // pH is out of bounds
    isPHAcceptable = false;
    Serial.println("pH is out of bounds!");
  }
  #if defined(__AVR__)
    wdt_reset();
  #endif
}

/**
 * Adjusts the pH by dispensing acid or base based on the current pH value.
 * Uses a switch-case structure to handle different pH ranges.
 * This version uses pauseableDelay() so that any long delay is interruptible
 * if neutralization is paused.
 */
void adjustPH() {
  int adjustmentCase = 0;
  
  // Determine which pH range we're in.
  if (currentPH > 9.5) {  // pH is too high; add acid to lower it
    if (currentPH > 11) {              // Extreme high pH
      adjustmentCase = 1;
    } else if (currentPH > 10.5) {       // Moderately high pH
      adjustmentCase = 2;
    } else {                           // Slightly above acceptable (9.5 - 10.5)
      adjustmentCase = 3;
    }
  } else if (currentPH < 5.5) {  // pH is too low; add base to raise it
    if (currentPH < 3.0) {             // Extremely low pH
      adjustmentCase = 9;
    } else if (currentPH < 3.5) {
      adjustmentCase = 8;
    } else if (currentPH < 4.0) {
      adjustmentCase = 7;
    } else if (currentPH < 4.5) {
      adjustmentCase = 6;
    } else if (currentPH < 5.0) {
      adjustmentCase = 5;
    } else {                         // pH between 5.0 and 5.5
      adjustmentCase = 4;
    }
  } else {
    // pH is within the acceptable range (5.5-9.5); no adjustment needed.
    return;
  }

  // Execute the corresponding acid/base adjustment based on the pH range.
  switch (adjustmentCase) {
    case 1: // pH > 11: Extreme high pH, add acid
      digitalWrite(ACID_RELAY_PIN, HIGH);
      if (!pauseableDelay(5000)) { digitalWrite(ACID_RELAY_PIN, LOW); return; }
      #if defined(__AVR__)
        wdt_reset();
      #endif
      if (!pauseableDelay(5000)) { digitalWrite(ACID_RELAY_PIN, LOW); return; }
      digitalWrite(ACID_RELAY_PIN, LOW);
      #if defined(__AVR__)
        wdt_reset();
      #endif  
      break;
      
    case 2: // 11 >= pH > 10.5: Moderately high pH, add acid
      digitalWrite(ACID_RELAY_PIN, HIGH);
      if (!pauseableDelay(3000)) { digitalWrite(ACID_RELAY_PIN, LOW); return; }
      digitalWrite(ACID_RELAY_PIN, LOW);
      #if defined(__AVR__)
        wdt_reset();
      #endif
      break;
      
    case 3: // 10.5 >= pH > 9.5: Slightly high pH, add acid
      digitalWrite(ACID_RELAY_PIN, HIGH);
      if (!pauseableDelay(2000)) { digitalWrite(ACID_RELAY_PIN, LOW); return; }
      digitalWrite(ACID_RELAY_PIN, LOW);
      #if defined(__AVR__)
        wdt_reset();
      #endif
      break;
      
    case 4: // 5.0 <= pH < 5.5: Slightly low pH, add base
      digitalWrite(BASE_RELAY_PIN, HIGH);
      if (!pauseableDelay(1000)) { digitalWrite(BASE_RELAY_PIN, LOW); return; }
      digitalWrite(BASE_RELAY_PIN, LOW);
      #if defined(__AVR__)
        wdt_reset();
      #endif
      break;
      
    case 5: // 4.5 <= pH < 5.0: Moderately low pH, add base
      digitalWrite(BASE_RELAY_PIN, HIGH);
      if (!pauseableDelay(1500)) { digitalWrite(BASE_RELAY_PIN, LOW); return; }
      digitalWrite(BASE_RELAY_PIN, LOW);
      #if defined(__AVR__)
        wdt_reset();
      #endif
      break;
      
    case 6: // 4.0 <= pH < 4.5: Low pH, add more base
      digitalWrite(BASE_RELAY_PIN, HIGH);
      if (!pauseableDelay(2000)) { digitalWrite(BASE_RELAY_PIN, LOW); return; }
      digitalWrite(BASE_RELAY_PIN, LOW);
      #if defined(__AVR__)
        wdt_reset();
      #endif
      break;
      
    case 7: // 3.5 <= pH < 4.0: Very low pH, add base
      digitalWrite(BASE_RELAY_PIN, HIGH);
      if (!pauseableDelay(3000)) { digitalWrite(BASE_RELAY_PIN, LOW); return; }
      digitalWrite(BASE_RELAY_PIN, LOW);
      #if defined(__AVR__)
        wdt_reset();
      #endif
      break;
      
    case 8: // 3.0 <= pH < 3.5: Extremely low pH, add base
      digitalWrite(BASE_RELAY_PIN, HIGH);
      if (!pauseableDelay(4000)) { digitalWrite(BASE_RELAY_PIN, LOW); return; }
      digitalWrite(BASE_RELAY_PIN, LOW);
      #if defined(__AVR__)
        wdt_reset();
      #endif
      break;
      
    case 9: // pH < 3.0: Critically low pH, add base with extended dosing
      digitalWrite(BASE_RELAY_PIN, HIGH);
      if (!pauseableDelay(5000)) { digitalWrite(BASE_RELAY_PIN, LOW); return; }
      #if defined(__AVR__)
        wdt_reset();
      #endif
      if (!pauseableDelay(3000)) { digitalWrite(BASE_RELAY_PIN, LOW); return; }
      digitalWrite(BASE_RELAY_PIN, LOW);
      #if defined(__AVR__)
        wdt_reset();
      #endif
      break;
  }
}

/**
 * Flushes the tank by first filling it until the high-level sensor triggers,
 * and then emptying it until the low-level sensor triggers.
 */
void flushTank() {
  Serial.println("Fill Tank");
  delay(500);
  
  // Open the valve to fill the tank until the high sensor indicates it is full.
  while (digitalRead(TANK_HIGH_SENSOR_PIN) == LOW) {
    digitalWrite(VALVE_RELAY_PIN, HIGH);  // Open valve
    delay(500);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PH: ");
    lcd.setCursor(4, 0);
    lcd.print(currentPH);
    lcd.setCursor(0, 1);
    lcd.print("Filling Tank");
    #if defined(__AVR__)
      wdt_reset();
    #endif
  }
  delay(500);
  
  // Close the valve once the tank is full.
  digitalWrite(VALVE_RELAY_PIN, LOW);
  Serial.println("Valve closed");
  #if defined(__AVR__)
    wdt_reset();
  #endif
  delay(500);
  Serial.println("Empty Tank");
  
  // Activate the pump to empty the tank until the low sensor indicates it is empty.
  while (digitalRead(TANK_LOW_SENSOR_PIN) == HIGH) {
    digitalWrite(PUMP_RELAY_PIN, HIGH);  // Start pump
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PH: ");
    lcd.setCursor(4, 0);
    lcd.print(currentPH);
    lcd.setCursor(0, 1);
    lcd.print("Emptying Tank");
    #if defined(__AVR__)
      wdt_reset();
    #endif
    delay(500);
  }
  
  // Turn off the pump once the tank is empty.
  Serial.println("Tank empty");
  digitalWrite(PUMP_RELAY_PIN, LOW);
  #if defined(__AVR__)
      wdt_reset();
  #endif
  delay(500);
  tankFlushed = false;
}

/**
 * Handles flush button presses.
 * - A short press (< 3 seconds) toggles neutralization mode on/off.
 *   When toggled from paused to active the first press displays a "resumed" message.
 * - A long press (>= 3 seconds) triggers an immediate tank flush.
 *   However, if neutralization is paused, a flush attempt will display a warning.
 */
void handleFlushButton() {
  // Check if flush button is pressed (active LOW)
  if (digitalRead(FLUSH_BUTTON_PIN) == LOW) {
    unsigned long pressStartTime = millis();
    // Wait while the button remains pressed (and reset watchdog)
    while (digitalRead(FLUSH_BUTTON_PIN) == LOW) {
      #if defined(__AVR__)
        wdt_reset();
      #endif
      delay(10);
    }
    delay(50); // Debounce delay
    unsigned long currentTime = millis();
    unsigned long pressDuration = currentTime - pressStartTime;
    if (currentTime < pressStartTime) {
      Serial.println("Ignoring button press: Current time is less than press start time");
      return;
    }
    if (pressDuration >= 3000) { // Long press: flush tank immediately
      if (!isPHWithinAcceptableRange(currentPH)) {
        Serial.println("Warning: Flush attempted while pH is out of bounds");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Flush Canceled:");
        lcd.setCursor(0, 1);
        lcd.print("Bad pH");
        delay(2000);
        return;
      } else {
        Serial.println("Long press detected: Flushing Tank");
        if (isPHWithinAcceptableRange(currentPH)) {
          flushTank();
        }
      }
    } else { // Short press: toggle neutralization mode
      neutralizationEnabled = !neutralizationEnabled;
      if (!neutralizationEnabled) {
        neutralizationDisabledTime = millis(); // Record the time when neutralization was disabled
        Serial.println("Neutralization disabled via short press");
      } else {
        Serial.println("Neutralization resumed");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Neutralization");
        lcd.setCursor(0,1);
        lcd.print("Resumed       ");
        delay(2000); // Display resumed message briefly
        lcd.clear();
      }
    }
  }
}



/**
 * Reads the pH value from the sensor using SoftwareSerial communication.
 * Sends a command to request a reading and parses the returned value.
 * If conversion to float fails (resulting in 0), the function retries up to 3 times.
 * @return The pH value as a float.
 */
float readPH() {
  #if defined(__AVR__)
    wdt_reset();
  #endif
  float phValue = 0.0;
  int retries = 0;
  while (retries < 3) { // try a maximum of 3 times
    phSensorResponseBuffer = ""; // clear previous response
    phSensorSerial.print("R \r");
    unsigned long startTime = millis();
    // Wait for sensor data with a timeout
    while (!phSensorSerial.available() && (millis() - startTime) < 1000) {
      #if defined(__AVR__)
        wdt_reset();
      #endif
      delay(10);
    }
    delay(50); // Allow time for data to be fully received
    while (phSensorSerial.available()) {
      char inChar = (char)phSensorSerial.read();
      phSensorResponseBuffer += inChar;
      if (inChar == '\r') { // end of data indicated by carriage return
        break;
      }
    }
    phValue = phSensorResponseBuffer.toFloat();
    if (phValue != 0.0) {
      break;
    }
    retries++;
    delay(500);
  }
  Serial.println(phValue);
  return phValue;
}

/**
 * Checks if a given pH value is within the acceptable range (5.5 < pH < 9.5).
 * @param phValue The pH value to check.
 * @return True if the pH is within range, false otherwise.
 */
bool isPHWithinAcceptableRange(float phValue) { 
  if (phValue < 9.5 && phValue > 5.5) {
    Serial.println("pH is Good");
    return true;
  } else {
    Serial.println("pH is out of bounds!");
    return false;
  }
}




void setup() {
  // Disable interrupts during timer setup
  noInterrupts();
  
  #if defined(__AVR__)
    //DAK - they aint even using these anymore 
    cli();
    // Configure Timer1 for CTC (Clear Timer on Compare Match) mode
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 15624;   // Set compare match value for approx. 1Hz (with 16MHz clock and 1024 prescaler)
    
    // Set Timer1 to CTC mode and start it with a prescaler of 1024
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS12) | (1 << CS10);
    // Enable Timer1 compare match interrupt
    TIMSK1 |= (1 << OCIE1A);
     // Re-enable interrupts after timer configuration
    interrupts();
    sei();
  #endif

 

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

  // Disable the watchdog timer, delay for stabilization, then re-enable it with an 8-second timeout
  #if defined(__AVR__)
    wdt_disable();
    delay(3000);
    wdt_enable(WDTO_8S);
  #endif
  
  Serial.println("WDT Enabled");
  Serial.println("Begin");
  
  // Send initialization commands to the pH sensor
  phSensorSerial.print("*OK,0 \r");
  phSensorSerial.print("R \r");
}

/**
 * Main loop function.
 * Performs periodic tasks such as reading the pH sensor, adjusting pH, updating the LCD,
 * handling flush operations, and monitoring neutralization mode.
 */
void loop() {
  #if defined(__AVR__)
    wdt_reset();
  #endif  // Reset watchdog timer
  Serial.println("wdt reset...");
  
  // Process flush button input (for toggling neutralization or flushing the tank)
  handleFlushButton();
  
  // Automatically re-enable neutralization after one hour if it was disabled
  if (!neutralizationEnabled && (millis() - neutralizationDisabledTime >= 3600000UL)) {
    neutralizationEnabled = true;
    Serial.println("One hour elapsed: Neutralization re-enabled automatically");
  }
  
  if (!neutralizationEnabled) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("NEUTRALIZATION");
    lcd.setCursor(0,1);
    lcd.print("PAUSED         ");
    Serial.println("Neutralization is paused");
    delay(500);
    return;
  } else {
    // Update neutralization status on LCD
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("PH is Good   ");
  }
  
  // Read the current pH value from the sensor
  currentPH = readPH();
  
  // Update the LCD display with the current pH value
  lcd.setCursor(0, 0);
  lcd.print("PH: ");
  lcd.setCursor(4, 0);
 	lcd.print(currentPH);
  
  bool initialMixRequired = true;
  delay(500);
  
  // Continue to adjust pH until it is within the acceptable range (5.5 < pH < 9.5)
  while (!isPHWithinAcceptableRange(currentPH)) {
    // Check for pause within the correction loop
    handleFlushButton();
    if (!neutralizationEnabled) {
      digitalWrite(MIXING_RELAY_PIN, LOW);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("NEUTRALIZATION");
      lcd.setCursor(0,1);
      lcd.print("PAUSED         ");
      Serial.println("Neutralization paused during pH correction");
      return; // Exit loop() to allow pause state to persist
    }
    
    if (initialMixRequired) {
      // Perform an initial mix to help distribute chemicals evenly
      Serial.println("Initial mix");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("pH Change: Mix");
      lcd.setCursor(0, 1);
      lcd.print("Initial Mix  ");
      digitalWrite(MIXING_RELAY_PIN, HIGH);  // Start mixing
      
      // Replace long delays with pauseableDelay calls for initial mix
      for (int i = 0; i < 5; i++) {
        if (!pauseableDelay(5000)) {
          digitalWrite(MIXING_RELAY_PIN, LOW);
          return;
        }
        #if defined(__AVR__)
          wdt_reset();
        #endif
      }
      digitalWrite(MIXING_RELAY_PIN, LOW);   // Stop mixing
      #if defined(__AVR__)
        wdt_reset();
      #endif
      lcd.clear();
      initialMixRequired = false;
    } 
    
    Serial.println("Correct PH");
    delay(500);
    // Adjust the pH by dispensing the appropriate acid or base
    adjustPH();
    
    // Continue mixing during the correction process
    digitalWrite(MIXING_RELAY_PIN, HIGH);
    // Instead of a long delay, check the pH repeatedly over 30 iterations (~21 seconds total)
    for (int i = 0; i < 30; i++) {
      handleFlushButton(); // Allow flush/neutralization toggle mid-loop
      if (!neutralizationEnabled) {
        digitalWrite(MIXING_RELAY_PIN, LOW);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("NEUTRALIZATION");
        lcd.setCursor(0,1);
        lcd.print("PAUSED         ");
        Serial.println("Neutralization paused during pH correction");
        return;
      }
      currentPH = readPH();
      // Record pH values (not used elsewhere)
      int phValues[30];
      phValues[i] = currentPH;
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("PH: ");
      lcd.setCursor(4, 0);
      lcd.print(currentPH);
      lcd.setCursor(0, 1);
      lcd.print("Correcting PH");
      #if defined(__AVR__)
        wdt_reset();
      #endif
      
      // Check for an overflow condition using the overflow sensor
      if (digitalRead(OVERFLOW_SENSOR_PIN) == HIGH) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("OVERFLOW");
        lcd.setCursor(0, 1);
        lcd.print("ERROR");
        delay(6000);
        #if defined(__AVR__)
          wdt_reset();
        #endif
        digitalWrite(MIXING_RELAY_PIN, LOW);
        flushTank();  // Immediately flush the tank if an overflow occurs regardless of pH
        Serial.println("Overflow detected: Flushing tank");
        return;
      }
      if (!pauseableDelay(700)) {
        digitalWrite(MIXING_RELAY_PIN, LOW);
        return;
      }
    }
  }
  digitalWrite(MIXING_RELAY_PIN, LOW);
  
  // If the tank is full (high sensor triggered) and the pH is acceptable, flush the tank
  if (digitalRead(TANK_HIGH_SENSOR_PIN) == HIGH) {
    if (isPHWithinAcceptableRange(currentPH)) {
      flushTank();
    }  
  }
}
