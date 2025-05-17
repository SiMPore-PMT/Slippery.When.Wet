// #include <Wire.h>
// #include <SparkFun_FS3000_Arduino_Library.h>
// #include <LiquidCrystal_I2C.h> 
// #include <movingAvg.h>
// #include <SPI.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

// #if defined(__AVR__)
//   #include <avr/wdt.h> 
// #elif defined(ARDUINO_UNOR4_WIFI)
//   // Renesas-specific includes or Arduino fallback
// #endif


// #define FAN 3
// #define ONOFF 8

// int SPEED = 0;
// int ReadRawAvgCheck;

// LiquidCrystal_I2C lcd(0x27,20,4); 
// FS3000 fs;
// movingAvg FaceVelocity(20); 

// void setup() {
//  // put your setup code here, to run once:
//  pinMode(FAN,OUTPUT);
//  pinMode(ONOFF, INPUT);
//  Serial.begin(9600);
//  FaceVelocity.begin();
//  Wire.begin();
//  lcd.clear();
//  lcd.setCursor(0,0);
//  lcd.print("FACE VELOCITY");

//  if (fs.begin() == false) { //Begin communication over I2C
//   Serial.println("The sensor did not respond. Please check wiring.");
//   while(1); //Freeze
//  }

//  // Set the range to match which version of the sensor you are using.
//  // FS3000-1005 (0-7.23 m/sec) --->>>  AIRFLOW_RANGE_7_MPS
//  // FS3000-1015 (0-15 m/sec)   --->>>  AIRFLOW_RANGE_15_MPS
//  //fs.setRange(AIRFLOW_RANGE_7_MPS);
//  fs.setRange(AIRFLOW_RANGE_15_MPS); 
//  Serial.println("Sensor is connected properly.");
//  analogWrite(FAN, SPEED);
 
//   #if defined(__AVR__)
//     wdt_disable(); //Disable WDT
//     delay(2000);
//     wdt_enable(WDTO_8S);
//   #endif
//  lcd.init();                      // initialize the lcd 
//  lcd.backlight();
//  lcd.setCursor(0,0);

// }



// void loop() {
//  delay(100);
//  wdt_reset();
//  SPEED = 150;
//  analogWrite(FAN, SPEED);

//  while (digitalRead(ONOFF) == HIGH){
//   //lcd.clear();   
//   delay(500);
//   //collect value from sensor
//   int ReadRaw = fs.readRaw();
//   Serial.print("Face Velocity Raw: ");
//   Serial.print(ReadRaw);
//   Serial.print(",");

//   int ReadRawAvg = FaceVelocity.reading(ReadRaw);
//   Serial.print("Face Velocity Moving Average: ");
//   Serial.print(ReadRawAvg);
//   Serial.print(",");
//   float ReadRawFloat = ReadRawAvg;
//   wdt_reset();

//   int FaceVelocityFPM = -.0021 * sq(ReadRawFloat) + 2.7415 * ReadRawFloat - 751.42;
//   Serial.print("Face Velocity FPM: ");
//   Serial.print(FaceVelocityFPM);
//   Serial.print(",");

//   if (ReadRawAvg<401){
//    FaceVelocityFPM = 0;
//   }

//   if (ReadRawAvg>760){
//    FaceVelocityFPM = 220;
//   }
    
//   if (FaceVelocityFPM<105){
//    if (SPEED<235){
//     SPEED = SPEED + 1;
//    }
//    else {
//     SPEED = 235;  
//    }   
//   }

//   if (FaceVelocityFPM>120){
//    if (SPEED>75){
//     SPEED = SPEED - 1;
//    }
//    else {
//     SPEED = 75;
//    }
//   }

//  delay(500);
//   ReadRawAvgCheck = FaceVelocity.reading(ReadRaw);
//   if (ReadRawAvgCheck == ReadRawAvg){
//     SPEED = SPEED + 2;
//   }

 

//   analogWrite(FAN, SPEED);
//   Serial.print("Fan Speed: ");
//   Serial.println(SPEED);
//   lcd.clear();
//   lcd.setCursor(1,0);
//   lcd.print("Face Velocity");
//   lcd.setCursor(5,1);
//   lcd.print(FaceVelocityFPM);
//   lcd.setCursor(9,1);
//   lcd.print("FPM");
//   wdt_reset();
//  }

//  if(digitalRead(ONOFF) == LOW){
//   SPEED = 0;
//   analogWrite(FAN, SPEED);
//   Serial.println("off");
//   lcd.clear();
//   lcd.setCursor(1,0);
//   lcd.print("Fan off");
//   delay(1000);
//   wdt_reset();
//  }
// }
