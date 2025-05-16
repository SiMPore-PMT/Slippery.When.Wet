#include <avr/io.h>
#include <avr/interrupt.h>
#include <SoftwareSerial.h> 
#include<avr/wdt.h>    
#include <Wire.h>
#include <LiquidCrystal_I2C.h>                     
volatile bool timerFlag = false;



#define MIXRELAY 2
#define VALVERELAY 3
#define ACIDRELAY 4
#define BASERELAY 5
#define PUMPRELAY 6 
#define ESENS 7 
#define LOWSENS 9
#define HIGHSENS 10
#define PH A5
#define BUTTON 13
#define rx 11                                         
#define tx 12                                         
 
SoftwareSerial myserial(rx, tx); 
bool good = true;
bool CleanTank = false;
int pH_Value; 
float Voltage;
String singleread = "R";
String inputstring = "";                              //a string to hold incoming data from the PC
String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
float pH = 7; 
bool phUpdated = false;
bool exit_loop=false;

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,20,4); 



void setup() {
  // put your setup code here, to run once:
  noInterrupts();
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 15624;
  TCCR1B |= (1<<WGM12);
  TCCR1B |= (1<<CS12) | (1<<CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
  sei();


  pinMode(MIXRELAY,OUTPUT);
  pinMode(BASERELAY,OUTPUT);
  pinMode(ACIDRELAY,OUTPUT);
  pinMode(PUMPRELAY,OUTPUT);
  pinMode(VALVERELAY,OUTPUT);
  pinMode(LOWSENS,INPUT);
  pinMode(HIGHSENS,INPUT);
  pinMode(ESENS,INPUT);
  pinMode(PH,INPUT);
  pinMode(BUTTON,INPUT);
  delay(400);
  digitalWrite(MIXRELAY,LOW);
  digitalWrite(BASERELAY,LOW);
  digitalWrite(ACIDRELAY,LOW);
  digitalWrite(PUMPRELAY,LOW); 
  digitalWrite(VALVERELAY,LOW); 
  digitalWrite(BUTTON,HIGH);
  
  Serial.begin(9600);
  myserial.begin(9600);                               
  //inputstring.reserve(10);                            
  sensorstring.reserve(30); 
  wdt_disable(); //Disable WDT
  delay(3000);
  wdt_enable(WDTO_8S); //Enable WDT with a timeout of 2 seconds
  Serial.println("WDT Enabled");
  Serial.println("Begin");
  
  myserial.print("*OK,0 \r");
  myserial.print("R \r");

  
  
}

ISR(TIMER1_COMPA_vect){
  
}

//FUNCTIONS



void acceptablePH() {

 delay(500);
 Serial.println("Read");
 myserial.println("R");
 delay(500);
 if (pH < 8 and pH > 6){ //within acceotable bounds
    good = true;
    Serial.println("pH is Good");
  }
  else if(pH < 6 or pH > 8){ // out of bounds
    good = false;
    Serial.println("pH is out of bounds!");
  }
 wdt_reset();
}

void correctPH() {
   if (pH > 11) {
    digitalWrite(ACIDRELAY,HIGH);
    delay(5000);
    wdt_reset();
    delay(5000);
    digitalWrite(ACIDRELAY,LOW);
    wdt_reset();
   }
   else if (11 > pH && pH > 10.5) {
    digitalWrite(ACIDRELAY,HIGH);
    delay(3000);
    digitalWrite(ACIDRELAY,LOW);
    wdt_reset();
   }
   else if (10.5 > pH && pH > 9.5) {
    digitalWrite(ACIDRELAY,HIGH);
    delay(2000);
    digitalWrite(ACIDRELAY,LOW);
    wdt_reset();
   }
  else if (9.5 > pH && pH > 9) {
    digitalWrite(ACIDRELAY,HIGH);
    delay(1500);
    digitalWrite(ACIDRELAY,LOW);
    wdt_reset();
   }
   else if (6.5 > pH && pH > 6) {
    digitalWrite(ACIDRELAY,HIGH);
    delay(750);
    digitalWrite(ACIDRELAY,LOW);
    wdt_reset();
   }
   else if (6 > pH && pH > 5) {
    digitalWrite(BASERELAY,HIGH);
    delay(1000);
    digitalWrite(BASERELAY,LOW);
    wdt_reset();
   }
   else if (5 > pH && pH > 4.5) {
    digitalWrite(ACIDRELAY,HIGH);
    delay(1500);
    digitalWrite(ACIDRELAY,LOW);
    wdt_reset();
   }
   else if (4.5 > pH && pH > 3.5) {
    digitalWrite(BASERELAY,HIGH);
    delay(2000);
    digitalWrite(BASERELAY,LOW);
    wdt_reset();
   }
   else if (3.5 > pH && pH > 3) {
    digitalWrite(BASERELAY,HIGH);
    delay(3000);
    digitalWrite(BASERELAY,LOW);
    wdt_reset();
   }
   else if (3 > pH) {
    digitalWrite(BASERELAY,HIGH);
    delay(2000);
    wdt_reset();
    delay(3000);
    digitalWrite(BASERELAY,LOW);
    wdt_reset();
   }
}

void FlushTank() {
 Serial.println("Fill Tank");
 delay(500);
 //OPEN FRESH WATER VALVE UNTIL UPPER SENSOR IS REACHED
 while (digitalRead(HIGHSENS)==LOW) {
   digitalWrite(VALVERELAY,HIGH);
   delay(500);
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("PH: ");
   lcd.setCursor(4,0);
   lcd.print(pH);
   lcd.setCursor(0,1);
   lcd.print("Filling Tank");
   wdt_reset();
   }
 delay(500);
 //CLOSE VALVE ONCE UPPER SENSOR IS REACHED
 digitalWrite(VALVERELAY,LOW);
 Serial.println("Valve closed");
 wdt_reset();
 delay(500);
 Serial.println("empty tank");
 //PUMP TANK UNTIL LOWER SENSOR IS SHUT OFF
 while(digitalRead(LOWSENS)==HIGH) {
  digitalWrite(PUMPRELAY,HIGH);
  lcd.setCursor(0,0);
  lcd.print("PH: ");
  lcd.setCursor(4,0);
  lcd.print(pH);
  lcd.setCursor(0,1);
  lcd.print("Emptying Tank");
  wdt_reset();
  delay(500);
  }

 //TURN OFF PUMP ONCE TANK IS EMPTIED
 Serial.println("Tank empty");
 digitalWrite(PUMPRELAY,LOW);
 wdt_reset();
 delay(500);
 CleanTank = false;
 }
 

void loop() {
 wdt_reset();
 pH = getPH();

 lcd.init();                      
 lcd.backlight();
 lcd.setCursor(0,0);
 lcd.print("PH: ");
 lcd.setCursor(4,0);
 lcd.print(pH);
 lcd.setCursor(0,1);
 lcd.print("PH is Good");

 bool InitialMix = true;
 delay(500);

 while (!checkPHRange(pH)){

  if (InitialMix == true) {
   Serial.println("initial mix");
   lcd.setCursor(0,0);
   lcd.print("pH Change: Mix");
   lcd.setCursor(0,1);
   lcd.print("Initial Mix");
   digitalWrite(MIXRELAY,HIGH);
   delay(5000);
   wdt_reset();
   delay(5000);
   wdt_reset();
   delay(5000);
   wdt_reset();
   delay(5000);
   wdt_reset();
   delay(5000);
   digitalWrite(MIXRELAY,LOW);
   wdt_reset();
   lcd.clear();
   InitialMix = false;
  } 

  Serial.println("Correct PH");
  delay(500);
  //DISPENSE ACID OR BASE TO CORRECT PH  
  correctPH(); 
  //MIX FOR 15 SEC
  digitalWrite(MIXRELAY,HIGH);
  //delay(15000);
  for(int i=0; i < 30; i++){
   pH = getPH();
   int pHArray[30];
   pHArray[i] = pH;
   delay(700);
   lcd.setCursor(0,0);
   lcd.print("PH: ");
   lcd.setCursor(4,0);
   lcd.print(pH);
   lcd.setCursor(0,1);
   lcd.print("Correcting PH");
   wdt_reset();
   //overflow sens
   if (digitalRead(ESENS) == HIGH) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("OVERFLOW");
    lcd.setCursor(0,1);
    lcd.print("ERROR");
    delay(6000);
    wdt_reset();
    digitalWrite(MIXRELAY,LOW);
    FlushTank();
   }

  }
 }
 digitalWrite(MIXRELAY,LOW);
 //check if tank is full
 //flush the tank if full or button is pressed
 if (digitalRead(HIGHSENS)==HIGH){
  if (checkPHRange(pH)) {
   FlushTank();
  }  
 }
 else if (digitalRead(BUTTON)==LOW){
  if (checkPHRange(pH)) {
   FlushTank();
  }  
 }
}

float getPH(){
 wdt_reset();
 boolean input_string_complete = false;                //have we received all the data from the PC
 boolean sensor_string_complete = false;               //have we received all the data from the Atlas Scientific product
 float pHval = 0.0;
 myserial.print("R \r");

 while (myserial.available()) {                  //if we see that the Atlas Scientific product has sent a character
  char inchar = (char)myserial.read();              //get the char we just received
  sensorstring += inchar;                           //add the char to the var called sensorstring
  if (inchar == '\r'){break;}                             //if the incoming character is a <CR>
 }
    
 pHval = sensorstring.toFloat(); 
  
 if(pHval == 0) {
  wdt_reset(); 
  delay(500);
  pHval = getPH();  
 }
 Serial.println(pHval);                               //send that string to the PC's serial monitor
 sensorstring = ""; 
 return pHval;
}

bool checkPHRange(float phValue){ 
 if (phValue < 9 and phValue > 6){ //within acceotable bounds
  Serial.println("pH is Good");
  return true;
 }
 else if(phValue < 6 or phValue > 9){ // out of bounds
  Serial.println("pH is out of bounds!");
  return false;
 }
}
