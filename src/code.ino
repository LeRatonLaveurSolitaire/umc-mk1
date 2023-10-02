
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>


LiquidCrystal_I2C lcd(0x27, 16, 2);

/*    

Pinout Cheat Sheet

A4 -> SDA
A5 -> SCL
D2 -> Zero Crossing Detection 
D3 -> motor_speed
D4 -> fire_pin
D5 -> Encoder
D6 -> Encoder
D7 -> Encoder

*/

/* Pinout Variables*/

#define pin_ZCD 2
#define pin_speed 3
#define pin_fire 4

#define encoder0PinA  5 
#define encoder0PinB  7 
#define pin_middle_enc  6


/* Encoder variables*/

int enc_step = 250;      //pas du codeur
int enc_min = 0;      //Vmin
int enc_max = 11000;    //Vmax


/* PID Variables */

float Kp = 1;
float Ki = 0;
float Kd = 0;


/* Other variables */

double timming;
float t;
bool overflowing =1;

int etatA;
int etatB;


int mapping[101]={10000,9362,9096,8891,8718,8564,8424,8295,8174,8060,
                  7951,7847,7748,7651,7558,7468,7380,7294,7210,7128,
                  7048,6969,6891,6815,6740,6666,6593,6521,6450,6379,
                  6309,6240,6172,6104,6036,5969,5903,5837,5771,5706,
                  5640,5576,5511,5447,5382,5318,5254,5191,5127,5063,
                  5000,4936,4872,4808,4745,4681,4617,4552,4488,4423,
                  4359,4293,4228,4162,4096,4030,3963,3895,3827,3759,
                  3690,3620,3549,3478,3406,3333,3259,3184,3108,3030,
                  2951,2871,2789,2705,2619,2531,2441,2348,2251,2152,
                  2048,1939,1825,1704,1575,1435,1281,1108,903,637,0} // delai en microsec avant le fire pour sin de f = 100Hz

double vitesse;
double dimming = 1; //puissance apportée au moteur (entre 0 et 100) 0 = off 100 = max
double target = enc_min;

/* Declaring PID object */

PID myPID(&vitesse, &dimming, &target, Kp, Ki, Kd, DIRECT);




void setup() {

/* Configure I2C LCD screen*/

  lcd.init();
  lcd.backlight();


/* Configure encoder pin and variable */

  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(pin_middle_enc, OUTPUT);

  etatA = digitalRead(encoder0PinA);
  etatB = digitalRead(encoder0PinB);

/* Configure PID */

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);

  pinMode(pin_speed, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_speed),rmp,RISING);
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS12); //Prescaler 256
  TIMSK1 |= (1 << TOIE1);
}


// Case of overflowing

ISR(TIMER1_OVF_vect) {
  overflowing = 1; // if the timer reach it's limits, the speed is consider to be 0
}



void loop() {
  
  if (overflowing == 0){
    t = 120 / (timming/ 31250.00);
    vitesse = round(7.5*t);
  }
  else{
    vitesse = 0;
  }
  
  encode();
  myPID.Compute();
}



// speed reading function

void rpm(){
  timming = TCNT1; // Get timing for an 8th of a turn
  TCNT1 = 0 ;      // Reset counter to 0
  overflowing = 0; // Ensure the overflowing variable is 0
}

// LCD screen writing function

void affiche() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Vtgt :");
  lcd.print(target, DEC);
  lcd.print("RPM");
  lcd.setCursor(0, 1);
  lcd.print("Vmot :");
  lcd.print(vitesse, DEC);
  lcd.setCursor(11, 1);
  lcd.print("RPM");
}


// Encoder reading function setting the motor RMP target

void encode() {
  if (digitalRead(encoder0PinA) != etatA) {
    etatA = digitalRead(encoder0PinA);
    if (etatA == HIGH) {
      if (etatB == HIGH) {
        target += enc_step;
        target = constrain( target , enc_min , enc_max);
      }
      else {
        target -= enc_step;
        target = constrain( target , enc_min , enc_max);
      }
    }
    else {
      if (etatB == HIGH) {
        target -= enc_step;
        target = constrain( target , enc_min , enc_max);
      }
      else {
        target += enc_step;
        target = constrain( target , enc_min , enc_max);
      }
    }
  }

  if (digitalRead(encoder0PinB) != etatB) {
    etatB = digitalRead(encoder0PinB);
    if (etatB == HIGH) {
      if (etatA == HIGH) {
        target -= enc_step;
        target = constrain( target , enc_min , enc_max);
      }
      else {
        target += enc_step;
        target = constrain( target , enc_min , enc_max);
      }
    }
    else {
      if (etatA == HIGH) {
        target += enc_step;
        target = constrain( target , enc_min , enc_max);
      }
      else {
        target -= enc_step;
        target = constrain( target , enc_min , enc_max);
      }
    }
  }
}
