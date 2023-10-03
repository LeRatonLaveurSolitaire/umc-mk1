

#include <Wire.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>

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
  D8 -> GND
  D9 -> GND

*/

/* Pinout Variables*/

#define pin_ZCD 2
#define pin_speed 3
#define pin_fire 4
#define encoder0PinA  5
#define encoder0PinB  7
#define pin_middle_enc  6
#define pin_active_1 8
#define pin_active_2 9

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
bool overflowing = 1;

int etatA;
int etatB;

double speed = 0;
double dimming = 0;
double target = enc_min;
int prescaler_timer_1;

/* Declare LCD object */

LiquidCrystal_I2C lcd(0x27, 16, 2);

/* Declaring PID object */

PID myPID(&speed, &dimming, &target, Kp, Ki, Kd, DIRECT);



void setup() {

  /* Check if the arduino is plugged in the control board */

  pinMode(pin_active_1, INPUT_PULLUP);
  pinMode(pin_active_2, INPUT_PULLUP);


  if ( (!digitalRead(pin_active_1)) && (!digitalRead(pin_active_2))) {

    /* Configure I2C LCD screen */

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
    myPID.SetOutputLimits(1, 156);

    /* Setup Timer 1 for speed mesure (16 bits timer) */

    TCCR1A = 0;                           // Normal operation, disconnecting ouput compare trigger (OC1A and OC1B)
    TCCR1B = 0;                           // Reseting the reg
    TCCR1B |= (1 << CS12 | 1 << CS10 );   // setting prescaler to 1024
    TIMSK1 |= (1 << TOIE1);               // Disable I/O capture/compare match interutpions and enable overflow interruption
    prescaler_timer_1 = 1024;

    /* Setup interruption for speed mesure */

    pinMode(pin_speed, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin_speed), RPM, RISING);

    /* Setup Timer 2 for fire & ZCD (8 bits timer) */

    TCCR2A = 0;
    TCCR2B = 0;                                       // reseting timer config reg
    TCCR2B |= (1 << CS22 | 1 << CS21 | 1 << CS20 );   // setting prescaler at 1024
    TIMSK2 |= ( 1 << OCIE2A | 1 << OCIE2B);           // Enable interrupt on output compare A (for fire activation) and output compare B (for reset before 10ms)
    OCR2A = 155;                                      // fire time (between 0 for max power and 155 for min power)
    OCR2B = 156;                                      // trig at 9.984 ms ou of 10ms period

    /* Declaring hardware interruption for ZCD */

    pinMode(pin_ZCD, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin_ZCD), ZCD, RISING);

    /* Declare fire_pin as output */

    pinMode(pin_fire, OUTPUT);
  }

}


/* Setting interruptions */

ISR (TIMER1_OVF_vect) {   // Timer 1 overflow interrupt
  speed = 0;              // if the timer reach it's limits, the speed is consider to be 0
}

ISR (TIMER2_COMPA_vect) { // Timer 2 compare output reg A interrupt -> use to fire
  PORTD |= B00010000;     // Pull D4 (fire_pin) to high
}

ISR (TIMER2_COMPB_vect) { // Timer 2 compare output reg B interrupt -> use to rest fire pin before ZC
  PORTD &= B11101111;     // Pull D4 (fire_pin) to low
}


void loop() {
  encoder_read();         // look if the encoder have turned (change speed target)
  myPID.Compute();        // Compute the dimming using PID algorithme
  OCR2A = 156 - dimming;  // Set register to trigger at the right time
  lcd_print();            // print current speed and targer speed on display
  delay(100);
}



// speed reading function

void RPM() {
  timming = TCNT1;        // Get time taken to perform an 8th of a turn
  speed = round(120000000 / prescaler_timer_1 / timming); // RPM = 16 000 000(HZ) * 60(s) / 8 (HZ/RPM) / prescaler / TCNT1
  TCNT1 = 0 ;             // Reset counter to 0
}

// Zero crossing detection function

void ZCD() {
  TCNT2 = 0 ;           // reset counter to 0
  PORTD &= B11101111;   // Pull D4 (fire_pin) to low
}

// LCD screen writing function

void lcd_print() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("t_speed : ");
  lcd.print(target, DEC);
  lcd.setCursor(11, 0);
  lcd.print("RPM");
  lcd.setCursor(0, 1);
  lcd.print("c_speed : ");
  lcd.print(speed, DEC);
  lcd.setCursor(11, 1);
  lcd.print("RPM");
}


// Encoder reading function setting the motor RMP target

void encoder_read() {
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
