// Il reste à mettre en place la detection du 0 crossing point et du fire angle controle cf vidéo timer interrupt 

#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>


LiquidCrystal_I2C lcd(0x27, 16, 2);

/*    pin 3 => sortie de l'octocopler le tacho
      autre pin octocopler => Gnd
      pin 2 => zero crossing
      pin 4 et 5 codeur incrémentale
      pin 7 => controle triac
*/

int pin_mot = 7;  //pin control triac
int etatA;
int etatB;
int k = 250;      //pas du codeur
int a = 750;      //Vmin
int b = 15000;    //Vmax


float Kp = 1;
float Ki = 0;
float Kd = 0;


double timming;
float t;
bool overflowing =1;
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
double target = a;
PID myPID(&vitesse, &dimming, &target, Kp, Ki, Kd, DIRECT);
#define encoder0PinA  4 //pin encoder A
#define encoder0PinB  5 //pin encoder B

void setup() {
  lcd.init();
  lcd.backlight();

  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  etatA = digitalRead(encoder0PinA);
  etatB = digitalRead(encoder0PinB);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);

  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3),rmp,RISING);
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS12); //Prescaler 256
  TIMSK1 |= (1 << TOIE1);
}



ISR(TIMER1_OVF_vect) {
  overflowing = 1;
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

void rpm(){
  timming = TCNT1;
  TCNT1 = 0 ;
  overflowing = 0;
}


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


void encode() {
  if (digitalRead(encoder0PinA) != etatA) {
    etatA = digitalRead(encoder0PinA);
    if (etatA == HIGH) {
      if (etatB == HIGH) {
        target += k;
        target = constrain( target , a , b);
      }
      else {
        target -= k;
        target = constrain( target , a , b);
      }
    }
    else {
      if (etatB == HIGH) {
        target -= k;
        target = constrain( target , a , b);
      }
      else {
        target += k;
        target = constrain( target , a , b);
      }
    }
  }

  if (digitalRead(encoder0PinB) != etatB) {
    etatB = digitalRead(encoder0PinB);
    if (etatB == HIGH) {
      if (etatA == HIGH) {
        target -= k;
        target = constrain( target , a , b);
      }
      else {
        target += k;
        target = constrain( target , a , b);
      }
    }
    else {
      if (etatA == HIGH) {
        target += k;
        target = constrain( target , a , b);
      }
      else {
        target -= k;
        target = constrain( target , a , b);
      }
    }
  }
}
