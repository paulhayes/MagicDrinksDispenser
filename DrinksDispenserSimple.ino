#include <EnableInterrupt.h>

#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))

#define DEBUG true

const byte DISPENSE_BTN_PIN_1 = 8;
const byte DISPENSE_BTN_PIN_2 = 11;
const byte MOTOR_PIN = 9;
const byte LIGHT_PIN = 10;

const int mFreq = 58;
unsigned int lFreq = 44;

unsigned long mW = 1000000 / mFreq;
unsigned long lW = 1000000 / lFreq;

unsigned long mD = round(0.4*mW);
unsigned long lD = 2000;

const byte ENCODER_BTN_PIN_1 = 4;
const byte ENCODER_BTN_PIN_2 = 6;

const byte ENCODER_PIN_1 = 3;
const byte ENCODER_PIN_2 = 2;
const byte ENCODER_PIN_VCC = 5;

long lastEncoderPos;
volatile long encoderPos = 0;

bool toggleOn;
int motorSpeed = 128;
int lastMotorSpeed = 0;

void setup() {
  // Set PWM to 30Hz
  //TCCR1B = (TCCR1B & 0b11111000) | 0x5;

  pinMode(LIGHT_PIN,OUTPUT);  
  pinMode(LIGHT_PIN,OUTPUT);  
  pinMode(DISPENSE_BTN_PIN_1,INPUT_PULLUP);
  pinMode(DISPENSE_BTN_PIN_2,OUTPUT);
  digitalWrite(DISPENSE_BTN_PIN_2, LOW);
  pinMode(ENCODER_BTN_PIN_1,INPUT_PULLUP);
  pinMode(ENCODER_BTN_PIN_2,OUTPUT);
  digitalWrite(ENCODER_BTN_PIN_2, LOW);

  pinMode(ENCODER_PIN_1, INPUT_PULLUP);
  pinMode(ENCODER_PIN_2, INPUT_PULLUP);  
  pinMode(ENCODER_PIN_VCC,OUTPUT);
  digitalWrite(ENCODER_PIN_VCC, LOW);
  enableInterrupt(ENCODER_PIN_1, EncAChange, CHANGE);
  enableInterrupt(ENCODER_PIN_2, EncBChange, CHANGE);

  #if DEBUG
  Serial.begin(9600); 
  #endif
}

void loop() {
  unsigned long t = micros();

  bool btnDown = isLow(DISPENSE_BTN_PIN_1);
  bool btn2 = isLow(ENCODER_BTN_PIN_1); 
  bool mOn = ((t%mW)<mD);
  bool lOn = (t%lW)<lD;
  int mS = 0;
  if( btnDown || toggleOn){
    mS=240;
  }
  if(btn2){
    toggleOn = !toggleOn;
  }

  if( encoderPos != lastEncoderPos ){
    long encoderPosTmp = encoderPos;
  
    //lFreq += encoderPosTmp - lastEncoderPos;
    //lW = 1000000 / lFreq;
    //lD = round(0.1*lW);
    lW += 10 * ( encoderPosTmp - lastEncoderPos );
    
    lastEncoderPos = encoderPosTmp;
    #if DEBUG
      Serial.print("wavelength ");
      Serial.println(lW);
    #endif
    //Serial.println(lFreq);
    //delay(500);
  }
  
  // Here's an idea. How about only settings this if it's changed
  if( lastMotorSpeed != mS ){
    analogWrite( MOTOR_PIN, mS);
    lastMotorSpeed = motorSpeed;
  } 
  
  digitalWrite( LIGHT_PIN, lOn);


  
  
}

volatile bool lastA;
volatile bool lastB;

void EncAChange(){
  bool A = isLow(ENCODER_PIN_1);
  bool B = isLow(ENCODER_PIN_2);

  if(lastA == A && lastB == B)
    return;
  if(lastA != A && lastB != B){
    return;
  }

  encoderPos += (A==B?1:-1);
  //Serial.print(A);
  //Serial.print(' ');
  //Serial.print(B);
  //Serial.println(' ');

  lastA = A;
  lastB = B;
}

void EncBChange(){
  bool A = isLow(ENCODER_PIN_1);
  bool B = isLow(ENCODER_PIN_2);

  if(lastA == A && lastB == B)
    return;
  if(lastA != A && lastB != B){
    return;
  }

  encoderPos += (B!=A?1:-1);
  //Serial.print(A);
  //Serial.print(' ');
  //Serial.print(B);
  //Serial.println(' ');
  lastA = A;
  lastB = B;

  
}
