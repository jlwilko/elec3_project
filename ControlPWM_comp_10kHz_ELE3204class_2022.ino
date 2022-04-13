// This code is to generate a pair of complemenatry PWM pulse using Timer 1 registers. The register ICR1 is for frequency,  currently of 100  for 10kHz.
//The duty is determined by the register OCR1A and OCR1B. The two PWM signals are out of pin9 and pin10, with duty of 30% and 70%. 
//Check details on Timer 1 registers from the datasheet of ATMega328 page113.

#define PWMA 9
#define PWMB 10
#define ENA 2
#define ENB 3

#define SYS_V 5
#define ADC_RES 1023

int duty_cycle;
volatile long encoderPos = 0;
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
long vel; 


//------------------------- setup routine ----------------------------//
void setup() {
  duty_cycle = 60;

  Serial.begin(9600);
  Serial.println("Working!");

  pinMode(ENB, INPUT); // encoder B pin 
  pinMode(ENA, INPUT);// encode A pin
  digitalWrite(ENA, HIGH); // enable pullup
  digitalWrite(ENB, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENA), countPulses, RISING);
   
  
  pinMode(PWMA, OUTPUT);          // output PWMA to Q1
  pinMode(PWMB, OUTPUT);          // output PWMB to Q2

  analogWrite(PWMA, 0);          // let PWMA=0
  analogWrite(PWMB, 0);          // let PWMB=0

  TCCR1A = 0; // clear Timer1 control register TCCR1A & B
  TCCR1B = 0;
  TCNT1 = 0; // clear Timer1 counter register

  TCCR1B |= _BV(CS11); //set prescaler=8 by lettin the bit value of CS11=1 in register TCCR1B, so the clock frequency=16MHz/8=2MHz
  ICR1 = 100;//  phase correct PWM. PWM frequency determined by counting up 0-100 and counting down 100-0 in the input compare register (ICR1), so freq=200*0.5us=10kHz 
}

//------------------------- main loop ----------------------------//
void loop() 
{
  if (Serial.available() > 0){
    duty_cycle = Serial.parseInt();
    Serial.println(duty_cycle);
    while (Serial.available()){
      Serial.read();
    }
  }
  PWM(duty_cycle);
  delay(500);
  newposition = encoderPos;
  newtime = millis();

  vel = (newposition - oldposition) * 60000/(newtime - oldtime)/12/99;
  Serial.print("speed = ");
  Serial.println(vel);
  oldposition = newposition;
  oldtime = newtime;
  Serial.println(encoderPos);
}

//------------------------- subroutine PWM generate complementary PWM from OCR1A and OCR1B ----------------------------//
void PWM(int pwm)
{
  int temp = (int)pwm;
  temp = constrain(temp,1,99);

  OCR1A = temp; //duty of PWM for pin9 is from output compare register A 
  TCCR1A |= _BV(COM1A1) | _BV(COM1A0); //set output to low level

  OCR1B = temp;//duty of PWM for pin10 is from output compare register B
  TCCR1A |= _BV(COM1B1); //set output to high level

  TCCR1B |= _BV(WGM13); //
  TCCR1A |= _BV(WGM11); //Set ICR1 phas correct mode
}

void countPulses(){
  encoderPos++;
}
