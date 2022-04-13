// This code is to generate a pair of complemenatry PWM pulse using Timer 1 registers. The register ICR1 is for frequency,  currently of 100  for 10kHz.
//The duty is determined by the register OCR1A and OCR1B. The two PWM signals are out of pin9 and pin10, with duty of 30% and 70%. 
//Check details on Timer 1 registers from the datasheet of ATMega328 page113.

#define PWMA 9
#define PWMB 10
#define ENA 2
#define ENB 3

int duty_cycle;
volatile long encoderPos = 0;
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
long vel; 

int state; // 0 for open loop, 1 for closed loop
int motor_dir; 

// Closed Loop Global variables
int goal_velocity; // in rpm

//------------------------- setup routine ----------------------------//
void setup() {
  duty_cycle = 50;
  state = 1; // start in closed loop mode
  goal_velocity = 80;
  
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
    goal_velocity = Serial.parseInt();
    Serial.println(goal_velocity);
    while (Serial.available()){
      Serial.read();
    }
  }

  if (state == 1){
    // closed loop control mode
    duty_cycle = ClosedLoopControl(duty_cycle, vel, goal_velocity);
    
  }
  PWM(duty_cycle);
  delay(250);

  newposition = encoderPos;
  newtime = millis();
  
  int d_pos = newposition - oldposition;
  int d_t = newtime - oldtime;

  vel = calc_velocity(d_pos, d_t)*motor_dir;

  Serial.print("speed = ");
  Serial.print(vel);
  Serial.print(", duty_cycle = ");
  Serial.print(duty_cycle);
  Serial.print(", goal_speed = ");
  Serial.println(goal_velocity);
  
  
  oldposition = newposition;
  oldtime = newtime;
}

int ClosedLoopControl(int duty_cycle, int curr_velocity, int goal_velocity){
  if (curr_velocity < goal_velocity){
    duty_cycle++;
  }
  else if (curr_velocity > goal_velocity){
    duty_cycle--;
  }
  
  return constrain(duty_cycle, 0, 100);
}



long calc_velocity(int d_pos, int d_t){
  int gear_ratio = 99;
  int tick_per_rev = 12;
  return d_pos * 60000 / d_t / tick_per_rev / gear_ratio;
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
  if (digitalRead(ENB) == LOW){
    motor_dir = 1;
  }
  else {
    motor_dir = -1;
  }
  encoderPos++;
}
