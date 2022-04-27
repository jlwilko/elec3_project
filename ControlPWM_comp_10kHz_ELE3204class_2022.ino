// This code is to generate a pair of complemenatry PWM pulse using Timer 1 registers. The register ICR1 is for frequency,  currently of 100  for 10kHz.
//The duty is determined by the register OCR1A and OCR1B. The two PWM signals are out of pin9 and pin10, with duty of 30% and 70%. 
//Check details on Timer 1 registers from the datasheet of ATMega328 page113.
#define PWMA 9 // yellow
#define PWMB 10 // orange
#define ENA 2 // yellow
#define ENB 3 // white

#define MODEBUTTON 7
#define EMERGENCY_STOP 6

volatile long encoderPos = 0;

volatile int motor_dir = 1;

enum ControlMode{
  OPENLOOP,
  CLOSEDLOOP,
  STOPPED
};

ControlMode controlmode = ControlMode::OPENLOOP;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("working");
  
  pinMode(ENB, INPUT); // encoder B pin 
  pinMode(ENA, INPUT);// encode A pin
  digitalWrite(ENA, HIGH); // enable pullup
  digitalWrite(ENB, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENA), countPulses, RISING);
  
  
  pinMode(PWMA, OUTPUT);          // output PWMA to Q1
  pinMode(PWMB, OUTPUT);          // output PWMB to Q2

  analogWrite(PWMA, 0);          // let PWMA=0
  analogWrite(PWMB, 0);          // let PWMB=0

  pinMode(EMERGENCY_STOP, INPUT_PULLUP);
  pinMode(MODEBUTTON, INPUT_PULLUP);

  TCCR1A = 0; // clear Timer1 control register TCCR1A & B
  TCCR1B = 0;
  TCNT1 = 0; // clear Timer1 counter register

  TCCR1B |= _BV(CS11); //set prescaler=8 by lettin the bit value of CS11=1 in register TCCR1B, so the clock frequency=16MHz/8=2MHz
  ICR1 = 100;//  phase correct PWM. PWM frequency determined by counting up 0-100 and counting down 100-0 in the input compare register (ICR1), so freq=200*0.5us=10kHz 
}

void loop() {
  // put your main code here, to run repeatedly:
  static int goal_velocity = 0;
  static int duty_cycle = 50;

  
  if (Serial.available() > 0){
    goal_velocity = Serial.parseInt();
    Serial.println(goal_velocity);
    while (Serial.available()){
      Serial.read();
    }
  }

  controlmode = readControlModeButton();
  if (digitalRead(EMERGENCY_STOP)){
    controlmode = ControlMode::STOPPED;
  }
  
  switch (controlmode){
    
  case ControlMode::OPENLOOP:
    duty_cycle = openLoopControl(duty_cycle, goal_velocity);
    break;

  case ControlMode::CLOSEDLOOP:
    duty_cycle = closedLoopControl(duty_cycle, goal_velocity);
    break;

  case ControlMode::STOPPED:
    duty_cycle = 50;
    break;

  default:
    Serial.println("Error: Illegal Mode");
  }


  PWM(duty_cycle);

 
}

int closedLoopControl(int duty_cycle, int goal_velocity){
  static unsigned long last_updated = millis();
  static unsigned long oldTime = millis();
  
  int dt = 100;
  static int oldPos = 0;

  

  if (millis() - last_updated > dt){
    Serial.print("Doing closed loop - ");    

    int encoder_ticks = encoderPos - oldPos;
    unsigned long time_elapsed = millis() - last_updated;
    
    oldPos = encoderPos;

    int curr_velocity = calc_velocity(encoder_ticks, time_elapsed)*motor_dir;
    
    if (curr_velocity < goal_velocity){
      duty_cycle++;
    }
    else if (curr_velocity > goal_velocity){
      duty_cycle--;
    }
    Serial.print("Velocity = ");
    Serial.print(curr_velocity);
    Serial.print(" Duty Cycle = ");
    Serial.print(duty_cycle);
    Serial.print(" Goal Velocity = ");
    Serial.println(goal_velocity);    
    last_updated = millis();
    return constrain(duty_cycle, 0, 100);
  }
  else {
    return duty_cycle;
  }

}

int openLoopControl(int duty_cycle, int goal_velocity){
  static unsigned long last_updated = millis();
  static int oldPos = 0; 
  int dt = 300;

  if (millis() - last_updated > dt){
    Serial.print("Doing open loop - ");

    int reqd_duty = map(goal_velocity, -97, 97, 0, 100);

    int encoder_ticks = encoderPos - oldPos;
    oldPos = encoderPos;
    unsigned long time_elapsed = millis() - last_updated;

    int curr_velocity = calc_velocity(encoder_ticks, time_elapsed)*motor_dir;

    if (reqd_duty < duty_cycle){
      duty_cycle--;
    }
    else if (reqd_duty > duty_cycle){
      duty_cycle++;
    }
    Serial.print("Velocity = ");
    Serial.print(curr_velocity);
    Serial.print(" Duty Cycle = ");
    Serial.print(duty_cycle);
    Serial.print(" Goal Velocity = ");
    Serial.println(goal_velocity);
    last_updated = millis();
    return constrain(duty_cycle, 0, 100);
  }
  else {
    return duty_cycle;
  }
}

ControlMode readControlModeButton(void){
  if (digitalRead(MODEBUTTON) == HIGH){
    return ControlMode::OPENLOOP;
  }
  else {
    return ControlMode::CLOSEDLOOP;
  }
}

long calc_velocity(int d_pos, unsigned long d_t){
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
