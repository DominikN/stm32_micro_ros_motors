#include "EncMotors.h"

// Motor 1

#define M1_PWM_TIM TIM10
#define M1_ENC_TIM TIM1

#define M1_PWM PF6
#define M1_ENC_A PE9
#define M1_ENC_B PE11
#define M1A_IN PE12
#define M1B_IN PE13
#define M1_ILIM PE10

// Motor 2

#define M2_PWM_TIM TIM11
#define M2_ENC_TIM TIM2

#define M2_PWM PF7
#define M2_ENC_A PA15
#define M2_ENC_B PB3
#define M2A_IN PG11
#define M2B_IN PG12
#define M2_ILIM PG15

HardwareTimer m1_pwm(M1_PWM_TIM);

EncMotors::EncMotors() {
  //   pinMode(M1_PWM, OUTPUT);
  pinMode(M1A_IN, OUTPUT_OPEN_DRAIN);
  pinMode(M1B_IN, OUTPUT_OPEN_DRAIN);
  pinMode(M1_ILIM, OUTPUT_OPEN_DRAIN);

  //   digitalWrite(M1_PWM, LOW);
  digitalWrite(M1A_IN, HIGH);
  digitalWrite(M1B_IN, LOW);
  digitalWrite(M1_ILIM, HIGH);

  //   m1_pwm.setPWM(1, M1_PWM, 1000, power);
  m1_pwm.setMode(1,TIMER_OUTPUT_COMPARE_PWM1,M1_PWM);
  m1_pwm.setOverflow(1000, HERTZ_FORMAT);
  m1_pwm.setCaptureCompare(1, 0, PERCENT_COMPARE_FORMAT);
  m1_pwm.resume();
}

void EncMotors::setPower(int power) {
  power = constrain(power, -100, 100);

  if (power >= 0) {
    digitalWrite(M1A_IN, HIGH);
    digitalWrite(M1B_IN, LOW);
  } else {
    power = -power;
    digitalWrite(M1A_IN, LOW);
    digitalWrite(M1B_IN, HIGH);
  }
  m1_pwm.setCaptureCompare(1, power, PERCENT_COMPARE_FORMAT);
}