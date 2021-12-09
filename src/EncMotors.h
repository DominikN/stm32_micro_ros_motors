#ifndef ENC_MOTORS_H
#define ENC_MOTORS_H
#include "Arduino.h"
#include "stm32f4xx_hal.h"



class EncMotors {

  public:
    EncMotors();

    uint8_t status();
    void setPower(int power);
    uint16_t getEnc();

//   private:
//     struct tcp_struct *_tcp_client;
};

#endif
