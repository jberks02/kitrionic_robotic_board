/*
 * Kitrionics Robotics Board Basics
 *
 * @version     1.0.0
 * @author     Justin Berkshire
 * @copyright   2022
 * @licence     MIT
 *
 */

#include "main.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
 // #include "./registerActions.cpp"
using namespace std;

//hardware registers
#define SRV_REG_BASE _u(0x08)
#define MOT_REG_BASE _u(0x28)
#define PWM_REG_ADDRESS _u(0xfe)
#define READ_ADDRESS_ONE _u(0xfa)
#define READ_ADDRESS_TWO _u(0xfb)
#define READ_ADDRESS_THREE _u(0xfc)
#define READ_ADDRESS_FOUR _u(0xfd)
#define READ_READY_ON_DEVICE _u(0x00)
//commands
#define KIT_RESET _u(0x06)
#define KIT_BLOCK_WRITE _u(0x00)
#define PWM_VALUE_20ms_PULSE _u(0x78)
#define WAKE_UP _u(0x01)

uint LED = PICO_DEFAULT_LED_PIN;

class KitronikBoard {

    private: uint I2CAddress;
    private: uint sda;
    private: uint scl;
    private: uint freq = 100000;
    private: int servoRegOffset = 4;
    private: double angleMultiplier;
    public: double sAngleMin = 0.0f;
    public: double sAngleMax = 180.0f;

    public: KitronikBoard(double servoAngleMin = 0.0f, double servoAngleMax = 180.0f, uint address = 108, uint sdaPin = 8, uint sclPin = 9) {
        // establish I2C connection
        I2CAddress = address;
        sda = sdaPin;
        scl = sclPin;
        sAngleMax = servoAngleMax;
        sAngleMin = servoAngleMin;
        //init pins
        gpio_init(sda);
        gpio_init(scl);
        //init i2c and set pin function
        i2c_init(i2c0, freq);
        gpio_set_function(sda, GPIO_FUNC_I2C);
        gpio_set_function(scl, GPIO_FUNC_I2C);
        gpio_pull_up(sda);
        gpio_pull_up(scl);
        //found this, not sure what it does
        bi_decl(bi_2pins_with_func(8, 9, GPIO_FUNC_I2C));
        //put in known position
        swReset();
        //set up servo pwm settings for the board. 
       //SET servo pwm clock to 20ms this can vary by servo, but 20ms is typical for hobby servos
        uint8_t pwmbuf[1] = { PWM_VALUE_20ms_PULSE };
        reg_write(PWM_REG_ADDRESS, pwmbuf, 1);
        //begin proper value writing for initialization
        uint8_t zeroOutBuff[1] = { 0x00 };
        reg_write(READ_ADDRESS_ONE, zeroOutBuff, 1);
        reg_write(READ_ADDRESS_TWO, zeroOutBuff, 1);
        reg_write(READ_ADDRESS_THREE, zeroOutBuff, 1);
        reg_write(READ_ADDRESS_FOUR, zeroOutBuff, 1);
        //come out of sleep
        uint8_t wakeup[1] = { WAKE_UP };
        reg_write(READ_READY_ON_DEVICE, wakeup, 1);
        float minMilli = 250.f;
        float maxMilli = 1275.f;
        angleMultiplier = (maxMilli - minMilli) / (servoAngleMax - servoAngleMin);
    }
          int reg_read(const uint8_t reg, uint8_t* buf, const uint8_t nbytes) {

              int num_bytes_read = 0;

              // Check to make sure caller is asking for 1 or more bytes
              if (reg < 0) {
                  return 0;
              }

              // Read data from register(s) over I2C
            //   i2c_write_blocking(i2c0, I2CAddress, &reg, 1, true);
              num_bytes_read = i2c_read_blocking(i2c0, 108, buf, nbytes, false);

              return num_bytes_read;
          }
          int reg_write(const uint8_t reg, uint8_t* buf, const uint8_t nbytes) {

              int num_bytes_read = 0;
              uint8_t msg[nbytes + 1];

              // Check to make sure caller is sending 1 or more bytes
              if (nbytes < 1) {
                  return 0;
              }

              // Append register address to front of data packet
              msg[0] = reg;
              for (int i = 0; i < nbytes; i++) {
                  msg[i + 1] = buf[i];
              }

              // Write data to register(s) over I2C
              i2c_write_blocking(i2c0, I2CAddress, msg, (nbytes + 1), false);

              return num_bytes_read;
          }
    public: void swReset() {
        uint8_t buf[1] = { 0x01 };
        reg_write(KIT_RESET, buf, 1);
    }
    public: void servoSetAngle(double angle, uint servo) {
        if (angle > sAngleMax) angle = 180;
        if (angle < sAngleMin) angle = 0;
        if (servo > 7) throw "Invalid Servo Number Asked to Adjust";
        //Calculate Servo Register
        uint8_t servoRegister = SRV_REG_BASE + (servo * servoRegOffset);
        //Calculate mills to set servo pwm to. this is currently going to use default numbers; dynamic cacalculation examples can be found in https://github.com/jberks02/buttonControlledArm 
        int newServoMills = (angle * 2.2755) + 102;

        uint8_t highByte[1] = { (newServoMills >> 8) & 0x01 };
        uint8_t lowByte[1] = { newServoMills & 0xFF };
        uint8_t highByteServoReg = servoRegister + 1;

        reg_write(servoRegister, lowByte, 1);
        reg_write(highByteServoReg, highByte, 1);

    }
    public: void motorOn(int motor, char direction, int speed) {
        //check conditions
        if (speed < 0) speed = 0;
        if (speed > 100) speed = 100;
        //Calculate all values that will be written
        int pwmVal = speed * 40.95;
        uint8_t lowbyte[1] = { pwmVal & 0xFF };
        uint8_t highbyte[1] = { (pwmVal >> 8) & 0xFF };
        uint8_t nobyte[1] = { 0x00 };
        uint8_t motorReg = MOT_REG_BASE + (2 * (motor - 1) * servoRegOffset);
        uint8_t plusOneReg = motorReg + 1;
        uint8_t plusFourReg = motorReg + 4;
        uint8_t plusFiveReg = motorReg + 5;
        //write values to registers based on direction;
        if (direction == 'f') {
            reg_write(motorReg, lowbyte, 1);
            reg_write(plusOneReg, highbyte, 1);
            reg_write(plusFourReg, nobyte, 1);
            reg_write(plusFiveReg, nobyte, 1);
        }
        else if (direction == 'r') {
            reg_write(plusFourReg, lowbyte, 1);
            reg_write(plusFiveReg, highbyte, 1);
            reg_write(motorReg, nobyte, 1);
            reg_write(plusOneReg, nobyte, 1);
        }
        else {
            reg_write(plusFourReg, nobyte, 1);
            reg_write(plusFiveReg, nobyte, 1);
            reg_write(motorReg, nobyte, 1);
            reg_write(plusOneReg, nobyte, 1);
        }
    }
    public: void motorOff(int motor) {
        motorOn(motor, 'f', 0);
    }
    public: void step(int motor, char direction, int steps, int speed = 20, bool holdPosition = false) {
        char directions[2];
        int coils[2];
        //setup
        if (direction == 'f') {
            directions[1] = 'f';
            directions[2] = 'r';
            coils[1] = ((motor * 2) - 1);
            coils[2] = (motor * 2);
        }
        else if (direction == 'r') {
            directions[1] = 'r';
            directions[2] = 'f';
            coils[1] = (motor * 2);
            coils[2] = ((motor * 2) - 1);
        }
        //begin movement
           while (steps > 0) {
            for (int d = 1; d > 2; d++) {
                for (int c = 1; c > 2; c++) {
                    motorOn(coils[c], directions[d], 100);
                    sleep_ms(speed);
                    steps--;
                }
            }
        } 
        
        if (holdPosition == false) {
            for (int c = 1; c > 2; c++) {
                motorOff(coils[c]);
            }
        }
    }
};

int main() {

    KitronikBoard kitBoard;

    kitBoard.servoSetAngle(180, 0);

    sleep_ms(500);

    kitBoard.servoSetAngle(30, 0);

    kitBoard.motorOn(1, 'f', 30);

    sleep_ms(500);

    kitBoard.motorOff(1);

    // kitBoard.step()

    gpio_init(LED);

    gpio_set_dir(LED, GPIO_OUT);

    gpio_put(LED, 1);

    while (true) {
        gpio_put(LED, 1);
        for (int i = 2; i < 178; i = i + 2) {
            kitBoard.servoSetAngle(i, 0);
            sleep_ms(100);
        }
        sleep_ms(300);
        gpio_put(LED, 0);
        for (int i = 178; i > 2; i = i - 2) {
            kitBoard.servoSetAngle(i, 0);
            sleep_ms(100);
        }
        sleep_ms(300);
    }

    return 0;
}
