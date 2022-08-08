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
    public: double sAngleMin = 0.0f;
    public: double sAngleMax = 180.0f;
    public: int servoRegOffset = 4;

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
        //begin proper value writing for initialization
        uint8_t wakeup[1] = { WAKE_UP };
        reg_write(READ_READY_ON_DEVICE, wakeup, 1);
        swReset();
        //set up servo pwm settings for the board. 
        //SET servo pwm clock to 20ms this can vary by servo, but 20ms is typical for hobby servos
        uint8_t pwmbuf[1] = { PWM_VALUE_20ms_PULSE };
        reg_write(PWM_REG_ADDRESS, pwmbuf, 1);
        // block write outputs
        // disableWritesFromI2cSlave();
    };
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

    private: void disableWritesFromI2cSlave() {
        uint8_t writeOne[2] = { READ_ADDRESS_ONE, _u(0x00) };
        uint8_t writeTwo[2] = { READ_ADDRESS_TWO, _u(0x00) };
        uint8_t writeThree[2] = { READ_ADDRESS_THREE, _u(0x00) };
        uint8_t writeFour[2] = { READ_ADDRESS_FOUR, _u(0x00) };
        // writeDoubleBuffer(writeOne);
        // writeDoubleBuffer(writeTwo);
        // writeDoubleBuffer(writeThree);
        // writeDoubleBuffer(writeFour);

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
        uint8_t newServoMills[1] = { (angle * 2.2755) + 102 };

        reg_write(servoRegister, newServoMills, 1);

        // uint8_t lowBytePWM = newServoMills & 0xFF;
        // uint8_t highBytePWM = (newServoMills >> 8) & 0x01;
        // uint8_t highByteRegister = servoRegister + 1;



        // uint8_t lowByte[2] = { servoRegister, lowBytePWM };
        // uint8_t highByte[2] = {highByteRegister, highBytePWM};
        // uint8_t lowByte[2] = { servoRegister + 0x01, newServoMills & 0xFF };

        // writeDoubleBuffer(lowByte);
        // writeDoubleBuffer(lowByte);
        // writeDoubleBuffer(highByte);

    }

};

int main() {

    KitronikBoard kitBoard;

    kitBoard.servoSetAngle(5, 1);

    sleep_ms(500);

    kitBoard.servoSetAngle(50, 1);

    gpio_init(LED);

    gpio_set_dir(LED, GPIO_OUT);

    gpio_put(LED, 1);

    int a = 4;

    int b = 8;

    int c = a + b;

    cout << c << '\n';

    while (true) {
        cout << "Deadly Virus";
        gpio_put(LED, 1);
        sleep_ms(a * 100);
        gpio_put(LED, 0);
        sleep_ms(b * 100);
    }

    return 0;
}
