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
using namespace std;

class KitronikBoard {

    private: uint I2CAddress;
    private: uint sda;
    private: uint scl;
    private: uint freq = 100000;

    public: KitronikBoard(uint address = 108, uint sdaPin = 8, uint sclPin = 9) {
        I2CAddress = address;
        sda = sdaPin;
        scl = sclPin;

        gpio_init(sda);
        gpio_init(scl);

        i2c_init(i2c_default, freq);
        gpio_set_function(sda, GPIO_FUNC_I2C);
        gpio_set_function(scl, GPIO_FUNC_I2C);
        gpio_pull_up(sda);
        gpio_pull_up(scl);



    };

    private: send_command(uint8_t cmd) {
        uint_t buf[2] = (0x80, cmd);
        // i2c_write_blocking(i2c_default, (I2CAddress & ))
    }

    private: reset_buffer() {

    }

};

int main() {
    const uint LED = PICO_DEFAULT_LED_PIN;

    gpio_init(LED);

    gpio_set_dir(LED, GPIO_OUT);

    stdio_init_all();

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
