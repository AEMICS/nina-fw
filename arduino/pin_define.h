#ifndef pin_define_h
#define pin_define_h
// Define GPIO number functions
#define PIN_UART_TX  (1)
#define PIN_SPI_DRDY (2)
#define PIN_UART_RX  (3)
#define PIN_SPI_MISO (12) // GPIO12, MUST high during boot!! SD Voltage!!
#define PIN_SPI_MOSI (13)
#define PIN_SPI_SCLK (14)
#define PIN_SPI_CSN  (15)
#define PIN_LED_B    (21)
#define PIN_LED_R    (23)
#define PIN_LED_G    (33)

// Pin high/low during boot is dependant on bootloader. Seems that the ESP bl is used?
// https://docs.espressif.com/projects/esp-idf/en/release-v3.3/api-guides/bootloader.html
// sdk config states: 
// ESP TOOL 115200 baud
// extern SPI mem, 40MHz, 2MB
// no test firmware (no 'test' type in partition table)

// https://github.com/micropython/micropython/issues/8896#issuecomment-1190050321
// python ~/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyACM0 --baud 115200 --before no_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x0 NINA_W102-v1.5.0-Nano-RP2040-Connect.bin
#endif
