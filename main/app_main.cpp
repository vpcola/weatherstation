/* Deep sleep wake up example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp32/ulp.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"
#include "driver/i2c.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include "hdc1080.h"
#include "TheThingsNetwork.h"
#include "CayenneLPP.h"


#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)


#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */


// Pins and other resources
#define TTN_SPI_HOST      VSPI_HOST
#define TTN_SPI_DMA_CHAN  1
#define TTN_PIN_SPI_SCLK  CONFIG_RFM95W_SCLK 
#define TTN_PIN_SPI_MOSI  CONFIG_RFM95W_MOSI
#define TTN_PIN_SPI_MISO  CONFIG_RFM95W_MISO 
#define TTN_PIN_NSS       CONFIG_RFM95W_NSS
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       CONFIG_RFM95W_RST
#define TTN_PIN_DIO0      CONFIG_RFM95W_DIO0
#define TTN_PIN_DIO1      CONFIG_RFM95W_DIO1

// NOTE:
// Lorawan specification defines the maximum payload (num. of bytes)
// to send depending on effecitive modulation rate. At SF12, this translates
// to 51 bytes.
#define MAX_LORA_PAYLOAD 51

// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'make menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.

// Copy the below hex string from the "Device EUI" field
// on your device's overview page in the TTN console.
const char *devEui = CONFIG_TTN_DEV_EUI;

// Copy the below two lines from bottom of the same page
const char *appEui = CONFIG_TTN_APP_EUI;
const char *appKey = CONFIG_TTN_APP_KEY;

static TheThingsNetwork ttn;
static CayenneLPP       lpp(MAX_LORA_PAYLOAD);

const unsigned RX_WAIT_TIME = 10; // Wait 10 seconds before going deep sleep
static RTC_DATA_ATTR struct timeval sleep_enter_time;

#ifdef CONFIG_ENABLE_TOUCH_WAKEUP
#define TOUCH_THRESH_NO_USE 0
static void calibrate_touch_pad(touch_pad_t pad);
#endif

static esp_err_t i2c_master_init(void);
static void gpio_led_init(void);
static void lora_module_init(void);
static void configure_deep_sleep_params(void);

void send_ttn_message(void * pvParameter);
void receive_ttn_message(const uint8_t* message, size_t length, port_t port);

extern "C" void app_main()
{
    float currTemp, currHumid;
    struct timeval now;

    // Calculate how much time was spent in deep sleep mode
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    i2c_master_init();
    gpio_led_init();
    lora_module_init();

    if ( hdc1080_init(I2C_MASTER_NUM) == ESP_OK)
    {
        if ( hdc1080_read_temperature(I2C_MASTER_NUM, &currTemp, &currHumid) == ESP_OK)
        {
            printf("Current temperature = %.2f C, Relative Humidity = %.2f %%\n", 
                currTemp, currHumid);
        }

    }

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }
#ifdef CONFIG_ENABLE_TOUCH_WAKEUP
        case ESP_SLEEP_WAKEUP_TOUCHPAD: {
            printf("Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
            break;
        }
#endif // CONFIG_ENABLE_TOUCH_WAKEUP
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    printf("Sending lorawan message ...\n");
    // TODO: Send updates to TTN here


    // The delay here is a MUST!, this is to ensure that
    // we receive messages from the allocated ping slot 
    // (usually after we sent a message to TTN for a Lora 
    // class A device).
    printf("Waiting for RX packets ...\n");
    vTaskDelay( RX_WAIT_TIME * 1000 / portTICK_PERIOD_MS );

    // Configure items before going to deep sleep mode

    printf("Entering deep sleep\n");
    configure_deep_sleep_params();

    // Set the LED off.
    gpio_set_level((gpio_num_t) CONFIG_WAKEUP_LED, 1);

    // Record the time when we entered deep sleep
    gettimeofday(&sleep_enter_time, NULL);

    // Deep sleep on!
    esp_deep_sleep_start();
}

/**
* @brief i2c master initialization
*/
static esp_err_t i2c_master_init()
{
   i2c_port_t i2c_master_port = I2C_MASTER_NUM;
   i2c_config_t conf;
   conf.mode = I2C_MODE_MASTER;
   conf.sda_io_num = (gpio_num_t) I2C_MASTER_SDA_IO;
   conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
   conf.scl_io_num = (gpio_num_t) I2C_MASTER_SCL_IO;
   conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
   conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
   i2c_param_config( i2c_master_port, &conf);
   return i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void gpio_led_init()
{
    gpio_pad_select_gpio((gpio_num_t) CONFIG_WAKEUP_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction((gpio_num_t) CONFIG_WAKEUP_LED, GPIO_MODE_OUTPUT);
    /* Set LED on (output low) */
    gpio_set_level((gpio_num_t) CONFIG_WAKEUP_LED, 0);
}

static void lora_module_init(void)
{
    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);

    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 0;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

    // The below line can be commented after the first run as the data is saved in NVS
    // ttn.provision(devEui, appEui, appKey);

    ttn.onMessage(receive_ttn_message);
}

static void configure_deep_sleep_params(void)
{
    // Configure timer wakeup
    const int wakeup_time_sec = CONFIG_WAKEUP_INTERVAL;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    // Configure GPIO (external wakup pin)
    const int ext_wakeup_pin_1 = 27;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;

    printf("Enabling EXT1 wakeup on pins GPIO%d\n", ext_wakeup_pin_1);
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

#ifdef CONFIG_ENABLE_TOUCH_WAKEUP
    // Initialize touch pad peripheral.
    // The default fsm mode is software trigger mode.
    touch_pad_init();
    // If use touch pad wake up, should set touch sensor FSM mode at 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    // Set reference voltage for charging/discharging
    // In this case, the high reference valtage will be 2.4V - 1V = 1.4V
    // The low reference voltage will be 0.5
    // The larger the range, the larger the pulse count value.
    touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    //init RTC IO and mode for touch pad.
    touch_pad_config(TOUCH_PAD_NUM8, TOUCH_THRESH_NO_USE);
    touch_pad_config(TOUCH_PAD_NUM9, TOUCH_THRESH_NO_USE);
    calibrate_touch_pad(TOUCH_PAD_NUM8);
    calibrate_touch_pad(TOUCH_PAD_NUM9);
    printf("Enabling touch pad wakeup\n");
    esp_sleep_enable_touchpad_wakeup();

#endif // CONFIG_ENABLE_TOUCH_WAKEUP


    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);
}



#ifdef CONFIG_ENABLE_TOUCH_WAKEUP
static void calibrate_touch_pad(touch_pad_t pad)
{
    int avg = 0;
    const size_t calibration_count = 128;
    for (int i = 0; i < calibration_count; ++i) {
        uint16_t val;
        touch_pad_read(pad, &val);
        avg += val;
    }
    avg /= calibration_count;
    const int min_reading = 300;
    if (avg < min_reading) {
        printf("Touch pad #%d average reading is too low: %d (expecting at least %d). "
               "Not using for deep sleep wakeup.\n", pad, avg, min_reading);
        touch_pad_config(pad, 0);
    } else {
        int threshold = avg - 100;
        printf("Touch pad #%d average: %d, wakeup threshold set to %d.\n", pad, avg, threshold);
        touch_pad_config(pad, threshold);
    }
}
#endif // CONFIG_ENABLE_TOUCH_WAKEUP

void send_ttn_message(void* pvParameter)
{
    float currTemp = 0.0;
    float currHumid = 0.0;

    lpp.reset();

    if ( hdc1080_read_temperature(I2C_MASTER_NUM, &currTemp, &currHumid) == ESP_OK)
    {

        printf("Current temperature = %.2f C, Relative Humidity = %.2f %%\n", 
                currTemp, currHumid);
    }

    lpp.addTemperature(0, currTemp);
    lpp.addRelativeHumidity(1, currHumid);    


    printf("Sending message...\n");
    TTNResponseCode res = ttn.transmitMessage(lpp.getBuffer(), lpp.getSize());
    printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

}

void receive_ttn_message(const uint8_t* message, size_t length, port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}


