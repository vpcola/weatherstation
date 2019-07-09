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
#include "freertos/queue.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp32/ulp.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/touch_pad.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"
#include "driver/i2c.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include "nmea_parser.h"
#include "dustsensor_parser.h"
#include "hdc1080.h"
#include "TheThingsNetwork.h"
#include "CayenneLPP.h"


#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define TIME_ZONE (+8)   //Singapore Time
#define YEAR_BASE (2000) //date in GPS starts from 2000

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


#if defined(CONFIG_DUSTSENSOR_UART_PORT_1)
#define DUSTSENSOR_UART_PORT UART_NUM_1
#elif defined(CONFIG_DUSTSENSOR_UART_PORT_2)
#define DUSTSENSOR_UART_PORT UART_NUM_2
#else
#error Please select the UART Port used by the dust sensor!
#endif

#if defined(CONFIG_GPS_UART_PORT_1)
#define GPS_UART_PORT UART_NUM_1
#elif defined(CONFIG_GPS_UART_PORT_2)
#define GPS_UART_PORT UART_NUM_2
#else
#error Please select the UART Port used by the GPS receiver!
#endif

#define MAIN_TASK_PRIO  10

// NOTE:
// Lorawan specification defines the maximum payload (num. of bytes)
// to send depending on effecitive modulation rate. At SF12, this translates
// to 51 bytes.
#define MAX_LORA_PAYLOAD 51

typedef enum
{
    EV_GPS_UPDATE,
    EV_DUST_DATA_UPDATE,
    EV_LORA_MSG_RECV,
    EV_TEMP_HUMIDITY_UPDATE,
} main_task_event_t;

typedef struct 
{
    main_task_event_t event;
    union {
        gps_t gps_data;
        dustsensor_t dust_data;
        unsigned char lora_data[58];
    } data;
} main_task_message_t;

/*************************
 * Static Variables      *
 *                       *
 *************************/
// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'make menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.

// Copy the below hex string from the "Device EUI" field
// on your device's overview page in the TTN console.
const char *devEui = CONFIG_TTN_DEV_EUI;

// Copy the below two lines from bottom of the same page
const char *appEui = CONFIG_TTN_APP_EUI;
const char *appKey = CONFIG_TTN_APP_KEY;

static const char *TAG = "MAIN";

static TheThingsNetwork ttn;
static CayenneLPP       lpp(MAX_LORA_PAYLOAD);

// Variables stored on slow memory, they're retained
// from deep sleep to deep sleep
static RTC_DATA_ATTR struct timeval sleep_enter_time;
static RTC_DATA_ATTR int boot_count = 0;
static nmea_parser_handle_t nmea_hdl;
static dustsensor_parser_handle_t dustsensor_hdl;
static QueueHandle_t main_task_queue;
static SemaphoreHandle_t shutdown_sem;

/*************************
 * Static Functions      *
 *                       *
 *************************/

static void i2c_master_init(void);
static void i2c_master_deinit(void);

static void gpio_led_init(void);
static void gpio_led_deinit(void);

static void lora_module_init(void);
static void lora_module_deinit(void);

static void gps_init(void);
static void gps_deinit(void);

static void dustsensor_init(void);
static void dustsensor_deinit(void);

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void dustsensor_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void
*event_data);
static void main_task(void * arg);

/**************************
 * Forward declarations   *
 *                        *
 **************************/
void send_ttn_message(void);
void receive_ttn_message(const uint8_t* message, size_t length, port_t port);

/**************************
 * Main entry function    *
 *                        *
 **************************/
extern "C" void app_main()
{
	struct timeval now;

	ESP_LOGI(TAG, "Boot Count = %d, Initializing peripherals ...\r\n", boot_count);
	// Initialize TTN, do provisioning
	lora_module_init();

	i2c_master_init();
	gpio_led_init();

    gps_init();
    dustsensor_init();

	/* Set LED on (output low) */
	gpio_set_level((gpio_num_t) CONFIG_WAKEUP_LED, 1);


	
	gettimeofday(&now, NULL);
	int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

	switch (esp_sleep_get_wakeup_cause())
	{
		case ESP_SLEEP_WAKEUP_TIMER: {
										 ESP_LOGI(TAG,"Wake up from timer. Time spent in deep sleep: %d ms\r\n", sleep_time_ms);
										 break;
									 }
		default:
									 {
										 ESP_LOGI(TAG,"Wake up from other sources ....\r\n");
										 break;
									 }
	}

	if ( hdc1080_init(I2C_MASTER_NUM) != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to initialize HDC1080 sensor!\r\n");
	}

    ESP_LOGI(TAG, "Creating main message queue\r\n");
    /* Create the main task message queue */
    main_task_queue = xQueueCreate(10, sizeof(struct main_task_queue_t *));
    if ( main_task_queue == 0)
    {
        ESP_LOGE(TAG, "Failed in creating main task queue!");
    }
    
    /* Create the shutdown semaphore */
    shutdown_sem = xSemaphoreCreateBinary();

    /* Create the main task */
    xTaskCreate(main_task, "main_task", 4096, NULL, MAIN_TASK_PRIO, NULL);

    /* Wait for the shutdown_sem to be signalled */
    xSemaphoreTake(shutdown_sem, portMAX_DELAY);
    ESP_LOGI(TAG, "Shutdown initiated ....\r\n");
	++boot_count;

    /* Task Cleanup */
    vQueueDelete(main_task_queue);
    vSemaphoreDelete(shutdown_sem);

    /* Deep Sleep Wakeup Setup */
	ESP_LOGI(TAG, "Deep sleep set for %d seconds ... \r\n", CONFIG_WAKEUP_INTERVAL);
	const int wakeup_time_sec = CONFIG_WAKEUP_INTERVAL;
	ESP_LOGI(TAG, "Enabling timer wakeup, %ds\r\n", wakeup_time_sec);
	esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);    

    /* Driver de-init */
    gps_deinit();
    dustsensor_deinit();

    /* De-init GPIO Led */
    gpio_led_deinit();
    i2c_master_deinit();
    lora_module_deinit();

    /* Deep Sleep */
    ESP_LOGI(TAG, "Entering deep sleep\r\n");
    gettimeofday(&sleep_enter_time, NULL);
	esp_deep_sleep_start();
}

/**
* @brief i2c master initialization
*/
static void i2c_master_init()
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
   if ( i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0) != ESP_OK)
   {
       ESP_LOGE(TAG, "Failed to initialize I2C driver!\r\n");
   }
}

static void i2c_master_deinit(void)
{
    if (i2c_driver_delete(I2C_MASTER_NUM) != ESP_OK)
        ESP_LOGE(TAG, "Failed to uninstall I2C driver!\r\n");
}


static void gpio_led_init(void)
{
    gpio_pad_select_gpio((gpio_num_t) CONFIG_WAKEUP_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction((gpio_num_t) CONFIG_WAKEUP_LED, GPIO_MODE_OUTPUT);
}

static void gpio_led_deinit(void)
{
    gpio_set_level((gpio_num_t) CONFIG_WAKEUP_LED, 0);
    gpio_set_pull_mode((gpio_num_t) CONFIG_WAKEUP_LED, GPIO_FLOATING);
    gpio_set_direction((gpio_num_t) CONFIG_WAKEUP_LED, GPIO_MODE_INPUT); 
}

static void lora_module_init(void)
{
    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
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


	// Initialize the NVS (non-volatile storage) for saving and restoring the keys
	err = nvs_flash_init();
	ESP_ERROR_CHECK(err);


	if (boot_count == 0)
	{
		ESP_LOGI(TAG, "Starting TTN provisioning!\r\n");
		ttn.provision(devEui, appEui, appKey);
	}

    ttn.onMessage(receive_ttn_message);
}

static void lora_module_deinit(void)
{
    /* TODO: un initialize SPI */
}

static void gps_init(void)
{
	/* NMEA parser configuration */
	nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    /* Set UART and the RX pin */
    config.uart.uart_port = (uart_port_t) GPS_UART_PORT;
    config.uart.rx_pin = CONFIG_GPS_UART_RX_PIN;
	/* init NMEA parser library */
	nmea_hdl = nmea_parser_init(&config);
	/* register event handler for NMEA parser library */
	nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);
}

static void gps_deinit(void)
{
    if ( nmea_parser_deinit(nmea_hdl) != ESP_OK)
        ESP_LOGE(TAG, "GPS de-initialization error!\r\n");
}

static void dustsensor_init(void)
{
    /* NMEA parser configuration */
    dustsensor_parser_config_t config = DUSTSENSOR_PARSER_CONFIG_DEFAULT();
    config.uart.uart_port = (uart_port_t) DUSTSENSOR_UART_PORT;
    config.uart.rx_pin = CONFIG_DUSTSENSOR_UART_RX_PIN;
    /* init NMEA parser library */
    dustsensor_hdl = dustsensor_parser_init(&config);
    /* register event handler for NMEA parser library */
    dustsensor_parser_add_handler(dustsensor_hdl, dustsensor_event_handler, NULL);
}

static void dustsensor_deinit(void)
{
   if (dustsensor_parser_deinit( dustsensor_hdl ) != ESP_OK)
       ESP_LOGE(TAG, "Dustsensor de-initialization error!\r\n");
}

/**
* @brief GPS Event Handler
*
* @param event_handler_arg handler specific arguments
* @param event_base event base, here is fixed to ESP_NMEA_EVENT
* @param event_id event id
* @param event_data event specific arguments
*/
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
   gps_t *gps = NULL;
   switch (event_id) {
   case GPS_UPDATE:
       gps = (gps_t *)event_data;
       /* print information parsed from GPS statements */
       ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => \r\n"
		"\tlatitude   = %.05f°N\r\n"
		"\tlongtitude = %.05f°E\r\n"
		"\taltitude   = %.02fm\r\n"
		"\tspeed      = %fm/s",
		gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
		gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
		gps->latitude, gps->longitude, gps->altitude, gps->speed);
       break;
   case GPS_UNKNOWN:
       /* print unknown statements */
       ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
       break;
   default:
       break;
   }
}

static void dustsensor_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    dustsensor_t *sensor = NULL;
    switch (event_id) {
    case SENSOR_UPDATE:
        sensor = (dustsensor_t *)event_data;
        // Handle the data from the sensor here
        ESP_LOGI(TAG, "Concentration Unit (Standard):\r\n"
                "\tPM1.0 = %d ug/cum, PM2.5 = %d ug/cum, PM10 = %d ug/cum\r\n", 
                sensor->pm1,
                sensor->pm25,
                sensor->pm10);
        ESP_LOGI(TAG, "Concentration Unit (Environmental):\r\n"
                "\tPM1.0 = %d ug/cum, PM2.5 = %d ug/cum, PM10 = %d ug/cum\r\n", 
                sensor->pm1_atmospheric,
                sensor->pm25_atmospheric,
                sensor->pm10_atmospheric);

        break;
    case SENSOR_UNKNOWN:
        /* print unknown statements */
        ESP_LOGE(TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

static void main_task(void * arg)
{
    main_task_message_t * msg;
	ESP_LOGI(TAG,"Joining TTN ...\r\n");

    if (ttn.join())
    {
        ESP_LOGI(TAG, "Join accepted, main event loop started!\r\n");
        while(1)
        {
            xQueueReceive(main_task_queue, &( msg ), portMAX_DELAY);

            switch(msg->event)
            {
                case EV_GPS_UPDATE:
                    break;
                case EV_DUST_DATA_UPDATE:
                    break;
                case EV_LORA_MSG_RECV:
                    break;
                case EV_TEMP_HUMIDITY_UPDATE:
                    break;
                default:
                    ESP_LOGE(TAG, "Unknown event type!\r\n");
            }

            /* TODO: Determine if we need to shutdown! */
            send_ttn_message();
            break;
        }
    }else
    {
        ESP_LOGE(TAG, "Failed to join TTN, restarting!\r\n");
    }

    /* Done with main task, resume shutdown */
    xSemaphoreGive(shutdown_sem);
    /* Remove this task */
    vTaskDelete(NULL);
}

void send_ttn_message()
{
    float currTemp = 0.0;
    float currHumid = 0.0;
    float waterLevel = 0.0;

    lpp.reset();

    printf("Waiting for next available transmit event ...\n");
    if ( hdc1080_read_temperature(I2C_MASTER_NUM, &currTemp, &currHumid) == ESP_OK)
    {

        printf("Current temperature = %.2f C, Relative Humidity = %.2f %%\n", 
                currTemp, currHumid);
    }

    // for now random values for water level
    waterLevel = rand() % 10 + 1;
    lpp.addTemperature(0, currTemp);
    lpp.addRelativeHumidity(1, currHumid);    
    lpp.addAnalogInput(2, waterLevel); 


    TTNResponseCode res = ttn.transmitMessage(lpp.getBuffer(), lpp.getSize());
    printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

}

void receive_ttn_message(const uint8_t* message, size_t length, port_t port)
{
    ESP_LOGI(TAG, "Message of %d bytes received on port %d\r\n", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}


