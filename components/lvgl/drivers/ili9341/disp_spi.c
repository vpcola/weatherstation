/**
 * @file disp_spi.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "disp_spi.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "freertos/task.h"
#include "shared_spi.h"
#include "../lvgl/lvgl.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void IRAM_ATTR spi_ready (spi_transaction_t *trans);

/**********************
 *  STATIC VARIABLES
 **********************/
static spi_device_handle_t spi_handle;
static volatile bool spi_trans_in_progress;
static volatile bool spi_color_sent;

/**********************
 *      MACROS
 **********************/


/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void disp_spi_init(void)
{

    esp_err_t ret;

    assert(shared_spi_is_initialized()); 

    // shared spi must be initialized first
    spi_device_interface_config_t devcfg={
            .clock_speed_hz=40*1000*1000,           //Clock out at 40 MHz
            .mode=0,                                //SPI mode 0
            .spics_io_num=DISP_SPI_CS,              //CS pin
            .queue_size=1,
            .pre_cb=NULL,
            .post_cb=spi_ready,			    // Callback when spi transaction is finished
    };


    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(shared_spi_get_host(), &devcfg, &spi_handle);
    assert(ret==ESP_OK);
}

void disp_spi_send_data(uint8_t * data, uint16_t length)
{
    if (length == 0) return;           //no need to send anything

    while(spi_trans_in_progress);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       	//Zero out the transaction
    t.length = length * 8;              //Length is in bytes, transaction length is in bits.
    t.tx_buffer = data;               	//Data
    spi_trans_in_progress = true;
    spi_color_sent = false;             //Mark the "lv_flush_ready" NOT needs to be called in "spi_ready"
    spi_device_queue_trans(spi_handle, &t, portMAX_DELAY);
}

void disp_spi_send_colors(uint8_t * data, uint16_t length)
{
    if (length == 0) return;           //no need to send anything

    while(spi_trans_in_progress);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));           //Zero out the transaction
    t.length = length * 8;              //Length is in bytes, transaction length is in bits.
    t.tx_buffer = data;                 //Data
    spi_trans_in_progress = true;
    spi_color_sent = true;              //Mark the "lv_flush_ready" needs to be called in "spi_ready"
    spi_device_queue_trans(spi_handle, &t, portMAX_DELAY);
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void IRAM_ATTR spi_ready (spi_transaction_t *trans)
{
    spi_trans_in_progress = false;

    if(spi_color_sent) lv_flush_ready();
}
