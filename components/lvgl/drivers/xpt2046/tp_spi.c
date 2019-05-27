/**
 * @file tp_spi.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "tp_spi.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <string.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void IRAM_ATTR touch_spi_ready (spi_transaction_t *trans);

/**********************
 *  STATIC VARIABLES
 **********************/
static spi_device_handle_t spi;
static volatile bool spi_trans_in_progress;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void tp_spi_init(spi_host_device_t spihost)
{

	esp_err_t ret;


	spi_device_interface_config_t devcfg={
		.clock_speed_hz = 10*1000*1000,           //Clock out at 80 MHz
		.mode = 0,                                //SPI mode 0
		.spics_io_num = -1,              //CS pin
		.queue_size = 1,
		.pre_cb = NULL,
		.post_cb = touch_spi_ready,
	};

	//Attach the LCD to the SPI bus
	ret=spi_bus_add_device(spihost, &devcfg, &spi);
	assert(ret==ESP_OK);
}

uint8_t tp_spi_xchg(uint8_t data_send)
{
	uint8_t data_rec = 0;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       	//Zero out the transaction
	t.length = 8;              //Length is in bytes, transaction length is in bits.
	t.tx_buffer = &data_send;            //Data
	t.rx_buffer = &data_rec;
	esp_err_t ret;

	// Set a variable to watch
	spi_trans_in_progress = true;
	spi_device_queue_trans(spi, &t, portMAX_DELAY);

	/* We can either call spi_device_get_trans_result()
	 * or check for a variable that is being set when
	 * a DMA or SPI completes the transaction
	 **/
	/*
	   spi_transaction_t * rt;
	   spi_device_get_trans_result(spi, &rt, portMAX_DELAY);
	   */
	while(spi_trans_in_progress == true);

	return data_rec;
}


/**********************
 *   STATIC FUNCTIONS
 **********************/
static void IRAM_ATTR touch_spi_ready (spi_transaction_t *trans)
{
    spi_trans_in_progress = false;
}

