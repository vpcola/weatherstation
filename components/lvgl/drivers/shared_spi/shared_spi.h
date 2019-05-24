#ifndef _SHARED_SPI_H_
#define _SHARED_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

	#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

void shared_spi_set_transfer_size(size_t trans_size);
void shared_spi_init(void); 
bool shared_spi_is_initialized(void);
spi_host_device_t shared_spi_get_host(void);

#ifdef __cplusplus
}
#endif

#endif
