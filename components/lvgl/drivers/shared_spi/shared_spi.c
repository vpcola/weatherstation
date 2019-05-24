#include "shared_spi.h"

#define MISO_PIN CONFIG_SHARED_SPI_MISO
#define MOSI_PIN CONFIG_SHARED_SPI_MOSI
#define SCK_PIN	 CONFIG_SHARED_SPI_SCK
#define USE_DMA	 CONFIG_SHARED_SPI_USE_DMA
#define SPI_TRANSFER_SIZE CONFIG_SHARED_SPI_TRANS_SIZE

static int spi_transfer_size = SPI_TRANSFER_SIZE;

#if defined(CONFIG_VSPI_HOST)
static spi_host_device_t spi_host = VSPI_HOST; 
#else
static spi_host_device_t spi_host = HSPI_HOST;
#endif

static bool is_initialized = false;

void shared_spi_set_transfer_size(size_t trans_size)
{
	spi_transfer_size = trans_size;
}

spi_host_device_t shared_spi_get_host(void)
{
	return spi_host;
}

bool shared_spi_is_initialized(void)
{
	return is_initialized;
}

void shared_spi_init(void) 
{
	esp_err_t ret;

	spi_bus_config_t buscfg={
		.miso_io_num = MISO_PIN,
		.mosi_io_num = MOSI_PIN,
		.sclk_io_num = SCK_PIN,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz = spi_transfer_size,
	};


	//Initialize the SPI bus
#if defined(CONFIG_SHARED_SPI_USE_DMA)
	ret=spi_bus_initialize(spi_host, &buscfg, 1);
#else
	ret=spi_bus_initialize(spi_host, &buscfg, 0);
#endif

	assert(ret==ESP_OK);

	is_initialized = true;

}

