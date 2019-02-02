#include "hdc1080.h"

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

/**
* @brief test code to read esp-i2c-slave
*        We need to fill the buffer of esp slave device, then master can read them out.
*
* _______________________________________________________________________________________
* | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
* --------|--------------------------|----------------------|--------------------|------|
*
*/
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t *data_rd, size_t size)
{
   if (size == 0) {
       return ESP_OK;
   }
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (slave_addr << 1) | READ_BIT, ACK_CHECK_EN);
   if (size > 1) {
       i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
   }
   i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   return ret;
}

/**
* @brief Test code to write esp-i2c-slave
*        Master device write data to slave(both esp32),
*        the data will be stored in slave buffer.
*        We can read them out from slave buffer.
*
* ___________________________________________________________________
* | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
* --------|---------------------------|----------------------|------|
*
*/
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t *data_wr, size_t size)
{
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
   i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   return ret;
}


esp_err_t hdc1080_init(i2c_port_t i2cnum)
{
   /* Write to reg addr 0x20 (config register), with value of 0x10 (14 bit adc resolution)
    */
    uint8_t bytes[2];

    bytes[0] = 0x20;
    bytes[1] = 0x10;

    return i2c_master_write_slave(i2cnum, HDC1080_ADDR, (uint8_t *) &bytes[0], 2); 
    
}

esp_err_t hdc1080_read_temperature(i2c_port_t i2cnum)
{
    esp_err_t err;
    int i;
    uint8_t data[4];
    
    data[0] = 0;

    // Initiate a read by writing to 0x0 to the slave
    err = i2c_master_write_slave(i2cnum, HDC1080_ADDR, (uint8_t *) &data[0], 1);

    // Add delays here
    vTaskDelay( 15 / portTICK_PERIOD_MS );

    // read 4 bytes
    err = i2c_master_read_slave(i2cnum, HDC1080_ADDR, (uint8_t *) &data[0], 4);

    // For now just display the data...
    for( i = 0; i < 4; i++)
    {
        printf("%0X ", data[i]);
    }
    printf("\n");

    return err;
}


