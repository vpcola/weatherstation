#include "dustsensor.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 10

#define DUSTSENSOR_LED 		CONFIG_DUSTSENSOR_LED_PIN

#if defined(CONFIG_DUSTSENSOR_USE_ADC1)
#define DUSTSENSOR_UNIT		ADC_UNIT_1
#elif defined(CONFIG_DUSTSENSOR_USE_ADC2)
#define DUSTSENSOR_UNIT		ADC_UNIT_2
#endif


// Set the typical output voltage in Volts when there is zero dust.
static float Voc = 0.6;

// Use the typical sensitivity in units of V per 100ug/m3.
const float K = 0.5;

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_0;     //GPIO36 if ADC1, GPIO4 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;				//USE ADC1

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void dustsensor_init()
{
	printf("Initializing dust sensor ADC\n");

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);	

	// Initialize dust sensor led
    gpio_pad_select_gpio((gpio_num_t) DUSTSENSOR_LED);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction((gpio_num_t) DUSTSENSOR_LED, GPIO_MODE_OUTPUT);
	
}

float dustsensor_read()
{
	 uint32_t adc_reading = 0;

	// Turn on led by setting it to LOW
	gpio_set_level((gpio_num_t) DUSTSENSOR_LED, 0);

	// Wait for .28ms before taking a reading
	usleep( 280 ); 
	
	// Take a reading on the analog pin, multisampling
	// mitigates the effect of noise
	for (int i = 0; i < NO_OF_SAMPLES; i++) {

		if (unit == ADC_UNIT_1)
		{
			adc_reading += adc1_get_raw((adc1_channel_t)channel);
		}else
		{
			int raw;
			adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
			adc_reading += raw;
		}
	}

	adc_reading /= NO_OF_SAMPLES;

	// Turn the led off by setting it HIGH
	gpio_set_level((gpio_num_t) DUSTSENSOR_LED, 1);

	// Get the raw output voltage	
    uint32_t vout = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
	printf("Raw: %d\tVoltage: %dmV\n", adc_reading, vout);	 

	// Calculate the dust density using the formula
	float dV = vout - Voc;
	if ( dV < 0 ) {
    	dV = 0;
	    Voc = vout;	// vout becomes the new constant
  	}

	float dustDensity = dV / K * 100.0;

	return dustDensity;
	
}


