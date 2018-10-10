#ifndef __VOLTIMETRO__
#include "Voltimetro.h"
#endif

static esp_adc_cal_characteristics_t* adc_chars;
static const adc1_channel_t channel = ADC1_CHANNEL_0;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten      = ADC_ATTEN_DB_0;
static const adc_unit_t unit        = ADC_UNIT_1;

Voltimetro::Voltimetro( float _r1, float _r2){

    r1 = _r1;
    r2 = _r2;

    // multiplication factor
    factor = (float) (r1 + r2) / (float) r2;

    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);

    //Characterize ADC
    adc_chars = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

}


float Voltimetro::analogRead(){
    adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t) channel);
    }

    //Convert adc_reading to voltage in mV
    adc_reading /= NO_OF_SAMPLES;

    // Makes conversion within own esp to millivolts
    return (float)esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
}

float Voltimetro::getVoltage(){
    return (analogRead() * factor)/1000.0;
}

// performs baterry measure and warning low voltage values
void Voltimetro::perform_reading(void * pvParameters) {

}
