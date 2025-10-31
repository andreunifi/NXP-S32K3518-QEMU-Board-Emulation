#include "adc.h"
void ADC_init(void){
    ADC_CTRL = 0x01;
}


uint32_t ADC_read(){
    ADC_CTRL |= 0x02;
    return ADC_DATA;
}