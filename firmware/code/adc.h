#ifndef __ADC__
#define __ADC__

#include "FreeRTOS.h"
 
 #define ADC_BASE_ADDRESS 0x400A000
 
 #define ADC0_OFFSET      (0    * 1024)
 #define ADC1_OFFSET      (16   * 1024)
 #define ADC2_OFFSET      (32   * 1024)
 
 #define ADC0 (ADC_BASE_ADDRESS + ADC0_OFFSET)
 #define ADC1 (ADC_BASE_ADDRESS + ADC1_OFFSET)
 #define ADC2 (ADC_BASE_ADDRESS + ADC2_OFFSET)
  
 #define S32K3XX_ADC_CTRL    0x00UL
 #define S32K3XX_ADC_CFG     0x04UL
 #define S32K3XX_ADC_DATA    0x08UL
 
 #define ADC0_ADDRESS                          (ADC_BASE_ADDRESS + ADC0_OFFSET)
 #define ADC_CTRL                              ( *( ( ( volatile uint32_t * ) ( ADC0 + S32K3XX_ADC_CTRL ) ) ) )
 #define ADC_CFG                               ( *( ( ( volatile uint32_t * ) ( ADC0 + S32K3XX_ADC_CFG ) ) ) )
 #define ADC_DATA                              ( *( ( ( volatile uint32_t * ) ( ADC0 + S32K3XX_ADC_DATA ) ) ) )
 

void ADC_init(void);
uint32_t ADC_read(void);

#endif
