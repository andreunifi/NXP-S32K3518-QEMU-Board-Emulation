/*
 * s32k3xx_adc.h
 *
 * Questo file definisce la struttura dell'ADC per la scheda S32K3X8EVB.
 * Usato per l'emulazione in QEMU.
 */

#ifndef NXP_ADC_H
#define NXP_ADC_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define ADC_BASE_ADDRESS 0x400A000

#define ADC0_OFFSET      (0    * 1024)
#define ADC1_OFFSET      (16   * 1024)
#define ADC2_OFFSET      (32   * 1024)

#define ADC0 (ADC_BASE_ADDRESS + ADC0_OFFSET)
#define ADC1 (ADC_BASE_ADDRESS + ADC1_OFFSET)
#define ADC2 (ADC_BASE_ADDRESS + ADC2_OFFSET)

#define ADC_SIZE        (16    * 1024)

/* End of conversion interrupt */
#define ADC0_EOC            180
#define ADC1_EOC            181
#define ADC2_EOC            182


/*
 * Registri dell'ADC.
 * CTRL: Controllo dell'ADC, accensione e start.
 * CFG:  Configurazione del sampling.
 * DATA: Il valore letto dall'ADC.
 */
#define NXP_ADC_CTRL    0x00
#define NXP_ADC_CFG     0x04
#define NXP_ADC_DATA    0x08

/* Bit di controllo */
#define NXP_ADC_ENABLE  0x01  // Accendi l'ADC
#define NXP_ADC_START   0x02  // Inizia la conversione

#define TYPE_NXP_ADC "nxp_adc"
OBJECT_DECLARE_SIMPLE_TYPE(NXPADCState, NXP_ADC)

/* 
 * Struttura che rappresenta l'ADC virtuale.
 */
struct NXPADCState {
    SysBusDevice parent_obj; // Device principale
    MemoryRegion mmio;       // Memoria mappata
    uint32_t ctrl;           // Registro di controllo
    uint32_t cfg;            // Registro di configurazione
    uint32_t data;           // Ultimo valore letto
    qemu_irq irq;            // Interrupt per segnalare fine conversione
};

#endif /* NXP_ADC_H */
