
#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/adc/nxps32k3x8evb_adc.h"
#include "hw/irq.h"


#define ADC_MAX_VALUE 0xFFF  // Massimo valore dell'ADC (12-bit)

#define ADC_LOG(fmt, ...) qemu_log("%s: " fmt, __func__, ## __VA_ARGS__) //questo è ganzo, puoi linkare il log di qemu ad una call ADC_LOG- utile

static uint32_t nxp_adc_generate_value(NXPADCState *s) {
  //scegliere l'implementazione. Come facciamo?
    s->data = (s->data + 20) & ADC_MAX_VALUE; // Cambia il valore ad ogni lettura.
    return s->data;
}

/*
 * Legge dai registri dell'ADC.
 */
static uint64_t nxp_adc_read(void *opaque, hwaddr addr, unsigned int size) {
    qemu_log("ADC READ, addr %ld\n", addr);

    NXPADCState *s = opaque;
    switch (addr) {
    case NXP_ADC_CTRL:
        return s->ctrl;
    case NXP_ADC_CFG:
        return s->cfg;
    case NXP_ADC_DATA:
        if ((s->ctrl & NXP_ADC_ENABLE) && (s->ctrl & NXP_ADC_START)) {
            qemu_log("ADC correctly configured for read\n");
            s->ctrl &= ~NXP_ADC_START; // Reset del bit START dopo la lettura
            uint32_t result = nxp_adc_generate_value(s);
            qemu_irq_pulse(s->irq); // Segnala interruzione (conversione finita)
            return result;
        } else {
            return s->data;
        }
    default:
        ADC_LOG("Accesso errato: 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}


static void nxp_adc_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size) {
    NXPADCState *s = opaque;
    switch (addr) {
    case NXP_ADC_CTRL:
        s->ctrl = value;
        ADC_LOG("CTRL scritto: 0x%x\n", s->ctrl);
        break;
    case NXP_ADC_CFG:
        s->cfg = value;
        ADC_LOG("CFG scritto: 0x%x\n", s->cfg);
        break;
    case NXP_ADC_DATA:
        ADC_LOG("Tentativo di scrittura su un registro di sola lettura\n");
        break;
    default:
        ADC_LOG("Accesso errato in scrittura: 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps nxp_adc_ops = {
    .read = nxp_adc_read,
    .write = nxp_adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/*
 * Reset dell'ADC
 */
static void nxp_adc_reset(DeviceState *dev) {
    NXPADCState *s = NXP_ADC(dev);
    s->ctrl = 0;
    s->cfg = 0;
    s->data = 0;
    qemu_log("ADC reset\n");

}

static const VMStateDescription vmstate_nxp_adc = {
    .name = TYPE_NXP_ADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, NXPADCState),
        VMSTATE_UINT32(cfg, NXPADCState),
        VMSTATE_UINT32(data, NXPADCState),
        VMSTATE_END_OF_LIST()
    }
};

static void nxp_adc_init(Object *obj) {
    NXPADCState *s = NXP_ADC(obj);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    memory_region_init_io(&s->mmio, obj, &nxp_adc_ops, s, TYPE_NXP_ADC, 0x0C);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qemu_log("ADC Initialized\n");

}

static void nxp_adc_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, nxp_adc_reset);
    dc->vmsd = &vmstate_nxp_adc;
}

static const TypeInfo nxp_adc_info = {
    .name          = TYPE_NXP_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NXPADCState),
    .instance_init = nxp_adc_init,
    .class_init    = nxp_adc_class_init,
};

static void nxp_adc_register_types(void) {
    type_register_static(&nxp_adc_info);
}

type_init(nxp_adc_register_types)
