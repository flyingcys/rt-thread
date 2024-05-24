#include <string.h>

#include <rtthread.h>
#include "pinctrl.h"
#include "drv_pinmux.h"

#define DBG_TAG              "drv.pinmux"
#define DBG_LVL               DBG_LOG
#include <rtdbg.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(ar)     (sizeof(ar)/sizeof(ar[0]))
#endif

/*
 * Function Selection for one Pin
 * @type: type of function
 * @select: value of selection
 */
struct fselect {
    fs_type type;
    uint8_t select;
};

/*
 * Function Mux for one Pin
 * @name: Pin Name
 * @addr: offset of pinmux registers against PINMUX_BASE
 * @offset: offset of function selection field in the pinmux register
 * @mask: mask of function selection field in the pinmux register
 */
struct fmux {
    char *name;
    uint8_t addr;
    uint8_t offset;
    uint8_t mask;
};

#define FS_NONE {fs_none, 0}

#define FS_PINMUX(PIN_NAME) {                   \
		.name = #PIN_NAME,                      \
        .addr = FMUX_GPIO_FUNCSEL_##PIN_NAME,   \
        .offset = PINMUX_OFFSET(PIN_NAME),      \
        .mask = PINMUX_MASK(PIN_NAME),          \
	}


/*
 * Define TWO tables for every SOC.
 * Table-1: pinmux array: every line maps to one pin register, store basic info.
 * Table-2: function selection array, extend Table-1, store function selection info.
 * Index of fmux array matches the same as that in fselect array.
 */
#if defined(SOC_TYPE_CV180X)

const struct fmux pinmux_array[] = {
	FS_PINMUX(SD0_CLK),
    FS_PINMUX(SD0_CMD),
    FS_PINMUX(SD0_D0),
    // TBD
};

struct fselect pin_selects_array[][8] = {
    
    /* SD0_CLK */ {{SDIO0_CLK, 0}, {IIC1_SDA, 1}, {SPI0_SCK, 2}, {XGPIOA_7, 3}, FS_NONE, {PWM_15, 5}, {EPHY_LNK_LED, 6}, {DBG_0, 7}},
    /* SD0_CMD */ {{SDIO0_CMD, 0}, {IIC1_SCL, 1}, {SPI0_SDO, 2}, {XGPIOA_8, 3}, FS_NONE, {PWM_14, 5}, {EPHY_SPD_LED, 6}, {DBG_1, 7}},
    /* SD0_D0 */  {{SDIO0_D_0, 0}, {CAM_MCLK1, 1}, {SPI0_SDI, 2}, {XGPIOA_9, 3}, {UART3_TX, 4}, {PWM_13, 5}, {WG0_D0, 6}, {DBG_2, 7}},
    // TBD
};

#elif defined(SOC_TYPE_SG2002)

// TBD

#else

#error "Unsupported SOC type!"

#endif

static int8_t pinmux_get_index(uint8_t pin_index, fs_type func_type)
{
    struct fselect *p;
    for (int i = 0; i < 8; i++) {
		p = &(pin_selects_array[pin_index][i]);
		LOG_D("[%d], type = %d, select = %d\n", i, p->type, p->select);
        if (p->type == func_type)
            return (int8_t)p->select; // it's safe bcos select should be [0, 7]
	}
    return -1;
}

static int pinmux_check_whitelist(const char *pin_name, const char *whitelist[])
{
    const char **name = &whitelist[0];
    while (*name) {
        if (0 == strcmp(pin_name, *name))
            return 0;
        name++;
    }
    return -1;
}

void pinmux_config(const char *pin_name, fs_type func_type, const char *whitelist[])
{
	const struct fmux *p_fmux;
	int index;
    int8_t select;

    if (whitelist) {
        if (0 != pinmux_check_whitelist(pin_name, whitelist)) {
            LOG_E("Pin Name \"%s\" is NOT Allowed by Whitelist!", pin_name);
            return;
        }
    }

	for (index = 0; index < ARRAY_SIZE(pinmux_array); index++) {
        p_fmux = &(pinmux_array[index]);
        LOG_I("index[%d]: name: %s, addr: %d, offset: %d, mask: %d\n",
                index, p_fmux->name, p_fmux->addr, p_fmux->offset, p_fmux->mask);
        if (0 == strcmp(pin_name, p_fmux->name)) {
            break;
        }
	}
    if (index == ARRAY_SIZE(pinmux_array)) {
        LOG_E("Pin Name \"%s\" is not found!", pin_name);
        return;
    }
    
    select = pinmux_get_index(index, func_type);
    if (-1 == select) {
        LOG_E("Can not found Function selection for Pin \"%s\"", pin_name);
        return;
    }
    LOG_I("Pin \"%s\" select Func [%d]", pin_name, select);
    //mmio_clrsetbits_32(PINMUX_BASE + p_fmux->addr, p_fmux->mask << p_fmux->offset, select);
}