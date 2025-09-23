#ifndef LP5814_H
#define LP5814_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 7-bit I2C address (datasheet) */
#define LP5814_I2C_ADDR_DEFAULT 0x2C

/* Channels and engines */
typedef enum { LP5814_CH0 = 0, LP5814_CH1 = 1, LP5814_CH2 = 2, LP5814_CH3 = 3 } lp5814_ch_t;
typedef enum { LP5814_ENG0 = 0, LP5814_ENG1 = 1, LP5814_ENG2 = 2, LP5814_ENG3 = 3 } lp5814_engine_t;

/* Pattern index 0..3 */
typedef enum { LP5814_PAT0 = 0, LP5814_PAT1 = 1, LP5814_PAT2 = 2, LP5814_PAT3 = 3 } lp5814_pattern_t;

/* Engine repeat codes (per datasheet) */
typedef enum {
    LP5814_ENGINE_REPT_0 = 0,   /* 0 times */
    LP5814_ENGINE_REPT_1 = 1,   /* 1 time  */
    LP5814_ENGINE_REPT_2 = 2,   /* 2 times */
    LP5814_ENGINE_REPT_INF = 3  /* infinite */
} lp5814_engine_repeat_t;

/* Optional host hooks */
typedef struct {
    /* Write 'len' bytes starting at register 'reg' */
    int (*write)(void *user, uint8_t i2c_addr7, uint8_t reg, const uint8_t *data, size_t len);
    /* Read 'len' bytes starting at register 'reg' */
    int (*read)(void *user, uint8_t i2c_addr7, uint8_t reg, uint8_t *data, size_t len);

    /* Optional timing */
    void (*delay_ms)(uint32_t ms);

    /* Optional lock for thread safety */
    void (*lock)(void *user);
    void (*unlock)(void *user);

    void *user; /* passed back to callbacks */
} lp5814_bus_t;

/* Driver instance */
typedef struct {
    lp5814_bus_t bus;
    uint8_t addr; /* 7-bit */
} lp5814_t;

/* Convenience helpers for time codes used by sloper/fade tables:
   0->0s, 1->0.05s, 2->0.1s, 3->0.15s, 4->0.2s, 5->0.25s, 6->0.3s, 7->0.35s,
   8->0.4s, 9->0.45s, 10->0.5s, 11->1s, 12->2s, 13->4s, 14->6s, 15->8s       */
uint8_t lp5814_seconds_to_code(float seconds);

/* Init does not touch power. It enables CHIP_EN if requested. */
int lp5814_init(lp5814_t *dev, const lp5814_bus_t *bus, uint8_t i2c_addr7);

/* Core commands */
int lp5814_chip_enable(lp5814_t *dev, bool enable);         /* reg 0x00 bit0 */
int lp5814_set_max_current(lp5814_t *dev, bool mc_51mA);    /* reg 0x01 bit0 (0=25.5mA, 1=51mA) */
int lp5814_enable_outputs(lp5814_t *dev, uint8_t mask_4b);  /* reg 0x02 bits[3:0] */
int lp5814_set_fade(lp5814_t *dev, bool ch0, bool ch1, bool ch2, bool ch3,
                    uint8_t fade_time_code);                /* reg 0x03 */
int lp5814_set_exp_and_mode(lp5814_t *dev, uint8_t exp_mask_4b, uint8_t auto_mask_4b); /* reg 0x04 */
int lp5814_map_output_to_engine(lp5814_t *dev, lp5814_engine_t ch_for_out[4]);         /* reg 0x05 */
int lp5814_update_config(lp5814_t *dev);                    /* reg 0x0F write 0x55 */
int lp5814_start(lp5814_t *dev);                            /* reg 0x10 write 0xFF */
int lp5814_stop(lp5814_t *dev);                             /* reg 0x11 write 0xAA */
int lp5814_pause(lp5814_t *dev, bool pause);                /* reg 0x12 bit0 */
int lp5814_shutdown(lp5814_t *dev);                         /* reg 0x0D write 0x33 */
int lp5814_reset(lp5814_t *dev);                            /* reg 0x0E write 0xCC */
int lp5814_clear_flags(lp5814_t *dev, bool clr_tsd, bool clr_por); /* reg 0x13 bits1:0 */

/* Status */
typedef struct {
    bool out_busy[4];
    bool any_engine_busy;
    bool tsd;
    bool por;
} lp5814_flags_t;
int lp5814_read_flags(lp5814_t *dev, lp5814_flags_t *flags); /* reg 0x40 */

/* Analog dimming (Dot Current) and manual PWM */
int lp5814_set_dc(lp5814_t *dev, lp5814_ch_t ch, uint8_t dc_0_255);         /* regs 0x14..0x17 */
int lp5814_set_manual_pwm(lp5814_t *dev, lp5814_ch_t ch, uint8_t pwm_0_255);/* regs 0x18..0x1B */

/* Engine ordering and repeats */
int lp5814_engine_set_order(lp5814_t *dev, lp5814_engine_t eng, uint8_t order_idx_0_3,
                            lp5814_pattern_t pat);                           /* regs 0x06..0x09 */
int lp5814_engine_enable_orders(lp5814_t *dev, lp5814_engine_t eng, uint8_t order_enable_mask_4b);
/* Set repeat for a single engine (2-bit code). This RMWs register 0x0C. */
int lp5814_engine_set_repeat(lp5814_t *dev, lp5814_engine_t eng, lp5814_engine_repeat_t rept);

/* Pattern builder: raw accessors and a helper pack */
typedef struct {
    /* Pause register (0x1C, 0x25, 0x2E, 0x37). Two nibbles (T0, T1). */
    uint8_t pause_time;
    /* Repeat register (0x1D, 0x26, 0x2F, 0x38). Lower bits repeat; upper are reserved per datasheet. */
    uint8_t repeat;
    /* 5 x PWM points [start, p1, p2, p3, end] at 23 kHz, 8-bit */
    uint8_t pwm[5];
    /* Two sloper time bytes: (T1<<4|T0) and (T3<<4|T2) where Tn are 4-bit codes from table */
    uint8_t sloper_time1; /* PATTERNx_SLOPER_TIME1 */
    uint8_t sloper_time2; /* PATTERNx_SLOPER_TIME2 */
} lp5814_pattern_cfg_t;

/* Write an entire PATTERNx atomically */
int lp5814_pattern_write(lp5814_t *dev, lp5814_pattern_t idx, const lp5814_pattern_cfg_t *cfg);

/* Small helper to pack four sloper codes into the two bytes used by the pattern */
static inline void lp5814_pack_sloper_times(uint8_t t0, uint8_t t1, uint8_t t2, uint8_t t3,
                                            uint8_t *out_time1, uint8_t *out_time2)
{
    if (out_time1) *out_time1 = (uint8_t)((t1 << 4) | (t0 & 0xF));
    if (out_time2) *out_time2 = (uint8_t)((t3 << 4) | (t2 & 0xF));
}

#ifdef __cplusplus
}
#endif
#endif /* LP5814_H */
