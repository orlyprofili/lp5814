#include "lp5814.h"
#include <string.h>

/* Register addresses (datasheet) */
enum {
    REG_CHIP_EN        = 0x00,
    REG_DEV_CONFIG0    = 0x01, /* MAX_CURRENT bit0 */
    REG_DEV_CONFIG1    = 0x02, /* OUTx_EN bits[3:0] */
    REG_DEV_CONFIG2    = 0x03, /* LED_FADE_TIME[7:4], OUTx_FADE_EN[3:0] */
    REG_DEV_CONFIG3    = 0x04, /* OUTx_EXP_EN[7:4], OUTx_AUTO_EN[3:0] */
    REG_DEV_CONFIG4    = 0x05, /* OUTx_ENGINE_CH pairs */
    REG_ENGINE_CFG0    = 0x06,
    REG_ENGINE_CFG1    = 0x07,
    REG_ENGINE_CFG2    = 0x08,
    REG_ENGINE_CFG3    = 0x09,
    REG_ENGINE_CFG4    = 0x0A, /* E1O*_EN (7:4), E0O*_EN(3:0) */
    REG_ENGINE_CFG5    = 0x0B, /* E3O*_EN (7:4), E2O*_EN(3:0) */
    REG_ENGINE_CFG6    = 0x0C, /* ENGINE3/2/1/0_REPT nibbles */
    REG_SHUTDOWN_CMD   = 0x0D, /* write 0x33 */
    REG_RESET_CMD      = 0x0E, /* write 0xCC */
    REG_UPDATE_CMD     = 0x0F, /* write 0x55 */
    REG_START_CMD      = 0x10, /* write 0xFF */
    REG_STOP_CMD       = 0x11, /* write 0xAA */
    REG_PAUSE_CONT     = 0x12, /* bit0 */
    REG_FLAG_CLR       = 0x13, /* bit1 TSD_CLR, bit0 POR_CLR */
    REG_OUT0_DC        = 0x14,
    REG_OUT1_DC        = 0x15,
    REG_OUT2_DC        = 0x16,
    REG_OUT3_DC        = 0x17,
    REG_OUT0_PWM       = 0x18,
    REG_OUT1_PWM       = 0x19,
    REG_OUT2_PWM       = 0x1A,
    REG_OUT3_PWM       = 0x1B,

    /* Pattern blocks: base + 0,1,2.. stride 9 bytes each */
    REG_PAT0_BASE      = 0x1C, /* PAUSE, REPEAT, PWM0..4, SLOPER_TIME1, SLOPER_TIME2 */
    REG_PAT_STRIDE     = 9,

    REG_FLAG           = 0x40
};

/* --- Internals --- */
static inline void lock_if(const lp5814_bus_t *b)   { if (b->lock)   b->lock(b->user); }
static inline void unlock_if(const lp5814_bus_t *b) { if (b->unlock) b->unlock(b->user); }
static inline void delay_if(const lp5814_bus_t *b, uint32_t ms) { if (b->delay_ms) b->delay_ms(ms); }

static int wr1(lp5814_t *dev, uint8_t reg, uint8_t val)
{
    if (!dev) return -1;
    const lp5814_bus_t *b = &dev->bus;
    if (!b->write) return -1;
    lock_if(b);
    int rc = b->write(b->user, dev->addr, reg, &val, 1);
    unlock_if(b);
    return rc;
}

static int rd1(lp5814_t *dev, uint8_t reg, uint8_t *val)
{
    if (!dev || !val) return -1;
    const lp5814_bus_t *b = &dev->bus;
    if (!b->read) return -1;
    lock_if(b);
    int rc = b->read(b->user, dev->addr, reg, val, 1);
    unlock_if(b);
    return rc;
}

static int rmw(lp5814_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    if (!dev) return -1;
    uint8_t cur = 0;
    int rc = rd1(dev, reg, &cur);
    if (rc) return rc;
    cur = (uint8_t)((cur & ~mask) | (value & mask));
    return wr1(dev, reg, cur);
}

uint8_t lp5814_seconds_to_code(float s)
{
    /* Map to nearest allowed code. */
    const float steps[16] = {0.f,0.05f,0.10f,0.15f,0.20f,0.25f,0.30f,0.35f,0.40f,0.45f,0.50f,1.0f,2.0f,4.0f,6.0f,8.0f};
    uint8_t best = 0; float best_err = 1e9f;
    for (uint8_t i=0;i<16;i++){ float e = s-steps[i]; if (e<0) e=-e; if (e<best_err){ best_err=e; best=i; } }
    return best;
}

/* --- Public API --- */
int lp5814_init(lp5814_t *dev, const lp5814_bus_t *bus, uint8_t i2c_addr7)
{
    if (!dev || !bus || !bus->write || !bus->read) return -1;
    dev->bus = *bus;
    dev->addr = i2c_addr7 ? i2c_addr7 : LP5814_I2C_ADDR_DEFAULT;
    return 0;
}

int lp5814_chip_enable(lp5814_t *dev, bool enable)
{
    if (!dev) return -1;
    int rc = rmw(dev, REG_CHIP_EN, 0x01u, enable ? 0x01u : 0x00u);
    /* Datasheet suggests ~1 ms after power up before config; keep a small guard */
    if (!rc && enable) delay_if(&dev->bus, 1);
    return rc;
}

int lp5814_set_max_current(lp5814_t *dev, bool mc_51mA)
{
    return rmw(dev, REG_DEV_CONFIG0, 0x01u, mc_51mA ? 0x01u : 0x00u);
}

int lp5814_enable_outputs(lp5814_t *dev, uint8_t mask_4b)
{
    return rmw(dev, REG_DEV_CONFIG1, 0x0Fu, (uint8_t)(mask_4b & 0x0F));
}

int lp5814_set_fade(lp5814_t *dev, bool ch0, bool ch1, bool ch2, bool ch3, uint8_t fade_time_code)
{
    uint8_t v = (uint8_t)((fade_time_code & 0xF) << 4);
    v |= (uint8_t)((ch3 ? 1u:0u) << 3) | (uint8_t)((ch2?1u:0u) << 2) |
         (uint8_t)((ch1 ? 1u:0u) << 1) | (uint8_t)(ch0 ? 1u:0u);
    return wr1(dev, REG_DEV_CONFIG2, v);
}

int lp5814_set_exp_and_mode(lp5814_t *dev, uint8_t exp_mask_4b, uint8_t auto_mask_4b)
{
    uint8_t v = (uint8_t)((exp_mask_4b & 0x0F) << 4) | (uint8_t)(auto_mask_4b & 0x0F);
    return wr1(dev, REG_DEV_CONFIG3, v);
}

int lp5814_map_output_to_engine(lp5814_t *dev, lp5814_engine_t ch_for_out[4])
{
    if (!ch_for_out) return -1;
    uint8_t v = 0;
    v |= (uint8_t)(((uint8_t)ch_for_out[0] & 0x3u) << 0);
    v |= (uint8_t)(((uint8_t)ch_for_out[1] & 0x3u) << 2);
    v |= (uint8_t)(((uint8_t)ch_for_out[2] & 0x3u) << 4);
    v |= (uint8_t)(((uint8_t)ch_for_out[3] & 0x3u) << 6);
    return wr1(dev, REG_DEV_CONFIG4, v);
}

int lp5814_update_config(lp5814_t *dev) { return wr1(dev, REG_UPDATE_CMD, 0x55); }
int lp5814_start(lp5814_t *dev)        { return wr1(dev, REG_START_CMD,  0xFF); }
int lp5814_stop(lp5814_t *dev)         { return wr1(dev, REG_STOP_CMD,   0xAA); }

int lp5814_pause(lp5814_t *dev, bool pause_run)
{
    return rmw(dev, REG_PAUSE_CONT, 0x01u, pause_run ? 0x01u : 0x00u);
}

int lp5814_shutdown(lp5814_t *dev) { return wr1(dev, REG_SHUTDOWN_CMD, 0x33); }
int lp5814_reset(lp5814_t *dev)    { return wr1(dev, REG_RESET_CMD,    0xCC); }

int lp5814_clear_flags(lp5814_t *dev, bool clr_tsd, bool clr_por)
{
    uint8_t v = (uint8_t)((clr_tsd ? 1u:0u) << 1) | (uint8_t)(clr_por ? 1u:0u);
    return wr1(dev, REG_FLAG_CLR, v);
}

int lp5814_read_flags(lp5814_t *dev, lp5814_flags_t *flags)
{
    if (!flags) return -1;
    uint8_t v=0; int rc = rd1(dev, REG_FLAG, &v); if (rc) return rc;
    flags->out_busy[0]     = (v >> 3) & 1u;
    flags->out_busy[1]     = (v >> 4) & 1u;
    flags->out_busy[2]     = (v >> 5) & 1u;
    flags->out_busy[3]     = (v >> 6) & 1u;
    flags->any_engine_busy = (v >> 2) & 1u;
    flags->tsd             = (v >> 1) & 1u;
    flags->por             = (v >> 0) & 1u;
    return 0;
}

static uint8_t dc_reg_for(lp5814_ch_t ch)  { return (uint8_t)(REG_OUT0_DC  + (uint8_t)ch); }
static uint8_t pwm_reg_for(lp5814_ch_t ch) { return (uint8_t)(REG_OUT0_PWM + (uint8_t)ch); }

int lp5814_set_dc(lp5814_t *dev, lp5814_ch_t ch, uint8_t dc_0_255)
{
    return wr1(dev, dc_reg_for(ch), dc_0_255);
}

int lp5814_set_manual_pwm(lp5814_t *dev, lp5814_ch_t ch, uint8_t pwm_0_255)
{
    return wr1(dev, pwm_reg_for(ch), pwm_0_255);
}

static uint8_t engine_cfg_reg(lp5814_engine_t eng)
{
    switch (eng) { case LP5814_ENG0: return REG_ENGINE_CFG0;
                   case LP5814_ENG1: return REG_ENGINE_CFG1;
                   case LP5814_ENG2: return REG_ENGINE_CFG2;
                   default:          return REG_ENGINE_CFG3; }
}

int lp5814_engine_set_order(lp5814_t *dev, lp5814_engine_t eng, uint8_t order_idx_0_3,
                            lp5814_pattern_t pat)
{
    if (order_idx_0_3 > 3) return -1;
    uint8_t reg = engine_cfg_reg(eng);
    uint8_t shift = (uint8_t)(order_idx_0_3 * 2);
    uint8_t mask  = (uint8_t)(0x3u << shift);
    uint8_t val   = (uint8_t)(((uint8_t)pat & 0x3u) << shift);
    return rmw(dev, reg, mask, val);
}

int lp5814_engine_enable_orders(lp5814_t *dev, lp5814_engine_t eng, uint8_t order_enable_mask_4b)
{
    uint8_t reg = (eng < LP5814_ENG2) ? REG_ENGINE_CFG4 : REG_ENGINE_CFG5;
    uint8_t shift = (eng == LP5814_ENG0) ? 0 :
                    (eng == LP5814_ENG1) ? 4 :
                    (eng == LP5814_ENG2) ? 0 : 4;
    uint8_t mask = (uint8_t)(0xFu << shift);
    uint8_t val  = (uint8_t)((order_enable_mask_4b & 0xF) << shift);
    return rmw(dev, reg, mask, val);
}

int lp5814_engine_set_repeat(lp5814_t *dev, lp5814_engine_t eng, lp5814_engine_repeat_t rept)
{
    uint8_t shift = (eng == LP5814_ENG0) ? 0 :
                    (eng == LP5814_ENG1) ? 2 :
                    (eng == LP5814_ENG2) ? 4 : 6;
    uint8_t mask = (uint8_t)(0x3u << shift);
    uint8_t val  = (uint8_t)(((uint8_t)rept & 0x3u) << shift);
    return rmw(dev, REG_ENGINE_CFG6, mask, val);
}

static int write_seq(lp5814_t *dev, uint8_t start_reg, const uint8_t *data, size_t len)
{
    if (!dev || !data || !len) return -1;
    const lp5814_bus_t *b = &dev->bus;
    if (!b->write) return -1;
    lock_if(b);
    int rc = b->write(b->user, dev->addr, start_reg, data, len);
    unlock_if(b);
    return rc;
}

int lp5814_pattern_write(lp5814_t *dev, lp5814_pattern_t idx, const lp5814_pattern_cfg_t *cfg)
{
    if (!cfg || !dev) return -1;
    if ((uint8_t)idx > (uint8_t)LP5814_PAT3) return -1;
    uint8_t base = (uint8_t)(REG_PAT0_BASE + (uint8_t)idx * REG_PAT_STRIDE);
    uint8_t buf[9];
    buf[0] = cfg->pause_time;
    buf[1] = (uint8_t)(cfg->repeat & 0x0Fu);
    buf[2] = cfg->pwm[0];
    buf[3] = cfg->pwm[1];
    buf[4] = cfg->pwm[2];
    buf[5] = cfg->pwm[3];
    buf[6] = cfg->pwm[4];
    buf[7] = cfg->sloper_time1;
    buf[8] = cfg->sloper_time2;
    return write_seq(dev, base, buf, sizeof(buf));
}
