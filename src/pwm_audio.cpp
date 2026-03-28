/*
 * pwm_audio.cpp
 *
 * Dual PWM audio output for RP2350 using DMA + PWM
 *
 * DCO1 on pin1 (GPIO2), DCO2 on pin2 (GPIO4)
 *
 * Both PWM slices share the same clock divider so they run
 * at identical sample rates. DMA interrupt on DCO1 completion
 * triggers refill of both buffers simultaneously.
 *
 * Double buffering: DMA plays buffer A while CPU fills buffer B.
 *
 * RC filter on each output: 1k + 47nF to GND
 */

#include "pwm_audio.h"
#include <Arduino.h>
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

/* --------------------------------------------------------
 * Configuration
 * -------------------------------------------------------- */
#define PWM_BITS        11
#define PWM_WRAP        ((1 << PWM_BITS) - 1)   /* 2047 */
#define PWM_CENTRE      (PWM_WRAP / 2)           /* 1023 */
#define PWM_SCALE       (PWM_CENTRE * 0.85f)     /* keep away from extremes */
#define BUFFER_SIZE     256

/* --------------------------------------------------------
 * Internal state - DCO1
 * -------------------------------------------------------- */
static uint8_t   pin1Gpio   = 0;
static uint8_t   slice1     = 0;
static uint8_t   chan1      = 0;
static int       dma1       = -1;

static uint16_t  dco1BufA[BUFFER_SIZE];
static uint16_t  dco1BufB[BUFFER_SIZE];
static uint16_t *dco1Play   = dco1BufA;
static uint16_t *dco1Fill   = dco1BufB;

/* --------------------------------------------------------
 * Internal state - DCO2
 * -------------------------------------------------------- */
static uint8_t   pin2Gpio   = 0;
static uint8_t   slice2     = 0;
static uint8_t   chan2      = 0;
static int       dma2       = -1;

static uint16_t  dco2BufA[BUFFER_SIZE];
static uint16_t  dco2BufB[BUFFER_SIZE];
static uint16_t *dco2Play   = dco2BufA;
static uint16_t *dco2Fill   = dco2BufB;

/* Signal main loop to refill */
static volatile bool bufferReady = false;

/* --------------------------------------------------------
 * Convert float buffer to PWM uint16_t levels
 * -------------------------------------------------------- */
static void floatToPWM(float *input, uint16_t *output, int len)
{
    for (int i = 0; i < len; i++)
    {
        float s = input[i];
        if (s >  1.0f) s =  1.0f;
        if (s < -1.0f) s = -1.0f;
        output[i] = (uint16_t)(s * PWM_SCALE + (float)PWM_CENTRE);
    }
}

/* --------------------------------------------------------
 * DMA interrupt - fires when DCO1 DMA finishes a buffer
 * Swap both DCO buffers and restart both DMAs immediately
 * -------------------------------------------------------- */
static void __not_in_flash_func(dmaIRQHandler)(void)
{
    /* clear DCO1 interrupt */
    dma_hw->ints0 = (1u << dma1);

    /* swap DCO1 buffers */
    uint16_t *tmp = dco1Play;
    dco1Play = dco1Fill;
    dco1Fill = tmp;
    dma_channel_set_read_addr(dma1, dco1Play, true);

    /* swap DCO2 buffers */
    tmp      = dco2Play;
    dco2Play = dco2Fill;
    dco2Fill = tmp;
    dma_channel_set_read_addr(dma2, dco2Play, true);

    bufferReady = true;
}

/* --------------------------------------------------------
 * Setup one PWM slice
 * -------------------------------------------------------- */
static void setupPWMSlice(uint8_t pin, uint8_t *sliceOut, uint8_t *chanOut, float clkDiv)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    *sliceOut = pwm_gpio_to_slice_num(pin);
    *chanOut  = pwm_gpio_to_channel(pin);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, clkDiv);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_FREE_RUNNING);
    pwm_init(*sliceOut, &cfg, false);
    pwm_set_chan_level(*sliceOut, *chanOut, PWM_CENTRE);
}

/* --------------------------------------------------------
 * Setup one DMA channel for PWM output
 * -------------------------------------------------------- */
static int setupDMAChannel(uint8_t slice, uint16_t *playBuf, bool enableIRQ)
{
    int ch = dma_claim_unused_channel(true);

    dma_channel_config dcfg = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&dcfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dcfg, true);
    channel_config_set_write_increment(&dcfg, false);
    channel_config_set_dreq(&dcfg, pwm_get_dreq(slice));

    dma_channel_configure(
        ch,
        &dcfg,
        &pwm_hw->slice[slice].cc,
        playBuf,
        BUFFER_SIZE,
        false
    );

    if (enableIRQ)
    {
        dma_channel_set_irq0_enabled(ch, true);
        irq_set_exclusive_handler(DMA_IRQ_0, dmaIRQHandler);
        irq_set_enabled(DMA_IRQ_0, true);
    }

    return ch;
}

/* --------------------------------------------------------
 * API
 * -------------------------------------------------------- */
void PWMAudio_Init(uint8_t pin1, uint8_t pin2, uint32_t sample_rate)
{
    pin1Gpio = pin1;
    pin2Gpio = pin2;

    float clkDiv = (float)clock_get_hz(clk_sys) / ((float)sample_rate * (float)(PWM_WRAP + 1));

    /* setup both PWM slices with identical clock divider */
    setupPWMSlice(pin1, &slice1, &chan1, clkDiv);
    setupPWMSlice(pin2, &slice2, &chan2, clkDiv);

    /* setup DMA - only DCO1 triggers the interrupt */
    dma1 = setupDMAChannel(slice1, dco1Play, true);
    dma2 = setupDMAChannel(slice2, dco2Play, false);

    /* pre-fill all buffers with silence */
    for (int i = 0; i < BUFFER_SIZE; i++)
        dco1BufA[i] = dco1BufB[i] = dco2BufA[i] = dco2BufB[i] = PWM_CENTRE;

    /* fill with first real audio blocks */
    float tmp[BUFFER_SIZE];

    PWM_CB_FillBuffer_DCO1(tmp, BUFFER_SIZE);
    floatToPWM(tmp, dco1Play, BUFFER_SIZE);
    PWM_CB_FillBuffer_DCO1(tmp, BUFFER_SIZE);
    floatToPWM(tmp, dco1Fill, BUFFER_SIZE);

    PWM_CB_FillBuffer_DCO2(tmp, BUFFER_SIZE);
    floatToPWM(tmp, dco2Play, BUFFER_SIZE);
    PWM_CB_FillBuffer_DCO2(tmp, BUFFER_SIZE);
    floatToPWM(tmp, dco2Fill, BUFFER_SIZE);

    /* start both PWM slices and DMAs together */
    pwm_set_mask_enabled((1u << slice1) | (1u << slice2));
    dma_channel_set_read_addr(dma1, dco1Play, true);
    dma_channel_set_read_addr(dma2, dco2Play, true);
}

void PWMAudio_Process(void)
{
    if (bufferReady)
    {
        bufferReady = false;

        float tmp[BUFFER_SIZE];

        PWM_CB_FillBuffer_DCO1(tmp, BUFFER_SIZE);
        floatToPWM(tmp, dco1Fill, BUFFER_SIZE);

        PWM_CB_FillBuffer_DCO2(tmp, BUFFER_SIZE);
        floatToPWM(tmp, dco2Fill, BUFFER_SIZE);
    }
}
