/*
 * IC VISION v2 — 14-pin 74HC logic IC tester, STM32CubeIDE / HAL, NUCLEO-F401RE, NO RELAY.
 *
 * DUT power: 3V3 -> INA219 VIN+ -> VIN- -> socket pin 14 (always on). Unplug USB before swapping chips.
 *
 * CubeMX (.ioc):
 *   - Board selector NUCLEO-F401RE, default peripherals (84 MHz, USART2 PA2/PA3 115200 8N1)
 *   - USART2: NVIC "USART2 global interrupt" ENABLED (if not, code falls back to blocking/polled UART)
 *   - I2C1  : enable it (any pins). This code forces it onto PB8 (SCL) / PB9 (SDA) at I2C_HZ itself,
 *             and recovers the bus automatically if a transfer fails or the peripheral locks up BUSY.
 *   - Everything else (DUT pins, ADC1, buttons, LEDs, buzzer PWM on TIM2, LD2 heartbeat) is configured here.
 * main.c:  #include "icv.h";  icv_init() after MX_*_Init();  icv_loop() inside while(1).
 * Do not define HAL_UART_RxCpltCallback / TxCpltCallback / ErrorCallback elsewhere.
 *
 * Buttons:  SEL (D8) short = next IC / next result page   SEL hold 0.8 s = live pin meter / back
 *           RUN (B1) short = test                           RUN hold 0.8 s = identify chip
 *           MAN (D5) short = manual mode (idle)             in manual: MAN = next input pattern,
 *                                                           SEL = next gate, RUN = auto-step on/off, hold = exit
 */
#include "main.h"
#include "icv.h"
#include "ina219.h"
#include "oled.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef  hi2c1;

/* ============================== CONFIG ============================== */
#define FW_VERSION   "icvision-f401-hal-4.1"
/* The IC VISION carrier PCB v1 uses exactly this Nucleo pin map (same pins, same LEDs/buttons), so the
 * same firmware runs on both. Only check SystemClock_Config(): it must use HSI (no ST-Link MCO on the PCB). */
#define SIMPLE_UI    1               /* 1 = one-screen IC checker (any button = retest); 0 = full menu UI */
#define OLED_SH1106  0               /* 0 = 0.96" SSD1306 (current), 1 = 1.3" SH1106 */
#define OLED_ADDR    0x3C
#define OLED_ADDR_ALT 0x3D            /* tried if 0x3C does not answer */
#define I2C_HZ       400000           /* 400 kHz fast mode (SSD1306 + INA219); drops to 100 kHz on its own if the bus is unreliable */
#define I2C_HZ_SAFE  100000
#define INA_ADDR     0x40

#define BTN_SEL_PORT GPIOA
#define BTN_SEL_PIN  9               /* D8, to GND */
#define BTN_RUN_PORT GPIOC
#define BTN_RUN_PIN  13              /* B1 (blue)  */
#define BTN_MAN_PORT GPIOB
#define BTN_MAN_PIN  4               /* D5, to GND: manual mode / next input pattern */
#define BUZ_PORT     GPIOB
#define BUZ_PIN      10              /* D6  */
#define LEDP_PORT    GPIOA
#define LEDP_PIN     8               /* D7  (D9/PC7 header contact was bad on this board) */
#define LEDF_PORT    GPIOB
#define LEDF_PIN     6               /* D10 */
#define HB_PORT      GPIOA
#define HB_PIN       5               /* Nucleo LD2 / PCB blue RUN LED D3: blinks = firmware running */

static const float    VCC_MIN        = 3.00f;
static const float    VCC_MAX        = 3.60f;
static const float    VALID_HI       = 0.70f;   /* logic 1 if V >= 0.7*VCC */
static const float    VALID_LO       = 0.30f;   /* logic 0 if V <= 0.3*VCC, else X */
static const float    STRONG_MARGIN  = 0.15f;   /* "strong" = within 0.15 V of the rail */
static const float    IDD_WARN_MA    = 2.0f;    /* INA219 offset alone can be +-0.5 mA on this range */
static const float    IDD_FAIL_MA    = 8.0f;
static const float    IDD_ABORT_MA   = 40.0f;
static const uint32_t SETTLE_US      = 300;
static const uint32_t PULL_SETTLE_US = 200;
static const uint8_t  ADC_AVG        = 8;
static const uint32_t LIVE_MS        = 250;     /* live VCC / current / meter refresh */
static const uint32_t LONG_MS        = 800;
static const uint32_t DEBOUNCE_MS    = 30;
static const float W_FUNC = 55, W_LEVEL = 20, W_SUPPLY = 10, W_INTEG = 15;
static const float FAIL_CAP = 49;

#ifndef VREFINT_CAL_RAW
#define VREFINT_CAL_RAW (*(const uint16_t *)0x1FFF7A2AUL)   /* F401: VREFINT @ VDDA=3.3 V, 30 degC */
#endif
#define ADC_CH_VREFINT  17u

/* ============================== PIN MAP ============================== */
/* socket pin -> GPIO + ADC1 channel (-1 = digital only). 7 = GND, 14 = VCC via INA219. */
typedef struct { GPIO_TypeDef *port; uint8_t pin; int8_t ch; } PinDef;
static const PinDef DUT[15] = {
    [1]  = { GPIOA,  0,  0 },   /* A0     */
    [2]  = { GPIOA,  1,  1 },   /* A1     */
    [3]  = { GPIOA,  4,  4 },   /* A2     */
    [4]  = { GPIOB,  0,  8 },   /* A3     */
    [5]  = { GPIOA, 10, -1 },   /* D2     */
    [6]  = { GPIOC,  1, 11 },   /* A4     */
    [8]  = { GPIOC,  0, 10 },   /* A5     */
    [9]  = { GPIOB,  5, -1 },   /* D4     */
    [10] = { GPIOA,  7,  7 },   /* D11    */
    [11] = { GPIOA,  6,  6 },   /* D12    */
    [12] = { GPIOC,  2, 12 },   /* CN7-35 */
    [13] = { GPIOC,  3, 13 },   /* CN7-37 */
};

/* ============================== IC DATABASE ============================== */
typedef enum { G_NAND, G_AND, G_OR, G_NOR, G_XOR, G_NOT } GateFn;
typedef struct { uint8_t in[3]; uint8_t nin; uint8_t out; } Gate;
typedef struct { const char *id; const char *name; GateFn fn; uint8_t ngates; Gate g[6]; } ICDef;

#define QUAD_A {{{1,2},2,3},{{4,5},2,6},{{9,10},2,8},{{12,13},2,11}}
#define TRI3   {{{1,2,13},3,12},{{3,4,5},3,6},{{9,10,11},3,8}}

static const ICDef IC_DB[] = {
    {"7400", "Quad 2-in NAND",   G_NAND, 4, QUAD_A},
    {"7402", "Quad 2-in NOR",    G_NOR,  4, {{{2,3},2,1},{{5,6},2,4},{{8,9},2,10},{{11,12},2,13}}},
    {"7404", "Hex inverter",     G_NOT,  6, {{{1},1,2},{{3},1,4},{{5},1,6},{{9},1,8},{{11},1,10},{{13},1,12}}},
    {"7408", "Quad 2-in AND",    G_AND,  4, QUAD_A},
    {"7410", "Triple 3-in NAND", G_NAND, 3, TRI3},
    {"7411", "Triple 3-in AND",  G_AND,  3, TRI3},
    {"7427", "Triple 3-in NOR",  G_NOR,  3, TRI3},
    {"7432", "Quad 2-in OR",     G_OR,   4, QUAD_A},
    {"7486", "Quad 2-in XOR",    G_XOR,  4, QUAD_A},
};
#define IC_COUNT ((uint8_t)(sizeof IC_DB / sizeof IC_DB[0]))

/* ============================== STATE ============================== */
typedef struct { uint8_t x, exp, ok; char m; float v; } VecRes;
typedef struct { VecRes vec[8]; uint8_t nvec; bool pass; } GateRes;
typedef struct { uint8_t pin; bool a, b; } PinRes;           /* outputs: hi/lo hold, inputs: pu/pd follow */

typedef struct {
    uint16_t reads, correct, strong, xtalk;
    float    qsum;
    uint16_t integTot, integOk, outTot, outFloat;
    float    iddMax;
    GateRes  g[6];
    PinRes   outs[6];  uint8_t nOuts;
    PinRes   ins[12];  uint8_t nIns;
    bool     funcDone, integDone;
} Tally;

typedef struct {
    bool        valid;
    uint8_t     ic;
    const char *verdict, *reason;
    float       elec, fFunc, fLevel, fInteg, fSupp, vcc, idd;
    bool        noIc;
    uint8_t     nLooks;
    const char *looks[IC_COUNT];
    float       vOH[6], vOL[6];                               /* per gate: min high / max low, -1 = none */
} Result;

typedef struct { float v[15]; char role[15]; float vcc, idd; } PinSnap;   /* role: i o G V */

typedef enum { UI_IDLE, UI_METER, UI_BUSY, UI_RESULT, UI_IDENT, UI_ALERT, UI_MANUAL, UI_SIMPLE } UiMode;

static Tally    g_main, g_scratch;           /* static: CubeIDE default stack is only 1 KB */
static Result   R;
static PinSnap  snap;
static UiMode   ui = UI_IDLE;
static uint8_t  resPage;
static bool     inaOk, oledOk;
static uint8_t  selIdx;
static float    vccDut = 3.3f, vdda = 3.3f;
static float    liveV = 0, liveI = -1;
static float    iddOffset;               /* INA219 zero offset, measured at boot with an empty socket */
static uint32_t liveT;
static const char *idM[IC_COUNT];
static uint8_t  idN;
static int      idClaim = -1;
static const char *alertMsg;
static uint32_t i2cRecoveries, inaErrSeen;
static uint32_t i2cHz = I2C_HZ, i2cRecT, i2cRecBurst;
static int8_t   autoState = -2;          /* -2 unknown, -1 empty socket, 0 bad, 1 good */
static bool     autoSel = true;          /* AUTO: switch to whatever IC type the chip behaves like */
static bool     idTried;                 /* identify already run for the current BAD chip */
static int8_t   looksIdx = -1;           /* DB index the chip behaves like (when BAD), -1 none */
static float    looksScore;              /* its truth-table match (0..1) */
static uint8_t  diagGatesOk;             /* bit g set = gate g passed (selected type, last auto check) */
static int8_t   diagBadGate = -1;        /* first failing gate, -1 none */
static uint8_t  diagBadPin;              /* first pin failing the pull test (0 none) */
static uint32_t evalSig;                 /* fingerprint of the chip's response to the selected type */
/* SIMPLE_UI result */
enum { SS_NONE, SS_GOOD, SS_BAD, SS_EMPTY, SS_UNKNOWN, SS_ERROR };
static uint8_t     sState = SS_NONE;
static const char *sReason;
static int8_t      sBadGate = -1;
static bool        quietSteps;           /* step(): JSON only, no OLED progress screen */
static bool        pendingCheck;
typedef struct { GPIO_TypeDef *port; uint8_t pin; bool prev; uint32_t t; bool longFired; } Btn;
static Btn bSel = { BTN_SEL_PORT, BTN_SEL_PIN, false, 0, false };
static Btn bRun = { BTN_RUN_PORT, BTN_RUN_PIN, false, 0, false };
static Btn bMan = { BTN_MAN_PORT, BTN_MAN_PIN, false, 0, false };

/* manual mode */
static uint8_t  manGate, manBits;
static bool     manAuto;
static uint32_t manT;
static char     manGot[9];               /* measured level per input pattern, '?' = not yet */
static float    manV;
static char     manM;
static bool     manOk;
static uint32_t autoT;
static bool     skipIdd;                 /* run_functional: no INA219 reads (keeps I2C quiet) */

static const char *auto_text(void)
{
    return autoState == 1 ? "GOOD" : autoState == 0 ? "BAD" : autoState == -1 ? "EMPTY" : "....";
}

static void i2c_recover(void);
static void flush(void);

/* ============================== LOW LEVEL ============================== */
enum { M_IN = 0u, M_OUT = 1u, M_AF = 2u, M_AN = 3u };
enum { PL_NONE = 0u, PL_UP = 1u, PL_DOWN = 2u };

static void pin_cfg(GPIO_TypeDef *g, uint8_t n, uint32_t mode, uint32_t pull)
{
    uint32_t s = 2u * n;
    g->PUPDR = (g->PUPDR & ~(3u << s)) | (pull << s);
    g->MODER = (g->MODER & ~(3u << s)) | (mode << s);
}
static inline void pin_wr(GPIO_TypeDef *g, uint8_t n, bool v) { g->BSRR = v ? (1u << n) : (1u << (n + 16u)); }
static inline bool pin_rd(GPIO_TypeDef *g, uint8_t n)         { return (g->IDR >> n) & 1u; }

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
static void delay_us(uint32_t us)
{
    uint32_t n = us * (SystemCoreClock / 1000000u), t0 = DWT->CYCCNT;
    while (DWT->CYCCNT - t0 < n) { }
}

static void adc_init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0 | ADC_CCR_TSVREFE; /* /4, VREFINT on */
    ADC1->CR1 = 0;                                                          /* 12-bit, no scan */
    ADC1->CR2 = 0;                                                          /* single, right-aligned */
    uint32_t s1 = 0, s2 = 0;
    for (uint32_t i = 0; i < 10; i++) s2 |= 4u << (3u * i);                 /* ch0-9: 84 cycles */
    for (uint32_t i = 0; i < 9;  i++) s1 |= 4u << (3u * i);                 /* ch10-18: 84 cycles */
    s1 = (s1 & ~(7u << (3u * (ADC_CH_VREFINT - 10u)))) | (7u << (3u * (ADC_CH_VREFINT - 10u)));  /* VREFINT 480 */
    ADC1->SMPR1 = s1;
    ADC1->SMPR2 = s2;
    ADC1->SQR1  = 0;
    ADC1->CR2   = ADC_CR2_ADON;
    delay_us(20);
}
static uint16_t adc_read(uint8_t ch)
{
    ADC1->SQR3 = ch;
    ADC1->SR   = 0;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    uint32_t guard = 100000;
    while (!(ADC1->SR & ADC_SR_EOC) && --guard) { }
    return (uint16_t)ADC1->DR;
}

/* VDDA from the factory VREFINT calibration -> absolute pin volts. */
static void vdda_measure(void)
{
    uint32_t s = 0;
    for (uint8_t i = 0; i < 16; i++) s += adc_read(ADC_CH_VREFINT);
    float raw = s / 16.0f;
    uint16_t cal = VREFINT_CAL_RAW;
    vdda = (raw > 100.0f && cal > 1300 && cal < 1800) ? 3.3f * cal / raw : 3.3f;
}

static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
static float absf(float x)                       { return x < 0 ? -x : x; }

static bool eval_gate(GateFn f, uint8_t bits, uint8_t n)
{
    bool all = bits == (uint8_t)((1u << n) - 1u), any = bits != 0;
    switch (f) {
        case G_NAND: return !all;
        case G_AND:  return all;
        case G_OR:   return any;
        case G_NOR:  return !any;
        case G_XOR:  return __builtin_parity(bits);
        case G_NOT:  return !any;
    }
    return false;
}

static void dut_all_hiz(void)
{
    for (uint8_t p = 1; p <= 14; p++)
        if (DUT[p].port) pin_cfg(DUT[p].port, DUT[p].pin, DUT[p].ch >= 0 ? M_AN : M_IN, PL_NONE);
}

/* Gate gsel gets `bits`, every other gate gets 0. gsel = -1 -> all 0. */
static void drive_inputs(const ICDef *ic, int8_t gsel, uint8_t bits)
{
    for (uint8_t g = 0; g < ic->ngates; g++) {
        uint8_t b = (g == gsel) ? bits : 0;
        for (uint8_t k = 0; k < ic->g[g].nin; k++) {
            const PinDef *d = &DUT[ic->g[g].in[k]];
            pin_wr(d->port, d->pin, (b >> k) & 1u);       /* set ODR before switching to output */
            pin_cfg(d->port, d->pin, M_OUT, PL_NONE);
        }
    }
}

/* Chip is always powered (no relay): keep its inputs at a defined LOW, outputs hi-Z. */
static void dut_park(const ICDef *ic) { dut_all_hiz(); drive_inputs(ic, -1, 0); }

static float idd_now(void)  { float i = absf(ina219_current_mA()) - iddOffset; return i < 0 ? 0.0f : i; }
static float read_idd(void) { return inaOk ? idd_now() : -1.0f; }

static bool ina_clean(void) { uint32_t e = ina219_errors(); bool ok = e == inaErrSeen; inaErrSeen = e; return ok; }

static void live_update(void)
{
    if (inaOk) {
        liveV = ina219_bus_V(); liveI = idd_now();
        if (!ina_clean()) {                                    /* I2C hiccup: recover bus, read again */
            i2c_recover();
            liveV = ina219_bus_V(); liveI = idd_now();
            ina_clean();
        }
    } else { liveV = vdda; liveI = -1.0f; }
    liveT = HAL_GetTick();
}

/* Overcurrent only if 3 fresh, error-free INA219 readings (5 ms apart) all exceed the limit.
 * One corrupted I2C read must never raise the alarm. */
static bool overcurrent_confirmed(void)
{
    if (!inaOk) return false;
    for (uint8_t k = 0; k < 3; k++) {
        HAL_Delay(5);                                          /* > one 4.3 ms INA219 averaging period */
        uint32_t e0 = ina219_errors();
        float i = idd_now();
        if (ina219_errors() != e0 || i <= IDD_ABORT_MA) { ina_clean(); return false; }
        liveI = i;
    }
    return true;
}

/* DUT VCC: the INA219 bus voltage, unless it disagrees with the MCU's own VDDA (same 3V3 rail) by > 0.25 V.
 * That happens when the INA219 GND pin is not solidly on the Nucleo GND: then trust VDDA and flag it. */
static bool inaVMismatch;
static float vcc_ref(void)
{
    if (!inaOk) return vdda;
    inaVMismatch = (liveV - vdda > 0.25f) || (vdda - liveV > 0.25f);
    return inaVMismatch ? vdda : liveV;
}

static float read_volts(uint8_t dutPin)
{
    const PinDef *d = &DUT[dutPin];
    if (d->ch < 0) return pin_rd(d->port, d->pin) ? vccDut : 0.0f;
    uint32_t s = 0;
    for (uint8_t i = 0; i < ADC_AVG; i++) s += adc_read((uint8_t)d->ch);
    return (float)s * vdda / (ADC_AVG * 4095.0f);
}
static float read_ratio(uint8_t dutPin) { return read_volts(dutPin) / vccDut; }

/* Returns abort reason or NULL. */
static const char *supply_check(void)
{
    live_update();
    vccDut = vcc_ref();
    if (inaOk && liveI > IDD_ABORT_MA && overcurrent_confirmed()) return "OVERCURRENT";
    if (vccDut < VCC_MIN)              return "VCC_LOW";
    if (vccDut > VCC_MAX)              return "VCC_HIGH";
    return NULL;
}

typedef struct { char m; float q; } Lvl;
static Lvl classify(float r, bool exp)
{
    Lvl L;
    L.m = (r >= VALID_HI) ? 'H' : (r <= VALID_LO) ? 'L' : 'X';
    float sr = STRONG_MARGIN / vccDut;
    float q  = exp ? (r - VALID_HI) / ((1.0f - sr) - VALID_HI)
                   : (VALID_LO - r) / (VALID_LO - sr);
    L.q = clampf(q, 0.0f, 1.0f);
    return L;
}

/* ============================== UART TX (interrupt ring, falls back to blocking) ============================== */
static uint8_t           txr[6144];
static volatile uint16_t txHead, txTail, txLen;
static volatile bool     txBusy;
static bool              uartIrq;

static void tx_kick_locked(void)
{
    if (txBusy || txHead == txTail) return;
    uint16_t end = (txHead > txTail) ? txHead : (uint16_t)sizeof txr;
    txLen  = (uint16_t)(end - txTail);
    txBusy = true;
    if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)&txr[txTail], txLen) != HAL_OK) txBusy = false;
}
static void tx_kick(void) { __disable_irq(); tx_kick_locked(); __enable_irq(); }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    if (h != &huart2) return;
    txTail = (uint16_t)((txTail + txLen) % sizeof txr);
    txBusy = false;
    tx_kick_locked();
}

static void tx_write(const char *s, size_t n)
{
    if (!uartIrq) { HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)n, 1000); return; }
    uint32_t t0 = HAL_GetTick();
    for (size_t i = 0; i < n; i++) {
        uint16_t nx = (uint16_t)((txHead + 1u) % sizeof txr);
        while (nx == txTail) {                                    /* ring full: let the ISR drain it */
            tx_kick();
            if (HAL_GetTick() - t0 > 1000) return;
        }
        txr[txHead] = (uint8_t)s[i];
        txHead = nx;
    }
    tx_kick();
}

/* ============================== JSON OUT ============================== */
/* Every value is written with a trailing ','; j_close / j_send strip it. No float printf needed. */
static char   jb[4096];
static size_t jn;

static void j_reset(void)        { jn = 0; }
static void j_putc(char c)       { if (jn < sizeof jb - 3) jb[jn++] = c; }
static void j_puts(const char *s){ while (*s) j_putc(*s++); }
static void j_key(const char *k) { if (k) { j_putc('"'); j_puts(k); j_putc('"'); j_putc(':'); } }
static void j_open(const char *k, char c) { j_key(k); j_putc(c); }
static void j_close(char c)      { if (jn && jb[jn - 1] == ',') jn--; j_putc(c); j_putc(','); }
static void j_bool(const char *k, bool v) { j_key(k); j_puts(v ? "true" : "false"); j_putc(','); }
static void j_int(const char *k, long v)
{
    char b[16]; snprintf(b, sizeof b, "%ld", v);
    j_key(k); j_puts(b); j_putc(',');
}
static void j_str(const char *k, const char *v)
{
    j_key(k); j_putc('"');
    for (; *v; v++) {
        if (*v == '"' || *v == '\\') j_putc('\\');
        if ((unsigned char)*v >= 0x20) j_putc(*v);
    }
    j_putc('"'); j_putc(',');
}
static void j_num(const char *k, float x)     /* up to 3 decimals, trailing zeros stripped */
{
    char b[24], f[4];
    int32_t v  = (int32_t)(x * 1000.0f + (x >= 0 ? 0.5f : -0.5f));
    bool    ng = v < 0;
    uint32_t a = ng ? (uint32_t)(-v) : (uint32_t)v, ip = a / 1000u, fp = a % 1000u;
    if (fp == 0) snprintf(b, sizeof b, "%s%lu", ng ? "-" : "", (unsigned long)ip);
    else {
        snprintf(f, sizeof f, "%03lu", (unsigned long)fp);
        for (int i = 2; i > 0 && f[i] == '0'; i--) f[i] = 0;
        snprintf(b, sizeof b, "%s%lu.%s", ng ? "-" : "", (unsigned long)ip, f);
    }
    j_key(k); j_puts(b); j_putc(',');
}
static void j_send(void)
{
    if (jn && jb[jn - 1] == ',') jn--;
    jb[jn++] = '\r'; jb[jn++] = '\n';
    tx_write(jb, jn);
}
static void send_err(const char *e)
{
    j_reset(); j_putc('{'); j_str("type", "error"); j_str("error", e); j_close('}'); j_send();
}

/* ============================== OLED HELPERS ============================== */
/* float -> fixed string without %f (newlib-nano). 4 rotating buffers so several fit in one line. */
static const char *F(float x, uint8_t dec)
{
    static char b[4][24];
    static uint8_t k;
    char *o = b[k++ & 3u];
    uint32_t mul = dec == 0 ? 1u : dec == 1 ? 10u : dec == 2 ? 100u : 1000u;
    int32_t  v   = (int32_t)(x * (float)mul + (x >= 0 ? 0.5f : -0.5f));
    bool     ng  = v < 0;
    uint32_t a   = ng ? (uint32_t)(-v) : (uint32_t)v;
    if (dec == 0) snprintf(o, 24, "%s%lu", ng ? "-" : "", (unsigned long)a);
    else          snprintf(o, 24, "%s%lu.%0*lu", ng ? "-" : "", (unsigned long)(a / mul), (int)(dec > 3 ? 3 : dec), (unsigned long)(a % mul));
    return o;
}
static void L(int16_t x, int16_t y, uint8_t size, const char *fmt, ...)
{
    char b[24];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    oled_text(x, y, b, size);
}
static const char *vi_line(float v, float i)
{
    static char b[24];
    if (i < 0) snprintf(b, sizeof b, "VCC %sV I n/a", F(v, 2));
    else       snprintf(b, sizeof b, "VCC %sV%s I %smA", F(v, 2), inaVMismatch ? "!" : "", F(i, 2));
    return b;
}

/* ============================== LEDs / BUZZER (non-blocking) ============================== */
/* LED result indication: GOOD = green on, BAD = red on, -1 = both off. On the idle screen the selected IC is
 * auto-checked every second, so the LEDs always show the current chip's state. */
static int8_t   ledMode = -1;

static void led_pins_init(void)          /* also called after every I2C recovery (PB6 shares I2C1 AF) */
{
    LEDP_PORT->OTYPER &= ~(1u << LEDP_PIN);
    LEDF_PORT->OTYPER &= ~(1u << LEDF_PIN);
    pin_cfg(LEDP_PORT, LEDP_PIN, M_OUT, PL_NONE);
    pin_cfg(LEDF_PORT, LEDF_PIN, M_OUT, PL_NONE);
}
static void leds(int8_t mode)
{
    ledMode = mode;
    pin_wr(LEDP_PORT, LEDP_PIN, mode == 1);
    pin_wr(LEDF_PORT, LEDF_PIN, mode == 0);
}
/* Buzzer = 2.7 kHz PWM on PB10 (TIM2_CH3, AF1): sounds on both passive and active buzzers. */
static void buzzer_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    uint32_t tclk = HAL_RCC_GetPCLK1Freq();
    if (RCC->CFGR & RCC_CFGR_PPRE1_2) tclk *= 2u;                 /* APB1 prescaler > 1 -> timer clock x2 */
    TIM2->CR1   = 0;
    TIM2->PSC   = tclk / 1000000u - 1u;                            /* 1 MHz tick */
    TIM2->ARR   = 370u - 1u;                                       /* ~2.7 kHz */
    TIM2->CCR3  = 185u;                                            /* 50 % duty */
    TIM2->CCMR2 = (TIM2->CCMR2 & ~0xFFu) | (6u << 4) | (1u << 3);  /* OC3M = PWM1, preload */
    TIM2->CCER |= 1u << 8;                                         /* CC3E */
    TIM2->EGR   = 1u;                                              /* UG */
    TIM2->CR1   = (1u << 7) | 1u;                                  /* ARPE | CEN */
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFu << 8)) | (1u << 8);    /* PB10 -> AF1 (TIM2_CH3) */
}
static void buzzer_on(bool on)
{
    if (on) pin_cfg(BUZ_PORT, BUZ_PIN, M_AF, PL_NONE);
    else  { pin_wr(BUZ_PORT, BUZ_PIN, false); pin_cfg(BUZ_PORT, BUZ_PIN, M_OUT, PL_NONE); }
}
static struct { uint8_t left; uint16_t on, off; uint32_t t; bool st; } bz;
static void buzz(uint8_t n, uint16_t on, uint16_t off)
{
    bz.left = n; bz.on = on; bz.off = off; bz.st = false; bz.t = HAL_GetTick() - off;
}
static void buzz_tick(void)
{
    uint32_t now = HAL_GetTick();
    if (bz.st) {
        if (now - bz.t >= bz.on) { buzzer_on(false); bz.st = false; bz.t = now; if (bz.left) bz.left--; }
    } else if (bz.left && now - bz.t >= bz.off) {
        buzzer_on(true); bz.st = true; bz.t = now;
    }
}

/* ============================== SCREENS ============================== */
static const char *fn_name(GateFn f)
{
    return f == G_NAND ? "NAND" : f == G_AND ? "AND" : f == G_OR ? "OR" : f == G_NOR ? "NOR" : f == G_XOR ? "XOR" : "NOT";
}


static void ui_idle(void)
{
    const ICDef *ic = &IC_DB[selIdx];
    oled_clear();
    if (i2cRecoveries) L(0, 0, 1, "IC VISION r%lu", (unsigned long)i2cRecoveries);
    else               L(0, 0, 1, "IC VISION");
    if (autoSel) L(98, 0, 1, "AUTO");
    else         L(98, 0, 1, "%u/%u", selIdx + 1, IC_COUNT);
    if (bSel.prev || bRun.prev || bMan.prev)                     /* live press indicator (wiring check) */
        L(58, 0, 1, "[%s]", bSel.prev ? "SEL" : bMan.prev ? "MAN" : "RUN");
    oled_hline(0, 9, 128);
    bool detecting = autoSel && autoState == -2;
    bool empty     = autoState == -1;
    bool badHere   = autoState == 0 && !(looksIdx >= 0 && looksIdx != (int8_t)selIdx);
    oled_text(0, 13, detecting ? "...." : (empty && autoSel) ? "----" : ic->id, 2);
    L(56, 13, 1, "74HC @3.3V");
    L(56, 23, 1, "chip: %s", auto_text());
    if (detecting)      L(0, 31, 1, "detecting chip...");
    else if (empty)     L(0, 31, 1, "no chip in socket");
    else if (autoState == 0 && looksIdx >= 0 && looksIdx != (int8_t)selIdx)
                        L(0, 31, 1, "Acts like %s %u%%", IC_DB[looksIdx].id, (unsigned)(looksScore * 100 + 0.5f));
    else if (badHere) {                                                /* which gates pass / fail */
        char okb[8] = {0}, bad[8] = {0}; uint8_t a = 0, b = 0;
        for (uint8_t g = 0; g < ic->ngates; g++) {
            if (diagGatesOk & (1u << g)) okb[a++] = (char)('1' + g); else bad[b++] = (char)('1' + g);
        }
        if (b) L(0, 31, 1, "%s ok:%s bad:%s", fn_name(ic->fn), a ? okb : "-", bad);
        else   L(0, 31, 1, "%s gates ok, pin %u", fn_name(ic->fn), diagBadPin);
    }
    else                L(0, 31, 1, "%s", ic->name);
    if (!inaOk) {
        L(0, 42, 1, "INA219 missing!");
        L(0, 53, 1, "SEL:IC   RUN:TEST");
    } else if (vcc_ref() < VCC_MIN) {
        L(0, 42, 1, "VCC %sV  LOW!", F(vcc_ref(), 2));
        L(0, 53, 1, "chk VIN+ / pin7 GND");
    } else {
        L(0, 42, 1, "%s", vi_line(vcc_ref(), liveI));
        if (badHere && diagBadGate >= 0) {
            const Gate *G = &ic->g[diagBadGate];
            if (G->nin == 1)      L(0, 53, 1, "G%d p%u>p%u FAILS", diagBadGate + 1, G->in[0], G->out);
            else if (G->nin == 2) L(0, 53, 1, "G%d p%u,%u>p%u FAILS", diagBadGate + 1, G->in[0], G->in[1], G->out);
            else                  L(0, 53, 1, "G%d p%u,%u,%u>p%u BAD", diagBadGate + 1, G->in[0], G->in[1], G->in[2], G->out);
        } else if (badHere && diagBadPin) L(0, 53, 1, "pin %u not responding", diagBadPin);
        else L(0, 53, 1, "SEL:IC RUN:TST D5:MAN");
    }
    flush();
}

static void meter_snapshot(const ICDef *ic, uint16_t highMask, PinSnap *s);

static void ui_meter(void)
{
    const ICDef *ic = &IC_DB[selIdx];
    oled_clear();
    if (snap.idd < 0) L(0, 0, 1, "M%s %sV", ic->id, F(snap.vcc, 2));
    else              L(0, 0, 1, "M%s %sV %smA", ic->id, F(snap.vcc, 2), F(snap.idd, 2));
    for (uint8_t r = 0; r < 7; r++) {
        uint8_t p = (uint8_t)(r + 1), q = (uint8_t)(14 - r);
        int16_t y = (int16_t)(8 + r * 8);
        if (p == 7) L(0, y, 1, "%2u GND", p);
        else        L(0, y, 1, "%2u %s%c", p, F(snap.v[p], 2), snap.role[p]);
        if (q == 14) L(68, y, 1, "%2u %sV", q, F(snap.v[q], 2));
        else         L(68, y, 1, "%2u %s%c", q, F(snap.v[q], 2), snap.role[q]);
    }
    oled_rect(60, 9, 2, 54, true);                      /* DIP body divider */
    flush();
}

static void ui_busy(const ICDef *ic, uint8_t n, const char *disp, uint8_t sub, uint8_t subs)
{
    if (!oledOk) return;
    oled_clear();
    L(0, 0, 1, "TESTING %s", ic->id);
    if (subs) L(80, 0, 1, "%u/%u", sub, subs);
    oled_hline(0, 9, 128);
    L(0, 13, 1, "Step %u/5", n);
    oled_text(0, 24, disp, 2);
    float frac = ((float)(n - 1) + (subs ? (float)sub / subs : 0.5f)) / 5.0f;
    oled_rect(0, 42, 128, 8, false);
    oled_rect(2, 44, (int16_t)(124 * clampf(frac, 0, 1)), 4, true);
    L(0, 54, 1, "%s", vi_line(vcc_ref(), liveI));
    flush();
}

static void ui_result(void)
{
    const ICDef *ic = &IC_DB[R.ic];
    const Tally *t  = &g_main;
    oled_clear();
    switch (resPage) {
    case 0: {
        L(0, 0, 1, "%s RESULT", ic->id); L(104, 0, 1, "1/4");
        oled_hline(0, 9, 128);
        oled_text(0, 12, R.verdict, 2);
        if (R.reason) L(0, 30, 1, "%s", R.reason);
        else          L(0, 30, 1, "E-score %s/100", F(R.elec, 1));
        L(0, 39, 1, "%s", vi_line(R.vcc, inaOk ? R.idd : -1));
        uint8_t gp = 0;
        for (uint8_t g = 0; g < ic->ngates; g++) if (t->g[g].pass) gp++;
        if (R.nLooks)          L(0, 48, 1, "Acts like %s!", R.looks[0]);
        else if (R.noIc)       L(0, 48, 1, "No chip / dead chip");
        else if (t->funcDone && gp < ic->ngates) {
            uint8_t g = 0; while (g < ic->ngates && t->g[g].pass) g++;
            const Gate *G = &ic->g[g];
            if (G->nin == 2) L(0, 48, 1, "%u/%u ok, G%u p%u,%u>p%u", gp, ic->ngates, g + 1, G->in[0], G->in[1], G->out);
            else             L(0, 48, 1, "%u/%u ok, G%u out p%u", gp, ic->ngates, g + 1, G->out);
        }
        else if (t->funcDone)  L(0, 48, 1, "Gates %u/%u  Pins %u/%u", gp, ic->ngates, t->integOk, t->integTot);
        L(0, 56, 1, "SEL:page  RUN:retest");
        break;
    }
    case 1:
        L(0, 0, 1, "GATE VECTORS"); L(104, 0, 1, "2/4");
        oled_hline(0, 9, 128);
        if (!t->funcDone) { L(0, 20, 1, "no data (aborted)"); break; }
        for (uint8_t g = 0; g < ic->ngates; g++) {
            char m[9] = {0};
            for (uint8_t v = 0; v < t->g[g].nvec && v < 8; v++) m[v] = t->g[g].vec[v].m;
            L(0, (int16_t)(11 + g * 9), 1, "G%u o%-2u %-8s %s", g + 1, ic->g[g].out, m, t->g[g].pass ? "OK" : "BAD");
        }
        break;
    case 2:
        L(0, 0, 1, "OUTPUT LEVELS V"); L(104, 0, 1, "3/4");
        oled_hline(0, 9, 128);
        if (!t->funcDone) { L(0, 20, 1, "no data (aborted)"); break; }
        for (uint8_t g = 0; g < ic->ngates; g++)
            L(0, (int16_t)(11 + g * 9), 1, "G%u o%-2u H%s L%s", g + 1, ic->g[g].out,
              R.vOH[g] < 0 ? "--" : F(R.vOH[g], 2), R.vOL[g] < 0 ? "--" : F(R.vOL[g], 2));
        break;
    default: {
        L(0, 0, 1, "PINS / SUPPLY"); L(104, 0, 1, "4/4");
        oled_hline(0, 9, 128);
        uint8_t oOk = 0, iOk = 0;
        for (uint8_t i = 0; i < t->nOuts; i++) oOk += t->outs[i].a + t->outs[i].b;
        for (uint8_t i = 0; i < t->nIns; i++)  iOk += t->ins[i].a + t->ins[i].b;
        L(0, 12, 1, "Out drive  %u/%u", oOk, t->nOuts * 2);
        L(0, 21, 1, "In follow  %u/%u", iOk, t->nIns * 2);
        L(0, 30, 1, "Strong lvl %u/%u", t->strong, t->reads);
        L(0, 39, 1, "Crosstalk  %u", t->xtalk);
        if (inaOk) L(0, 48, 1, "IDD static %smA", F(R.idd, 2));
        else       L(0, 48, 1, "IDD n/a (no INA219)");
        L(0, 56, 1, "VDDA %sV VCC %sV", F(vdda, 2), F(R.vcc, 2));
        break;
    }
    }
    flush();
}

static void ui_ident(void)
{
    oled_clear();
    L(0, 0, 1, "IDENTIFY RESULT");
    oled_hline(0, 9, 128);
    oled_text(0, 13, idN ? idM[0] : "NONE", 2);
    if (idN > 1) L(0, 31, 1, "also: %s %s", idM[1], idN > 2 ? idM[2] : "");
    if (idClaim >= 0) {
        bool found = false;
        for (uint8_t i = 0; i < idN; i++) if (!strcmp(idM[i], IC_DB[idClaim].id)) found = true;
        L(0, 41, 1, "Marked %s: %s", IC_DB[idClaim].id, found ? "MATCH" : "MISMATCH");
    }
    L(0, 56, 1, "SEL:back  RUN:test");
    flush();
}

static void ui_alert(void)
{
    oled_clear();
    L(0, 0, 1, "!!!   ALERT   !!!");
    oled_hline(0, 9, 128);
    if (alertMsg && !strcmp(alertMsg, "OVERCURRENT")) {
        oled_text(0, 13, "OVERCURR", 2);
        L(0, 32, 1, "IDD %smA", F(liveI, 1));
        L(0, 42, 1, "Chip reversed/short?");
        L(0, 52, 1, "Unplug USB. SEL=back");
    } else {
        oled_text(0, 13, alertMsg ? alertMsg : "?", 1);
        L(0, 32, 1, "%s", vi_line(vcc_ref(), liveI));
    }
    flush();
}

static void ui_manual(void)
{
    const ICDef *ic = &IC_DB[selIdx];
    const Gate  *G  = &ic->g[manGate];
    bool exp = eval_gate(ic->fn, manBits, G->nin);
    oled_clear();
    L(0, 0, 1, "MANUAL %s G%u/%u", ic->id, manGate + 1, ic->ngates);
    if (manAuto) L(110, 0, 1, "AU");
    oled_hline(0, 9, 128);
    char big[12] = {0}; uint8_t k = 0;
    for (uint8_t i = 0; i < G->nin; i++) {                        /* A1 B0 C1 */
        big[k++] = (char)('A' + i); big[k++] = (char)('0' + ((manBits >> i) & 1u));
        if (i + 1 < G->nin) big[k++] = ' ';
    }
    oled_text(0, 12, big, 2);
    char ins[16] = {0};
    if (G->nin == 1)      snprintf(ins, sizeof ins, "p%u", G->in[0]);
    else if (G->nin == 2) snprintf(ins, sizeof ins, "p%u,%u", G->in[0], G->in[1]);
    else                  snprintf(ins, sizeof ins, "p%u,%u,%u", G->in[0], G->in[1], G->in[2]);
    L(0, 30, 1, "%s>p%u %sV %c", ins, G->out, F(manV, 2), manM);
    L(0, 39, 1, "%s expect %c  %s", fn_name(ic->fn), exp ? 'H' : 'L', manOk ? "OK" : "BAD");
    char e[9] = {0};
    uint8_t nv = (uint8_t)(1u << G->nin);
    for (uint8_t v = 0; v < nv && v < 8; v++) e[v] = eval_gate(ic->fn, v, G->nin) ? 'H' : 'L';
    L(0, 48, 1, "e:%s g:%s", e, manGot);
    L(0, 57, 1, "D5:IN SEL:G RUN:AUTO");
    flush();
}

static void ui_simple(void);
static void ui_render(void)
{
    if (!oledOk) return;
    switch (ui) {
        case UI_IDLE:   ui_idle();   break;
        case UI_METER:  ui_meter();  break;
        case UI_RESULT: ui_result(); break;
        case UI_IDENT:  ui_ident();  break;
        case UI_ALERT:  ui_alert();  break;
        case UI_MANUAL: ui_manual(); break;
        case UI_SIMPLE: ui_simple(); break;
        default: break;
    }
}

/* One test step: OLED progress + UART event. */
static void step(const ICDef *ic, uint8_t n, const char *disp, const char *name, uint8_t sub, uint8_t subs)
{
    if (!quietSteps) ui_busy(ic, n, disp, sub, subs);
    j_reset(); j_putc('{');
    j_str("type", "step"); j_str("ic", ic->id); j_int("n", n); j_int("of", 5); j_str("name", name);
    if (subs) { j_int("sub", sub); j_int("subs", subs); }
    j_close('}'); j_send();
}

/* ============================== TESTS ============================== */
static void tally_reset(Tally *t) { memset(t, 0, sizeof *t); }

static bool run_functional(const ICDef *ic, Tally *t, const char **abortMsg, bool report)
{
    for (uint8_t g = 0; g < ic->ngates; g++) { t->g[g].nvec = 0; t->g[g].pass = true; }
    for (uint8_t g = 0; g < ic->ngates; g++) {
        if (report) step(ic, 2, "TRUTH TBL", "truth_table", (uint8_t)(g + 1), ic->ngates);
        uint8_t nv = (uint8_t)(1u << ic->g[g].nin);
        for (uint8_t v = 0; v < nv; v++) {
            drive_inputs(ic, (int8_t)g, v);
            delay_us(SETTLE_US);
            float idd = skipIdd ? 0.0f : read_idd();
            if (idd > IDD_ABORT_MA && overcurrent_confirmed()) { *abortMsg = "OVERCURRENT"; t->iddMax = liveI; return false; }
            if (idd > IDD_ABORT_MA) idd = 0;                   /* unconfirmed spike = bad read, ignore */
            if (idd > t->iddMax) t->iddMax = idd;
            for (uint8_t h = 0; h < ic->ngates; h++) {
                bool  exp = eval_gate(ic->fn, h == g ? v : 0, ic->g[h].nin);
                float r   = read_ratio(ic->g[h].out);
                Lvl   Lv  = classify(r, exp);
                bool  ok  = Lv.m == (exp ? 'H' : 'L');
                t->reads++;
                if (ok) { t->correct++; t->qsum += Lv.q; if (Lv.q >= 1.0f) t->strong++; }
                else    { t->g[h].pass = false; if (h != g) t->xtalk++; }
                if (h == g) {
                    VecRes *e = &t->g[g].vec[t->g[g].nvec++];
                    e->x = v; e->exp = exp; e->v = r * vccDut; e->m = Lv.m; e->ok = ok;
                }
            }
        }
    }
    t->funcDone = true;
    return true;
}

/* Outputs must read the same under pull-up and pull-down (actively driven).
 * Inputs must follow pull-up and pull-down (not shorted / leaky). */
static void run_integrity(const ICDef *ic, Tally *t)
{
    t->nOuts = 0; t->nIns = 0;
    for (uint8_t g = 0; g < ic->ngates; g++) {
        const Gate *G = &ic->g[g];
        int8_t vHi = -1, vLo = -1;
        for (uint8_t v = 0; v < (1u << G->nin); v++) {
            bool e = eval_gate(ic->fn, v, G->nin);
            if (e && vHi < 0)  vHi = (int8_t)v;
            if (!e && vLo < 0) vLo = (int8_t)v;
        }
        const PinDef *o = &DUT[G->out];
        bool a, b, okHi, okLo;
        drive_inputs(ic, (int8_t)g, (uint8_t)vHi);
        pin_cfg(o->port, o->pin, M_IN, PL_UP);   delay_us(PULL_SETTLE_US); a = pin_rd(o->port, o->pin);
        pin_cfg(o->port, o->pin, M_IN, PL_DOWN); delay_us(PULL_SETTLE_US); b = pin_rd(o->port, o->pin);
        okHi = a == b;
        drive_inputs(ic, (int8_t)g, (uint8_t)vLo);
        pin_cfg(o->port, o->pin, M_IN, PL_UP);   delay_us(PULL_SETTLE_US); a = pin_rd(o->port, o->pin);
        pin_cfg(o->port, o->pin, M_IN, PL_DOWN); delay_us(PULL_SETTLE_US); b = pin_rd(o->port, o->pin);
        okLo = a == b;
        pin_cfg(o->port, o->pin, o->ch >= 0 ? M_AN : M_IN, PL_NONE);
        t->integTot += 2; t->integOk  += okHi + okLo;
        t->outTot   += 2; t->outFloat += !okHi + !okLo;
        t->outs[t->nOuts++] = (PinRes){ G->out, okHi, okLo };
    }
    drive_inputs(ic, -1, 0);
    for (uint8_t g = 0; g < ic->ngates; g++) {
        for (uint8_t k = 0; k < ic->g[g].nin; k++) {
            uint8_t p = ic->g[g].in[k];
            const PinDef *d = &DUT[p];
            pin_cfg(d->port, d->pin, M_IN, PL_UP);   delay_us(PULL_SETTLE_US); bool u  = pin_rd(d->port, d->pin);
            pin_cfg(d->port, d->pin, M_IN, PL_DOWN); delay_us(PULL_SETTLE_US); bool dn = !pin_rd(d->port, d->pin);
            pin_wr(d->port, d->pin, false); pin_cfg(d->port, d->pin, M_OUT, PL_NONE);
            t->integTot += 2; t->integOk += u + dn;
            t->ins[t->nIns++] = (PinRes){ p, u, dn };
        }
    }
    t->integDone = true;
}

/* Functional + drive check for one DB entry (identify / looks_like). Leaves pins parked for `ic`. */
static bool quick_match(const ICDef *ic)
{
    Tally *t = &g_scratch;
    tally_reset(t);
    const char *ab = NULL;
    dut_park(ic);
    delay_us(500);
    vccDut = vcc_ref();
    if (vccDut < VCC_MIN) return false;
    bool ok = false;
    if (run_functional(ic, t, &ab, false) && t->correct == t->reads) {
        run_integrity(ic, t);                     /* empty socket can read "correct" by luck */
        ok = t->outFloat == 0;
    }
    if (ab) dut_all_hiz();
    return ok;
}

static uint8_t collect_matches(const char **m, const ICDef *forUi)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < IC_COUNT; i++) {
        step(forUi, 5, "IDENTIFY", "identify", (uint8_t)(i + 1), IC_COUNT);
        if (quick_match(&IC_DB[i])) m[n++] = IC_DB[i].id;
    }
    dut_park(&IC_DB[selIdx]);
    return n;
}

/* All 14 pins: outputs measured by ADC, inputs = level driven by the MCU. */
static void meter_snapshot(const ICDef *ic, uint16_t highMask, PinSnap *s)
{
    bool isIn[15] = {false};
    for (uint8_t g = 0; g < ic->ngates; g++) for (uint8_t k = 0; k < ic->g[g].nin; k++) isIn[ic->g[g].in[k]] = true;
    dut_park(ic);
    for (uint8_t p = 1; p <= 14; p++) if (isIn[p] && (highMask & (1u << p))) pin_wr(DUT[p].port, DUT[p].pin, true);
    delay_us(1000);
    live_update();
    vccDut  = vcc_ref();
    s->vcc  = vccDut;
    s->idd  = liveI;
    for (uint8_t p = 1; p <= 14; p++) {
        if (p == 7)        { s->role[p] = 'G'; s->v[p] = 0; }
        else if (p == 14)  { s->role[p] = 'V'; s->v[p] = vccDut; }
        else if (isIn[p])  { s->role[p] = 'i'; s->v[p] = (highMask & (1u << p)) ? vccDut : 0; }
        else               { s->role[p] = 'o'; s->v[p] = read_volts(p); }
    }
    dut_park(ic);
}

static void j_pins(const PinSnap *s)
{
    j_open("pins", '[');
    for (uint8_t p = 1; p <= 14; p++) {
        j_open(NULL, '{');
        j_int("p", p);
        const char *role = s->role[p] == 'G' ? "gnd" : s->role[p] == 'V' ? "vcc" : s->role[p] == 'i' ? "in" : "out";
        j_str("role", role);
        j_num("v", s->v[p]);
        if (s->role[p] == 'i') j_bool("set", true);
        j_close('}');
    }
    j_close(']');
}

/* Number of socket pins (1-13) actively driven by a chip: each must hold its level against both pulls.
 * 0 = empty socket (or dead chip). Leaves all DUT pins hi-Z. */
static uint8_t socket_driven(void)
{
    dut_all_hiz();
    uint8_t driven = 0;
    for (uint8_t p = 1; p <= 13; p++) {
        if (!DUT[p].port) continue;
        const PinDef *d = &DUT[p];
        pin_cfg(d->port, d->pin, M_IN, PL_UP);   delay_us(PULL_SETTLE_US); bool a = pin_rd(d->port, d->pin);
        pin_cfg(d->port, d->pin, M_IN, PL_DOWN); delay_us(PULL_SETTLE_US); bool b = pin_rd(d->port, d->pin);
        pin_cfg(d->port, d->pin, d->ch >= 0 ? M_AN : M_IN, PL_NONE);
        if (a == b) driven++;
    }
    return driven;
}

/* ============================== AUTO CHECK (idle) ============================== */
/* Quick functional + drive check of the selected IC every second while idle: LEDs follow the chip live. */
/* Truth-table match of the chip against one IC type, using only each gate's OWN output rows
 * (XOR vs OR = 75 %, XOR vs AND = 25 %, true type = 100 %). 0 on overcurrent. */
static float quick_score(const ICDef *ic)
{
    Tally *t = &g_scratch;
    tally_reset(t);
    const char *ab = NULL;
    dut_park(ic);
    delay_us(300);
    vccDut = vcc_ref();
    run_functional(ic, t, &ab, false);
    if (ab) { dut_all_hiz(); return 0.0f; }
    uint16_t ok = 0, tot = 0;
    for (uint8_t g = 0; g < ic->ngates; g++)
        for (uint8_t v = 0; v < t->g[g].nvec; v++) { tot++; ok += t->g[g].vec[v].ok; }
    return tot ? (float)ok / tot : 0.0f;
}

/* Best-matching IC type (>= 70 % and strictly better than the runner-up), -1 if none. */
static int8_t best_match(float *score)
{
    float best = 0, second = 0; int8_t bi = -1;
    skipIdd = true;
    for (uint8_t i = 0; i < IC_COUNT; i++) {
        float sc = quick_score(&IC_DB[i]);
        if (sc > best)        { second = best; best = sc; bi = (int8_t)i; }
        else if (sc > second) { second = sc; }
    }
    skipIdd = false;
    dut_park(&IC_DB[selIdx]);
    *score = best;
    return (best >= 0.70f && best > second) ? bi : -1;
}

/* GOOD / BAD / EMPTY for the selected IC type. Leaves the inputs parked. */
static int8_t eval_selected(void)
{
    const ICDef *ic = &IC_DB[selIdx];
    Tally *t = &g_scratch;
    tally_reset(t);
    const char *ab = NULL;
    dut_park(ic);
    delay_us(300);
    vccDut = vcc_ref();
    skipIdd = true;
    bool fOk = run_functional(ic, t, &ab, false);
    skipIdd = false;
    run_integrity(ic, t);
    diagGatesOk = 0; diagBadGate = -1; diagBadPin = 0;
    evalSig = (uint32_t)t->correct | ((uint32_t)t->integOk << 16);
    for (uint8_t g = 0; g < ic->ngates; g++) {
        if (t->g[g].pass) diagGatesOk |= (uint8_t)(1u << g);
        else if (diagBadGate < 0) diagBadGate = (int8_t)g;
    }
    for (uint8_t i = 0; i < t->nOuts && !diagBadPin; i++) if (!t->outs[i].a || !t->outs[i].b) diagBadPin = t->outs[i].pin;
    for (uint8_t i = 0; i < t->nIns  && !diagBadPin; i++) if (!t->ins[i].a  || !t->ins[i].b)  diagBadPin = t->ins[i].pin;
    int8_t st;
    if (t->outTot && t->outFloat == t->outTot)                           st = socket_driven() ? 0 : -1;   /* chip there, wrong type = BAD */
    else if (fOk && t->correct == t->reads && t->integOk == t->integTot) st = 1;
    else                                                                st = 0;
    dut_park(ic);
    return st;
}

static void auto_check(void)
{
    int8_t st = eval_selected();
    static uint32_t lastSig;
    if (st == 0 && evalSig != lastSig) idTried = false;          /* chip behaves differently now: re-identify */
    lastSig = evalSig;
    if (st == 0 && !idTried) {                                   /* chip present but not this type: what is it? */
        idTried = true;
        looksIdx = best_match(&looksScore);
        if (autoSel && looksIdx >= 0 && looksIdx != (int8_t)selIdx) {   /* AUTO: switch to best match */
            selIdx = (uint8_t)looksIdx;
            st = eval_selected();
        }
    }
    if (st != 0) { idTried = false; looksIdx = -1; }

    /* Debounce: a GOOD/BAD/EMPTY change is applied only after 2 agreeing checks (1 s apart), so one noisy
     * breadboard reading can't flip the LEDs. A new IC type (AUTO switch / selection) is applied at once. */
    static int8_t pend = -3, lastIc = -1;
    static uint8_t pendN;
    bool icChanged = (int8_t)selIdx != lastIc;
    if (st == autoState && !icChanged) { pendN = 0; return; }
    if (st == pend) pendN++; else { pend = st; pendN = 1; }
    if (autoState != -2 && !icChanged && pendN < 2) return;
    pendN = 0; pend = -3; lastIc = (int8_t)selIdx;

    const ICDef *ic = &IC_DB[selIdx];
    autoState = st;
    leds(st == 1 ? 1 : st == 0 ? 0 : -1);
    j_reset(); j_putc('{');
    j_str("type", "status"); j_str("ic", ic->id); j_bool("auto", autoSel);
    j_str("state", st == 1 ? "good" : st == 0 ? "bad" : "empty");
    if (looksIdx >= 0 && looksIdx != (int8_t)selIdx) { j_str("looks_like", IC_DB[looksIdx].id); j_num("match", looksScore * 100); }
    if (st == 0) {
        j_open("gates_ok", '['); for (uint8_t g = 0; g < ic->ngates; g++) j_bool(NULL, (diagGatesOk >> g) & 1u); j_close(']');
        if (diagBadPin) j_int("bad_pin", diagBadPin);
    }
    j_close('}'); j_send();
}

/* ============================== MANUAL MODE ============================== */
/* Drive the chosen gate's inputs with manBits (other gates LOW), read its output, compare with the truth table. */
static void manual_apply(bool report)
{
    const ICDef *ic = &IC_DB[selIdx];
    const Gate  *G  = &ic->g[manGate];
    drive_inputs(ic, (int8_t)manGate, manBits);
    delay_us(SETTLE_US);
    vccDut = vcc_ref();
    float r = read_ratio(G->out);
    bool exp = eval_gate(ic->fn, manBits, G->nin);
    Lvl lv = classify(r, exp);
    manV = r * vccDut; manM = lv.m;
    manOk = lv.m == (exp ? 'H' : 'L');
    manGot[manBits & 7u] = lv.m;
    leds(manOk ? 1 : 0);
    if (!report) return;
    j_reset(); j_putc('{');
    j_str("type", "manual"); j_str("ic", ic->id); j_str("fn", fn_name(ic->fn)); j_int("gate", manGate + 1);
    j_open("in_pins", '['); for (uint8_t k = 0; k < G->nin; k++) j_int(NULL, G->in[k]); j_close(']');
    j_open("in", '[');      for (uint8_t k = 0; k < G->nin; k++) j_int(NULL, (manBits >> k) & 1u); j_close(']');
    j_int("out_pin", G->out); j_num("v", manV);
    char m[2] = { manM, 0 }; j_str("m", m); j_str("exp", exp ? "H" : "L"); j_bool("ok", manOk);
    j_close('}'); j_send();
}

static void manual_reset_got(void)
{
    uint8_t nv = (uint8_t)(1u << IC_DB[selIdx].g[manGate].nin);
    memset(manGot, 0, sizeof manGot);
    for (uint8_t v = 0; v < nv && v < 8; v++) manGot[v] = '?';
}

static void manual_enter(void)
{
    ui = UI_MANUAL; manGate = 0; manBits = 0; manAuto = false; manT = HAL_GetTick();
    manual_reset_got();
    manual_apply(true);
    ui_render();
}

static void manual_exit(void)
{
    manAuto = false;
    dut_park(&IC_DB[selIdx]);
    leds(-1);
    ui = UI_IDLE; autoState = -2; autoT = 0;
    live_update();
    ui_render();
}

static void manual_next_input(void)
{
    uint8_t nv = (uint8_t)(1u << IC_DB[selIdx].g[manGate].nin);
    manBits = (uint8_t)((manBits + 1u) % nv);
    manual_apply(true);
    ui_render();
}

/* ============================== COMMANDS ============================== */
/* Accepts "7400", "74HC00", "SN74HC08N", "IC 7486" ... */
static int find_ic(const char *s)
{
    if (!s) return -1;
    const char *p = strstr(s, "74");
    if (!p) return -1;
    p += 2;
    while (*p && isalpha((unsigned char)*p)) p++;
    char num[4] = {0}; uint8_t n = 0;
    while (*p && isdigit((unsigned char)*p) && n < 3) num[n++] = *p++;
    char id[8]; snprintf(id, sizeof id, "74%s", num);
    for (uint8_t i = 0; i < IC_COUNT; i++) if (!strcmp(IC_DB[i].id, id)) return i;
    return -1;
}

static void cmd_test(uint8_t idx)
{
    const ICDef *ic = &IC_DB[idx];
    uint32_t t0 = HAL_GetTick();
    ui = UI_BUSY;
    leds(-1);

    Tally *t = &g_main;
    tally_reset(t);
    memset(&R, 0, sizeof R);
    R.ic = idx;

    /* 1. supply */
    dut_park(ic);
    delay_us(500);
    step(ic, 1, "SUPPLY", "supply", 0, 0);
    const char *ab = supply_check();
    R.vcc = vccDut;

    /* 2-4. truth table, pin integrity, static current */
    if (!ab && run_functional(ic, t, &ab, true)) {
        step(ic, 3, "PIN CHECK", "pin_integrity", 0, 0);
        run_integrity(ic, t);
        step(ic, 4, "CURRENT", "supply_current", 0, 0);
        drive_inputs(ic, -1, 0);
        HAL_Delay(10);                                   /* INA219 8-sample average settles */
        float i = read_idd();
        if (i > IDD_ABORT_MA && !overcurrent_confirmed()) i = read_idd();   /* re-read a suspicious value */
        R.idd = i;
        if (i > t->iddMax) t->iddMax = i;
        if (i > IDD_ABORT_MA && overcurrent_confirmed()) ab = "OVERCURRENT";
    }
    if (ab) dut_all_hiz(); else dut_park(ic);
    if (inaOk && !ina_clean()) i2c_recover();

    /* 5. score */
    step(ic, 5, "SCORE", "score", 0, 0);
    float fFunc  = t->reads    ? (float)t->correct / t->reads    : 0;
    float fLevel = t->reads    ? t->qsum / t->reads              : 0;
    float fInteg = t->integTot ? (float)t->integOk / t->integTot : 0;
    float fSupp  = (t->iddMax <= IDD_WARN_MA) ? 1.0f : (t->iddMax <= IDD_FAIL_MA) ? 0.5f : 0.0f;
    float wsum   = W_FUNC + W_LEVEL + W_INTEG + (inaOk ? W_SUPPLY : 0);
    float elec   = 100.0f * (W_FUNC * fFunc + W_LEVEL * fLevel + W_INTEG * fInteg + (inaOk ? W_SUPPLY * fSupp : 0)) / wsum;
    bool  noIc   = !ab && t->outTot && t->outFloat == t->outTot;
    if (ab || noIc) elec = 0;

    const char *verdict;
    if (ab)        verdict = "FAIL";
    else if (noIc) verdict = "NO_IC";
    else if (fFunc == 1 && fInteg == 1 && t->strong == t->reads && (!inaOk || t->iddMax <= IDD_WARN_MA)) verdict = "PASS";
    else if (fFunc == 1 && (!inaOk || t->iddMax <= IDD_FAIL_MA))                                         verdict = "MARGINAL";
    else           verdict = "FAIL";
    if (!strcmp(verdict, "FAIL") && elec > FAIL_CAP) elec = FAIL_CAP;

    for (uint8_t g = 0; g < 6; g++) { R.vOH[g] = -1; R.vOL[g] = -1; }
    if (t->funcDone)
        for (uint8_t g = 0; g < ic->ngates; g++)
            for (uint8_t v = 0; v < t->g[g].nvec; v++) {
                const VecRes *e = &t->g[g].vec[v];
                if (e->exp) { if (R.vOH[g] < 0 || e->v < R.vOH[g]) R.vOH[g] = e->v; }
                else        { if (e->v > R.vOL[g]) R.vOL[g] = e->v; }
            }

    bool doLooks = !ab && !noIc && fFunc < 1;
    if (doLooks) R.nLooks = collect_matches(R.looks, ic);   /* wrong / remarked part? */

    if (!ab) meter_snapshot(ic, 0, &snap);
    if (!R.idd && inaOk) R.idd = t->iddMax;

    R.valid = true; R.verdict = verdict; R.reason = ab; R.elec = elec; R.noIc = noIc;
    R.fFunc = fFunc; R.fLevel = fLevel; R.fInteg = fInteg; R.fSupp = fSupp;

    /* JSON result */
    j_reset(); j_putc('{');
    j_str("type", "result"); j_str("fw", FW_VERSION); j_str("ic", ic->id); j_str("name", ic->name);
    j_bool("ina", inaOk);
    j_num("vcc", R.vcc); j_num("vdda", vdda);
    if (t->funcDone) {
        j_open("gates", '[');
        for (uint8_t g = 0; g < ic->ngates; g++) {
            const Gate *G = &ic->g[g];
            j_open(NULL, '{');
            j_int("g", g + 1);
            j_open("in", '['); for (uint8_t k = 0; k < G->nin; k++) j_int(NULL, G->in[k]); j_close(']');
            j_int("out", G->out);
            j_open("vec", '[');
            for (uint8_t v = 0; v < t->g[g].nvec; v++) {
                const VecRes *e = &t->g[g].vec[v];
                char m[2] = { e->m, 0 };
                j_open(NULL, '{');
                j_int("x", e->x); j_int("exp", e->exp); j_num("v", e->v); j_str("m", m); j_bool("ok", e->ok);
                j_close('}');
            }
            j_close(']');
            j_bool("pass", t->g[g].pass);
            j_close('}');
        }
        j_close(']');
    }
    if (t->integDone) {
        j_open("integrity", '{');
        j_open("outputs", '[');
        for (uint8_t i = 0; i < t->nOuts; i++) {
            j_open(NULL, '{'); j_int("pin", t->outs[i].pin); j_bool("hi", t->outs[i].a); j_bool("lo", t->outs[i].b); j_close('}');
        }
        j_close(']');
        j_open("inputs", '[');
        for (uint8_t i = 0; i < t->nIns; i++) {
            j_open(NULL, '{'); j_int("pin", t->ins[i].pin); j_bool("pu", t->ins[i].a); j_bool("pd", t->ins[i].b); j_close('}');
        }
        j_close(']');
        j_close('}');
    }
    if (!ab) j_pins(&snap);
    j_num("idd_ma", inaOk ? t->iddMax : -1);
    j_open("stats", '{');
    j_int("reads", t->reads); j_int("correct", t->correct); j_int("strong", t->strong); j_int("xtalk", t->xtalk);
    j_int("integ_ok", t->integOk); j_int("integ_tot", t->integTot);
    j_close('}');
    j_open("score", '{');
    j_num("functional", 100 * fFunc); j_num("levels", 100 * fLevel); j_num("integrity", 100 * fInteg);
    j_num("supply", inaOk ? 100 * fSupp : -1); j_num("electrical", elec); j_num("weight", 0.8f);
    j_close('}');
    j_str("verdict", verdict);
    if (ab) j_str("reason", ab);
    if (doLooks) { j_open("looks_like", '['); for (uint8_t i = 0; i < R.nLooks; i++) j_str(NULL, R.looks[i]); j_close(']'); }
    j_int("ms", (long)(HAL_GetTick() - t0));
    j_close('}');
    j_send();

    if (ab && !strcmp(ab, "OVERCURRENT")) { alertMsg = ab; ui = UI_ALERT; }
    else { ui = UI_RESULT; resPage = 0; }
    if (quietSteps) return;                                  /* SIMPLE_UI: caller shows the result */
    bool pass = !strcmp(verdict, "PASS");
    leds(pass ? 1 : 0);                                      /* green = good, red = bad (both solid) */
    if (pass) buzz(1, 150, 60); else buzz(3, 80, 80);
    ui_render();
}

static void cmd_identify(const char *claimed)
{
    const ICDef *cur = &IC_DB[selIdx];
    ui = UI_BUSY;
    leds(-1);
    idN = collect_matches(idM, cur);
    idClaim = find_ic(claimed);

    j_reset(); j_putc('{');
    j_str("type", "identify");
    j_open("matches", '['); for (uint8_t i = 0; i < idN; i++) j_str(NULL, idM[i]); j_close(']');
    if (claimed) {
        j_str("claimed", idClaim >= 0 ? IC_DB[idClaim].id : claimed);
        bool found = false;
        for (uint8_t i = 0; i < idN; i++) if (idClaim >= 0 && !strcmp(idM[i], IC_DB[idClaim].id)) found = true;
        j_bool("mismatch", !found);
    }
    j_close('}');
    j_send();

    ui = UI_IDENT;
    buzz(idN ? 1 : 3, 100, 80);
    ui_render();
}

/* ============================== RESET CAUSE ============================== */
static const char *resetCause = "?";
static void reset_cause_read(void)
{
    uint32_t c = RCC->CSR;
    resetCause = (c & RCC_CSR_IWDGRSTF) ? "IWDG" : (c & RCC_CSR_WWDGRSTF) ? "WWDG" : (c & RCC_CSR_LPWRRSTF) ? "LPWR" :
                 (c & RCC_CSR_SFTRSTF)  ? "SOFT" : (c & RCC_CSR_PORRSTF)  ? "POWER" : (c & RCC_CSR_BORRSTF)  ? "BROWNOUT" :
                 (c & RCC_CSR_PINRSTF)  ? "RESET-BTN" : "?";
    RCC->CSR |= RCC_CSR_RMVF;                                  /* clear flags for next boot */
}

/* ============================== SELF TEST (boot) ============================== */
static void selftest(void)
{
    /* LEDs + buzzer on so the user can see/hear them (SIMPLE_UI does its own 500 ms pulse) */
    if (!SIMPLE_UI) { pin_wr(LEDP_PORT, LEDP_PIN, true); pin_wr(LEDF_PORT, LEDF_PIN, true); buzzer_on(true); }

    vdda_measure();
    live_update();
    vccDut = vcc_ref();
    bool vccOk = vccDut >= VCC_MIN && vccDut <= VCC_MAX;
    bool iddOk = inaOk && liveI <= IDD_FAIL_MA;
    bool selOk = pin_rd(BTN_SEL_PORT, BTN_SEL_PIN);          /* released = HIGH; LOW = stuck / shorted legs */
    bool runOk = pin_rd(BTN_RUN_PORT, BTN_RUN_PIN);

    /* socket: count pins that hold their level against both pulls (driven outputs) */
    uint8_t driven = socket_driven();
    if (inaOk && !driven) {                                    /* empty socket: whatever INA219 reads is offset */
        float sum = 0;
        for (uint8_t k = 0; k < 8; k++) { HAL_Delay(5); sum += absf(ina219_current_mA()); }
        iddOffset = sum / 8.0f;
        if (iddOffset > 1.0f) iddOffset = 0;                   /* not plausible as offset: ignore */
        live_update();
    }
    dut_park(&IC_DB[selIdx]);

    if (!SIMPLE_UI) HAL_Delay(150);
    buzzer_on(false);

    if (oledOk && !SIMPLE_UI) {
        oled_clear();
        L(0, 0,  1, "SELF TEST v4.1 %s", resetCause);
        L(0, 8,  1, "OLED       OK");
        L(0, 16, 1, "INA219     %s", inaOk ? "OK" : "MISSING");
        L(0, 24, 1, "VDDA       %sV", F(vdda, 2));
        if (inaOk && inaVMismatch) L(0, 32, 1, "INA %sV: INA GND?", F(liveV, 2));
        else if (inaOk) L(0, 32, 1, "VCC pin14  %sV %s", F(liveV, 2), vccOk ? "OK" : "BAD");
        else       L(0, 32, 1, "VCC pin14  n/a");
        if (inaOk) L(0, 40, 1, "IDD %smA %s", F(liveI, 2), iddOk ? "OK" : "HIGH");
        else       L(0, 40, 1, "IDD n/a");
        L(0, 48, 1, "BTN SEL %s RUN %s", selOk ? "OK" : "STK", runOk ? "OK" : "STK");
        L(0, 56, 1, "SOCKET %s (%u drv)", driven ? "chip" : "empty", driven);
        flush();
    }

    j_reset(); j_putc('{');
    j_str("type", "selftest"); j_str("fw", FW_VERSION);
    j_bool("oled", oledOk); j_bool("ina", inaOk); j_num("vdda", vdda);
    j_num("vcc", inaOk ? liveV : -1); j_bool("vcc_ok", vccOk); j_bool("ina_gnd_suspect", inaVMismatch);
    j_num("idd_ma", inaOk ? liveI : -1); j_bool("idd_ok", iddOk);
    j_bool("btn_sel_ok", selOk); j_bool("btn_run_ok", runOk);
    j_str("socket", driven ? "chip" : "empty"); j_int("driven_pins", driven);
    if (inaOk && !vccOk) j_str("hint", "VCC at socket pin 14 out of 3.0-3.6 V: check 3V3->INA219 VIN+, VIN-->pin14, pin7->GND");
    j_bool("ok", oledOk && inaOk && vccOk && iddOk && selOk && runOk);
    j_close('}'); j_send();

    if (SIMPLE_UI) { dut_park(&IC_DB[selIdx]); return; }   /* SIMPLE_UI: no self-test screen */
    HAL_Delay(1800);
    leds(-1);
    autoState = -2; autoT = 0;
    if (inaOk && liveI > IDD_ABORT_MA && overcurrent_confirmed()) { dut_all_hiz(); alertMsg = "OVERCURRENT"; ui = UI_ALERT; }
    else ui = UI_IDLE;
    ui_render();
}

/* ============================== SIMPLE UI (SIMPLE_UI = 1) ============================== */
/* Flow: reset -> LEDs + buzzer 500 ms -> "IC CHECKER" -> "Identifying IC..." -> result screen.
 * Result: IC number, GOOD/BAD, IC name, score /100, live VCC + current, [RETEST]. Any button = retest. */
static void ui_title(void)
{
    L(0, 1, 1, "IC CHECKER");
    oled_rect(84, 0, 44, 11, false);
    L(88, 2, 1, "RETEST");
    oled_hline(0, 12, 128);
}

static void ui_checking(const char *line1, const char *line2)
{
    if (!oledOk) return;
    oled_clear();
    ui_title();
    oled_text(0, 18, "CHECKING", 2);
    L(0, 38, 1, "%s", line1);
    if (line2) L(0, 47, 1, "%s", line2);
    L(0, 56, 1, "%s", vi_line(vcc_ref(), liveI));
    flush();
}

static void ui_simple(void)
{
    const ICDef *ic = &IC_DB[selIdx];
    oled_clear();
    ui_title();
    const char *id = (sState == SS_GOOD || sState == SS_BAD) ? ic->id : sState == SS_EMPTY ? "----" : "????";
    oled_text(0, 16, id, 2);
    const char *st = sState == SS_GOOD ? "GOOD" : sState == SS_EMPTY ? "EMPTY" : "BAD";
    oled_text((int16_t)(128 - 12 * (int16_t)strlen(st)), 16, st, 2);
    switch (sState) {
    case SS_GOOD:
        L(0, 34, 1, "%s", ic->name);
        L(0, 44, 1, "Score %s/100", F(R.elec, 0));
        break;
    case SS_BAD:
        L(0, 34, 1, "%s", ic->name);
        if (sBadGate >= 0) L(0, 44, 1, "Score %s/100 G%d BAD", F(R.elec, 0), sBadGate + 1);
        else               L(0, 44, 1, "Score %s/100", F(R.elec, 0));
        break;
    case SS_EMPTY:   L(0, 34, 1, "No IC in socket");   L(0, 44, 1, "Insert IC, press btn"); break;
    case SS_UNKNOWN: L(0, 34, 1, "Unknown / dead IC"); L(0, 44, 1, "Score 0/100"); break;
    case SS_ERROR:   L(0, 34, 1, "%s", sReason ? sReason : "Error"); L(0, 44, 1, "Score 0/100"); break;
    default: break;
    }
    L(0, 55, 1, "%s", vi_line(vcc_ref(), liveI));
    flush();
}

static void simple_status_json(void)
{
    j_reset(); j_putc('{');
    j_str("type", "check");
    j_str("state", sState == SS_GOOD ? "good" : sState == SS_BAD ? "bad" : sState == SS_EMPTY ? "empty" :
                   sState == SS_UNKNOWN ? "unknown" : "error");
    if (sState == SS_GOOD || sState == SS_BAD) { j_str("ic", IC_DB[selIdx].id); j_str("name", IC_DB[selIdx].name); j_num("score", R.elec); }
    if (sBadGate >= 0) j_int("bad_gate", sBadGate + 1);
    if (sReason) j_str("reason", sReason);
    j_num("vcc", vcc_ref()); j_num("idd_ma", liveI);
    j_close('}'); j_send();
}

/* Identify the chip, run the full scored test on that type, show one result screen. */
static void check_ic(void)
{
    ui = UI_BUSY;
    leds(-1);
    sReason = NULL; sBadGate = -1;
    ui_checking("Identifying IC...", NULL);

    const char *ab = supply_check();
    if (ab && !strcmp(ab, "OVERCURRENT")) {
        dut_all_hiz(); alertMsg = ab; ui = UI_ALERT; leds(0); buzz(5, 60, 60); ui_render(); return;
    }
    if (ab) {                                                  /* VCC_LOW / VCC_HIGH */
        sState = SS_ERROR; sReason = !strcmp(ab, "VCC_LOW") ? "VCC too low: wiring" : "VCC too high";
    } else if (!socket_driven()) {
        sState = SS_EMPTY;
    } else {
        float sc; int8_t bi = best_match(&sc);
        if (bi < 0) { sState = SS_UNKNOWN; looksScore = sc; }
        else {
            selIdx = (uint8_t)bi;
            char l2[24]; snprintf(l2, sizeof l2, "Found %s, testing", IC_DB[selIdx].id);
            ui_checking("Identifying IC...", l2);
            quietSteps = true;
            cmd_test(selIdx);
            quietSteps = false;
            if (ui == UI_ALERT) { leds(0); buzz(5, 60, 60); ui_render(); return; }
            bool good = R.verdict && (!strcmp(R.verdict, "PASS") || !strcmp(R.verdict, "MARGINAL"));
            sState = good ? SS_GOOD : SS_BAD;
            if (!good) for (uint8_t g = 0; g < IC_DB[selIdx].ngates; g++) if (!g_main.g[g].pass) { sBadGate = (int8_t)g; break; }
        }
    }
    dut_park(&IC_DB[selIdx]);
    live_update();
    if (sState == SS_GOOD)       { leds(1);  buzz(1, 150, 60); }
    else if (sState == SS_EMPTY) { leds(-1); buzz(2, 60, 80); }
    else                         { leds(0);  buzz(3, 80, 80); }
    simple_status_json();
    ui = UI_SIMPLE;
    ui_render();
}

static void boot_sequence(void)
{
    pin_wr(LEDP_PORT, LEDP_PIN, true); pin_wr(LEDF_PORT, LEDF_PIN, true);   /* both LEDs + buzzer 500 ms */
    buzzer_on(true);
    HAL_Delay(500);
    buzzer_on(false);
    leds(-1);
    if (oledOk) {                                                          /* splash */
        oled_clear();
        oled_text(4, 18, "IC CHECKER", 2);
        L(22, 44, 1, "74HC logic ICs");
        flush();
    }
    selftest();                                                            /* silent: offset zero + JSON */
    HAL_Delay(800);
    check_ic();
}

/* ============================== HOST RX ============================== */
static volatile uint8_t rxRing[256];
static volatile uint8_t rxHead, rxTail;
static uint8_t          rxByte;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h != &huart2) return;
    uint8_t n = (uint8_t)(rxHead + 1);
    if (n != rxTail) { rxRing[rxHead] = rxByte; rxHead = n; }
    HAL_UART_Receive_IT(&huart2, &rxByte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    if (h != &huart2) return;
    __HAL_UART_CLEAR_OREFLAG(h);
    if (h->gState == HAL_UART_STATE_READY) txBusy = false;
    HAL_UART_Receive_IT(&huart2, &rxByte, 1);
}

static void handle_line(char *line)
{
    while (isspace((unsigned char)*line)) line++;
    size_t n = strlen(line);
    while (n && isspace((unsigned char)line[n - 1])) line[--n] = 0;
    if (n < 2 || line[0] != '{' || line[n - 1] != '}') { send_err("bad_json"); return; }

    /* flat JSON field lookup */
    char cmd[16] = "", icbuf[32] = "";
    bool hasIc = false;
    const char *keys[2] = { "\"cmd\"", "\"ic\"" };
    char *outs[2] = { cmd, icbuf };
    size_t outn[2] = { sizeof cmd, sizeof icbuf };
    for (uint8_t k = 0; k < 2; k++) {
        const char *p = strstr(line, keys[k]);
        if (!p) continue;
        p += strlen(keys[k]);
        while (isspace((unsigned char)*p)) p++;
        if (*p != ':') continue;
        p++;
        while (isspace((unsigned char)*p)) p++;
        if (*p != '"') continue;
        size_t i = 0;
        for (p++; *p && *p != '"'; p++) if (i < outn[k] - 1) outs[k][i++] = *p;
        outs[k][i] = 0;
        if (k == 1) hasIc = true;
    }
    const char *ics = hasIc ? icbuf : NULL;
    int idx = ics ? find_ic(ics) : selIdx;

    if (!strcmp(cmd, "ping")) {
        j_reset(); j_putc('{');
        j_str("type", "pong"); j_str("fw", FW_VERSION); j_bool("ina", inaOk); j_bool("oled", oledOk);
        j_close('}'); j_send();
    } else if (!strcmp(cmd, "list")) {
        j_reset(); j_putc('{');
        j_str("type", "list");
        j_open("ics", '[');
        for (uint8_t i = 0; i < IC_COUNT; i++) {
            j_open(NULL, '{'); j_str("id", IC_DB[i].id); j_str("name", IC_DB[i].name); j_close('}');
        }
        j_close(']'); j_close('}'); j_send();
    } else if (!strcmp(cmd, "select")) {
        if (ics && (strstr(ics, "auto") || strstr(ics, "AUTO"))) { autoSel = true; idx = selIdx; }
        else if (idx < 0) { send_err("unknown_ic"); return; }
        else autoSel = false;
        selIdx = (uint8_t)idx; dut_park(&IC_DB[selIdx]); ui = UI_IDLE; autoState = -2; idTried = false; looksIdx = -1; autoT = 0; ui_render();
        j_reset(); j_putc('{'); j_str("type", "selected"); j_str("ic", IC_DB[selIdx].id); j_bool("auto", autoSel); j_close('}'); j_send();
    } else if (!strcmp(cmd, "test") && SIMPLE_UI && !ics) {
        check_ic();
    } else if (!strcmp(cmd, "test")) {
        if (idx < 0) { send_err("unknown_ic"); return; }
        if (ics) autoSel = false;
        selIdx = (uint8_t)idx; cmd_test(selIdx);
    } else if (!strcmp(cmd, "identify")) {
        cmd_identify(ics);
    } else if (!strcmp(cmd, "meter")) {
        if (idx < 0) { send_err("unknown_ic"); return; }
        uint16_t mask = 0;
        const char *p = strstr(line, "\"high\"");
        if (p && (p = strchr(p, '[')) != NULL) {
            for (p++; *p && *p != ']'; ) {
                char *e; long v = strtol(p, &e, 10);
                if (e == p) { p++; continue; }
                if (v >= 1 && v <= 14) mask |= (uint16_t)(1u << v);
                p = e;
            }
        }
        PinSnap s;
        meter_snapshot(&IC_DB[idx], mask, &s);
        dut_park(&IC_DB[selIdx]);
        j_reset(); j_putc('{');
        j_str("type", "meter"); j_str("ic", IC_DB[idx].id);
        j_num("vcc", s.vcc); j_num("idd_ma", s.idd);
        j_pins(&s);
        j_close('}'); j_send();
    } else if (!strcmp(cmd, "live")) {
        live_update();
        j_reset(); j_putc('{');
        j_str("type", "live"); j_num("vcc", inaOk ? liveV : -1); j_num("idd_ma", liveI); j_num("vdda", vdda);
        j_int("i2c_recoveries", (long)i2cRecoveries); j_int("i2c_hz", (long)i2cHz);
        j_close('}'); j_send();
    } else if (!strcmp(cmd, "manual")) {                          /* {"cmd":"manual","ic":"7486","gate":1,"x":3} */
        if (idx < 0) { send_err("unknown_ic"); return; }
        selIdx = (uint8_t)idx;
        const char *pg = strstr(line, "\"gate\""), *px = strstr(line, "\"x\"");
        long gv = pg ? strtol(strchr(pg, ':') + 1, NULL, 10) : 1;
        long xv = px ? strtol(strchr(px, ':') + 1, NULL, 10) : 0;
        const ICDef *ic = &IC_DB[selIdx];
        if (gv < 1 || gv > ic->ngates) gv = 1;
        if (ui != UI_MANUAL) { ui = UI_MANUAL; manAuto = false; }
        manGate = (uint8_t)(gv - 1);
        manual_reset_got();
        manBits = (uint8_t)(xv & ((1 << ic->g[manGate].nin) - 1));
        manual_apply(true);
        ui_render();
    } else if (!strcmp(cmd, "selftest")) {
        selftest();
    } else {
        send_err("unknown_cmd");
    }
}

static void poll_host(void)
{
    static char    buf[192];
    static uint8_t len;
    if (uartIrq) {
        if (huart2.RxState == HAL_UART_STATE_READY) HAL_UART_Receive_IT(&huart2, &rxByte, 1);   /* re-arm */
        if (huart2.gState == HAL_UART_STATE_READY && txBusy) { txBusy = false; tx_kick(); }    /* recover */
    } else if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {                                  /* polled fallback */
        uint8_t n = (uint8_t)(rxHead + 1);
        uint8_t c = (uint8_t)(huart2.Instance->DR & 0xFF);
        if (n != rxTail) { rxRing[rxHead] = c; rxHead = n; }
    }
    while (rxTail != rxHead) {
        char c = (char)rxRing[rxTail];
        rxTail = (uint8_t)(rxTail + 1);
        if (c == '\n' || c == '\r') { if (len) { buf[len] = 0; handle_line(buf); len = 0; } }
        else if (len < sizeof buf - 1) buf[len++] = c;
    }
}

/* ============================== BUTTONS ============================== */

enum { EV_NONE, EV_SHORT, EV_LONG };
static uint8_t btn_poll(Btn *b)
{
    bool pressed = !pin_rd(b->port, b->pin);
    uint32_t now = HAL_GetTick();
    uint8_t ev = EV_NONE;
    if (pressed && !b->prev)                                                  { b->t = now; b->longFired = false; }
    else if (pressed && b->prev && !b->longFired && now - b->t >= LONG_MS)   { b->longFired = true; ev = EV_LONG; }
    else if (!pressed && b->prev && !b->longFired && now - b->t >= DEBOUNCE_MS) ev = EV_SHORT;
    b->prev = pressed;
    return ev;
}

static void handle_buttons(void)
{
    uint8_t s = btn_poll(&bSel), r = btn_poll(&bRun), m = btn_poll(&bMan);
    static uint8_t lastRaw;
    uint8_t raw = (uint8_t)(bSel.prev | (bRun.prev << 1) | (bMan.prev << 2));
    if (raw != lastRaw) { lastRaw = raw; if (ui == UI_IDLE) ui_render(); }   /* show [SEL]/[RUN]/[MAN] */
    if (!s && !r && !m) return;
    if (SIMPLE_UI) { check_ic(); return; }                  /* any button (D8, D5, B1), short or hold = retest */

    if (ui == UI_MANUAL) {
        const ICDef *ic = &IC_DB[selIdx];
        if (m == EV_LONG || s == EV_LONG) { manual_exit(); return; }
        if (m == EV_SHORT) { manAuto = false; manual_next_input(); }
        if (s == EV_SHORT) {
            manGate = (uint8_t)((manGate + 1) % ic->ngates); manBits = 0;
            manual_reset_got(); manual_apply(true); ui_render();
        }
        if (r == EV_SHORT) { manAuto = !manAuto; manT = HAL_GetTick(); ui_render(); }
        if (r == EV_LONG)  { manAuto = false; cmd_test(selIdx); }
        return;
    }
    if (m == EV_SHORT && ui != UI_ALERT) { manual_enter(); return; }

    switch (ui) {
    case UI_IDLE:
        if (s == EV_SHORT) {                                          /* AUTO -> 7400 -> ... -> 7486 -> AUTO */
            if (autoSel)                     { autoSel = false; selIdx = 0; }
            else if (selIdx == IC_COUNT - 1) { autoSel = true; }
            else                             { selIdx++; }
            dut_park(&IC_DB[selIdx]); autoState = -2; idTried = false; looksIdx = -1; leds(-1); autoT = 0; ui_render();
        }
        if (s == EV_LONG)  { ui = UI_METER; meter_snapshot(&IC_DB[selIdx], 0, &snap); ui_render(); }
        break;
    case UI_METER:
        if (s)             { ui = UI_IDLE; live_update(); ui_render(); }
        break;
    case UI_RESULT:
        if (s == EV_SHORT) { resPage = (uint8_t)((resPage + 1) % 4); ui_render(); }
        if (s == EV_LONG)  { ui = UI_IDLE; autoState = -2; autoT = 0; live_update(); ui_render(); }
        break;
    case UI_IDENT:
        if (s)             { ui = UI_IDLE; live_update(); ui_render(); }
        break;
    case UI_ALERT:
        if (s) {
            live_update();
            if (!inaOk || liveI <= IDD_ABORT_MA) { dut_park(&IC_DB[selIdx]); ui = UI_IDLE; }
            ui_render();
        }
        return;                                              /* RUN disabled while alert is active */
    default: break;
    }
    if (r == EV_SHORT) cmd_test(selIdx);
    if (r == EV_LONG)  cmd_identify(IC_DB[selIdx].id);
}

/* ============================== I2C1 -> PB8/PB9 ============================== */
/* Whatever pins the .ioc gave I2C1 (default PB6/PB7 clashes with the FAIL LED), force SCL=PB8, SDA=PB9,
 * recover a stuck bus, and run at 400 kHz. */
static void i2c_setup(void)
{
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIOB->OTYPER |= (1u << 8) | (1u << 9);                        /* open-drain */
    pin_wr(GPIOB, 8, true); pin_wr(GPIOB, 9, true);
    pin_cfg(GPIOB, 8, M_OUT, PL_UP); pin_cfg(GPIOB, 9, M_OUT, PL_UP);
    for (uint8_t i = 0; i < 9; i++) { pin_wr(GPIOB, 8, false); delay_us(5); pin_wr(GPIOB, 8, true); delay_us(5); }
    pin_wr(GPIOB, 9, false); delay_us(5); pin_wr(GPIOB, 9, true); delay_us(5);   /* STOP */

    hi2c1.Init.ClockSpeed = i2cHz;
    hi2c1.Init.DutyCycle  = I2C_DUTYCYCLE_2;
    HAL_I2C_Init(&hi2c1);                                          /* MspInit may claim PB6/PB7 again */

    pin_cfg(GPIOB, 6, M_IN, PL_NONE);                              /* release PB6/PB7 from I2C1 */
    pin_cfg(GPIOB, 7, M_IN, PL_NONE);
    GPIOB->AFR[1]   = (GPIOB->AFR[1] & ~0xFFu) | 0x44u;            /* PB8, PB9 -> AF4 (I2C1) */
    GPIOB->OSPEEDR |= (3u << 16) | (3u << 18);
    pin_cfg(GPIOB, 8, M_AF, PL_UP);
    pin_cfg(GPIOB, 9, M_AF, PL_UP);

    hi2c1.Instance->CR1 |= I2C_CR1_SWRST;                          /* clear any BUSY seen while switching */
    hi2c1.Instance->CR1 &= ~I2C_CR1_SWRST;
    HAL_I2C_Init(&hi2c1);                                          /* state READY: no MspInit, just registers */
    led_pins_init();                                               /* PB6 = FAIL LED again (output, state kept) */
}

/* Full bus recovery: unstick bus + peripheral, re-init the devices, force a full screen redraw. */
static void i2c_recover(void)
{
    i2cRecoveries++;
    uint32_t now = HAL_GetTick();
    i2cRecBurst = (now - i2cRecT < 5000) ? i2cRecBurst + 1 : 1;     /* recoveries within 5 s of each other */
    i2cRecT = now;
    if (i2cRecBurst >= 3 && i2cHz > I2C_HZ_SAFE) i2cHz = I2C_HZ_SAFE;   /* wiring can't do 400 kHz: fall back */
    i2c_setup();
    if (oledOk) oled_reinit();                                     /* re-sends init, keeps framebuffer */
    if (inaOk)  ina219_begin(&hi2c1, INA_ADDR);
    inaErrSeen = ina219_errors();
}

/* Try to bring up whichever of OLED / INA219 is not up yet. */
static void devices_try(void)
{
    if (!oledOk) {
        oledOk = oled_begin(&hi2c1, OLED_ADDR, OLED_SH1106);
        if (!oledOk) oledOk = oled_begin(&hi2c1, OLED_ADDR_ALT, OLED_SH1106);
    }
    if (!inaOk) { inaOk = ina219_begin(&hi2c1, INA_ADDR); inaErrSeen = ina219_errors(); }
}

/* OLED flush with one recovery + retry if any page write failed. */
static void flush(void)
{
    if (oled_flush()) return;
    i2c_recover();
    oled_flush();
}

/* ============================== ENTRY ============================== */
void icv_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    dwt_init();
    reset_cause_read();

    i2c_setup();                                             /* first: HAL I2C init may touch PB6/PB7 */
    dut_all_hiz();

    pin_cfg(BTN_SEL_PORT, BTN_SEL_PIN, M_IN, PL_UP);
    pin_cfg(BTN_RUN_PORT, BTN_RUN_PIN, M_IN, PL_UP);
    pin_cfg(BTN_MAN_PORT, BTN_MAN_PIN, M_IN, PL_UP);                /* PB4: overrides NJTRST default */
    pin_wr(LEDP_PORT, LEDP_PIN, false); pin_wr(LEDF_PORT, LEDF_PIN, false);
    led_pins_init();
    pin_wr(HB_PORT, HB_PIN, false);     pin_cfg(HB_PORT, HB_PIN, M_OUT, PL_NONE);
    buzzer_init();
    buzzer_on(false);

    /* UART: interrupt mode only if CubeMX enabled the USART2 IRQ, otherwise blocking/polled */
    uartIrq = NVIC_GetEnableIRQ(USART2_IRQn) != 0;
    if (uartIrq) HAL_UART_Receive_IT(&huart2, &rxByte, 1);

    adc_init();
    HAL_Delay(150);                                          /* OLED / INA219 power-up after reset or flashing */
    for (uint8_t attempt = 0; attempt < 5 && !(oledOk && inaOk); attempt++) {
        if (attempt) { i2c_setup(); HAL_Delay(50); }         /* bus recovery between attempts */
        devices_try();
    }
    if (oledOk) { oled_clear(); L(0, 24, 1, "IC VISION  booting.."); flush(); }
    HAL_Delay(20);                                           /* first INA219 conversions */

    j_reset(); j_putc('{');
    j_str("type", "hello"); j_str("fw", FW_VERSION); j_str("reset", resetCause); j_bool("ina", inaOk); j_bool("oled", oledOk); j_bool("uart_irq", uartIrq); j_int("i2c_hz", I2C_HZ);
    j_close('}'); j_send();

    if (SIMPLE_UI) boot_sequence(); else selftest();
}

void icv_loop(void)
{
    static uint32_t hbT;
    poll_host();
    buzz_tick();
    handle_buttons();

    uint32_t now = HAL_GetTick();
    if (now - hbT >= 500) {
        hbT = now; HB_PORT->ODR ^= 1u << HB_PIN;                       /* LD2 heartbeat */
        if (hi2c1.Instance->SR2 & I2C_SR2_BUSY) {                    /* BUSY right after a STOP is normal ... */
            delay_us(2000);
            if (hi2c1.Instance->SR2 & I2C_SR2_BUSY) i2c_recover();     /* ... still BUSY 2 ms later = stuck bus */
        }
    }
    static uint32_t retryT;
    if ((!oledOk || !inaOk) && now - retryT >= 1000) {              /* late / re-plugged OLED or INA219 */
        retryT = now;
        bool had = oledOk;
        devices_try();
        if (oledOk && !had) { oled_invalidate(); ui_render(); }
    }

    if (ui == UI_IDLE && now - autoT >= 1000) {                     /* live GOOD / BAD / EMPTY on LEDs */
        autoT = now;
        int8_t before = autoState; uint8_t selBefore = selIdx;
        auto_check();
        if (autoState != before || selIdx != selBefore) ui_render();
    }

    if (now - liveT >= LIVE_MS) {
        static uint8_t healPage;
        bool measured = true;
        if (ui == UI_IDLE || ui == UI_SIMPLE) { live_update(); }
        else if (ui == UI_METER)  { meter_snapshot(&IC_DB[selIdx], 0, &snap); }
        else if (ui == UI_ALERT)  { live_update(); }
        else if (ui == UI_RESULT || ui == UI_IDENT) { liveT = now; measured = false; }
        else if (ui == UI_MANUAL) {
            liveT = now; measured = false;
            if (manAuto && now - manT >= 1000) {                     /* auto-step through the truth table */
                manT = now;
                uint8_t nv = (uint8_t)(1u << IC_DB[selIdx].g[manGate].nin);
                manBits = (uint8_t)((manBits + 1u) % nv);
                manual_apply(true);
            } else manual_apply(false);                               /* live re-read of the output */
        }
        else return;
        if (oledOk) oled_touch_page(healPage++ & 7u);                /* re-send one page per tick: self-heal */
        if (!measured) { ui_render(); return; }
        static uint32_t okSince;
        if (ui == UI_ALERT) {                                          /* auto-clear after 3 s of normal current */
            if (!inaOk || liveI <= IDD_WARN_MA) { if (!okSince) okSince = now; }
            else okSince = 0;
            if (okSince && now - okSince >= 3000) {
                okSince = 0; dut_park(&IC_DB[selIdx]); leds(-1);
                if (SIMPLE_UI) pendingCheck = true; else ui = UI_IDLE;
            }
        } else okSince = 0;
        if (inaOk && liveI > IDD_ABORT_MA && ui != UI_ALERT && overcurrent_confirmed()) {
            dut_all_hiz(); alertMsg = "OVERCURRENT"; ui = UI_ALERT;
            leds(0); buzz(5, 60, 60);
            j_reset(); j_putc('{'); j_str("type", "alert"); j_str("reason", "OVERCURRENT"); j_num("idd_ma", liveI); j_close('}'); j_send();
        }
        ui_render();                                         /* only changed OLED pages are sent */
    }
    if (pendingCheck) { pendingCheck = false; check_ic(); }
}
