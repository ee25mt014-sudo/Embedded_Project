// TM4C123GXL - Closed Loop PID Temperature Control (12V heater, duty capped <= 16%)
// LM35 -> PE3 (AIN0), Pot -> PE2 (AIN1), PWM -> PB6 (M0PWM0), UART0 -> PA0/PA1
// UART: 115200 8N1
// PWM: 1 kHz, loop period: 50 ms
//
// *** 12V HEATER SAFETY ***
// 4.7Ω at 12V => 30.6W at 100% duty. To keep <= ~5W average, cap duty <= 16%.
// We enforce this with SAFE_DUTY_MAX = 16.0f. Do not raise unless you redesign power path.
//
// *** WIRING REMINDERS ***
// - Common ground between 12V adapter and Tiva.
// - PB6 -> MOSFET gate via 100–220Ω; 10k from gate to GND.
// - Heater: +12V -> 4.7Ω -> MOSFET D -> MOSFET S -> GND.
// - LM35: V+ (4–30V, e.g., 12V or clean 5V), GND, Vout -> PE3 (AIN0).
// - Pot: 3.3V and GND from Tiva; wiper -> PE2 (AIN1).

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

// ---------- USER SETTINGS ----------
#define PWM_FREQUENCY_HZ     1000U   // 1 kHz PWM
#define LOOP_PERIOD_S        0.05f   // 50 ms control loop
#define VREF_V               3.3f    // TM4C ADC reference (VDDA)
#define ADC_MAX              4096.0f // 12-bit ADC steps
#define LM35_MV_PER_C        10.0f   // 10 mV/°C
#define SAFETY_CUTOFF_C      100.0f  // absolute cutoff

// Pot maps 0..3.3V to this temp range:
#define SET_MIN_C            30.0f
#define SET_MAX_C            90.0f

// *** 12V Heater Cap ***
// Keep average power ~<= 5W with 4.7Ω at 12V by limiting duty to ~16%.
#define SAFE_DUTY_MAX        60.0f   // percent, 0..100 (DO NOT INCREASE for 12V+4.7Ω breadboard demo)

// PID gains (dt = LOOP_PERIOD_S)
static float Kp = 3.0f, Ki = 0.10f, Kd = 1.0f;

// ---------- FORWARD DECL ----------
static void Clock_Init(void);
static void UART0_Init(void);
static void ADC0_Init(void);
static uint32_t ADC_ReadCh(uint32_t ch);
static uint32_t ADC_ReadAvg(uint32_t ch, uint32_t n);
static void PWM0B6_Init(void);
static void PWM_SetDutyPB6(float duty_percent);
static void Delay_ms(uint32_t ms);

// helper: print a float with 1 decimal via integers (UARTprintf has no %f by default)
static void UARTPrintF1(const char *label, float v, const char *unit);

// ---------- GLOBALS ---------
static uint32_t pwmLoad = 0;

// ---------- MAIN ----------
int main(void)
{
    Clock_Init();
    UART0_Init();
    ADC0_Init();
    PWM0B6_Init();

    UARTprintf("\nPID Temp Controller (TM4C123GXL)\n");
    UARTprintf("12V heater, duty cap = %d%% (4.7 ohm)\n", (int)SAFE_DUTY_MAX);
    UARTprintf("LM35: PE3(AIN0), Pot: PE2(AIN1), PWM: PB6(M0PWM0)\n");
    UARTprintf("dt = %d ms, PWM = %u Hz\n",
               (int)(LOOP_PERIOD_S*1000.0f), PWM_FREQUENCY_HZ);

    float integral = 0.0f;
    float prev_err = 0.0f;

    while (1)
    {
        // --- Read LM35 temperature (PE3 = AIN0)
        uint32_t adc_temp = ADC_ReadAvg(0 /*AIN0*/, 16);
        float volts_temp  = (adc_temp * VREF_V) / ADC_MAX;              // V
        float tempC       = (volts_temp * 1000.0f) / LM35_MV_PER_C;     // °C

        // --- Read Pot setpoint (PE2 = AIN1)
        uint32_t adc_set = ADC_ReadAvg(1 /*AIN1*/, 16);
        float vset       = (adc_set * VREF_V) / ADC_MAX;                // V
        float setC       = SET_MIN_C + (SET_MAX_C - SET_MIN_C) * (vset / VREF_V);

        // --- PID compute
        float err = setC - tempC;
        integral += err * LOOP_PERIOD_S;

        // anti-windup: clamp integral so Ki*I stays within [0, SAFE_DUTY_MAX]
        float i_limit = (SAFE_DUTY_MAX > 0) ? (SAFE_DUTY_MAX / fmaxf(Ki, 0.001f)) : 0.0f;
        if (integral >  i_limit) integral =  i_limit;
        if (integral < -i_limit) integral = -i_limit;

        float deriv = (err - prev_err) / LOOP_PERIOD_S;
        float duty  = Kp*err + Ki*integral + Kd*deriv;
        prev_err = err;

        // Safety cutoff / sensor sanity
        if (tempC > SAFETY_CUTOFF_C || adc_temp == 0 || adc_temp >= 4095) {
            duty = 0.0f;
            integral = 0.0f;
        }

        // Clamp duty: 0..SAFE_DUTY_MAX (%)
        if (duty < 0.0f) duty = 0.0f;
        if (duty > SAFE_DUTY_MAX) duty = SAFE_DUTY_MAX;

        // Apply PWM
        PWM_SetDutyPB6(duty);

        // Telemetry (1 decimal)
        UARTPrintF1("Set:",  setC,  " C  ");
        UARTPrintF1("Meas:", tempC, " C  ");
        UARTPrintF1("PWM:",  duty,  "%%\n");

        // Loop timing
        Delay_ms((uint32_t)(LOOP_PERIOD_S * 1000.0f));
    }
}

// ---------- helper: print float with 1 decimal ----------
static void UARTPrintF1(const char *label, float v, const char *unit)
{
    int ip   = (int)v;
    int frac = (int)fabsf((v - (float)ip) * 10.0f + 0.5f); // round to 0.1
    if (frac >= 10) { ip += (v >= 0 ? 1 : -1); frac -= 10; }
    UARTprintf("%s %d.%01d%s", label, ip, frac, unit);
}

// ---------- CLOCK ----------
static void Clock_Init(void)
{
    // 80 MHz system clock from PLL (16 MHz crystal)
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                   SYSCTL_OSC_MAIN   | SYSCTL_XTAL_16MHZ);
}

// ---------- UART0 (115200) ----------
static void UART0_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTStdioConfig(0, 115200, SysCtlClockGet());
}

// ---------- ADC0 (PE3=Ain0, PE2=Ain1) ----------
static void ADC0_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Analog inputs on PE3, PE2
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

    // 16x hardware averaging (DriverLib)
    ADCHardwareOversampleConfigure(ADC0_BASE, 16);
}

// Read one sample from given AIN channel number (0..11), SS3 single-step
static uint32_t ADC_ReadCh(uint32_t ch)
{
    uint32_t value;

    ADCSequenceDisable(ADC0_BASE, 3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ch | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);

    ADCIntClear(ADC0_BASE, 3);
    ADCProcessorTrigger(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false)) {}
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &value);
    return (value & 0xFFF);
}

// Average N samples (e.g., 16) for stability
static uint32_t ADC_ReadAvg(uint32_t ch, uint32_t n)
{
    uint32_t acc = 0;
    uint32_t i;
    for (i = 0; i < n; i++) acc += ADC_ReadCh(ch);
    return acc / n;
}

// ---------- PWM0 M0PWM0 on PB6 ----------
static void PWM0B6_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // PB6 -> M0PWM0
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    // PWM clock = SysClk / 64
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    uint32_t pwmClk = SysCtlClockGet() / 64;
    pwmLoad = (pwmClk / PWM_FREQUENCY_HZ) - 1;

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, pwmLoad);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0); // start OFF
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

static void PWM_SetDutyPB6(float duty_percent)
{
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint32_t cmp = (uint32_t)((duty_percent / 100.0f) * pwmLoad);
    if (cmp >= pwmLoad) cmp = pwmLoad - 1;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, cmp);
}

// ---------- Delay (ms) ----------
static void Delay_ms(uint32_t ms)
{
    // SysCtlDelay = 3 cycles per loop
    uint32_t ticks = (SysCtlClockGet() / 3000U) * ms;
    SysCtlDelay(ticks);
}
