// TM4C123GXL - Closed Loop PID Temperature Control
// LM35 -> PE3 (AIN0), Pot -> PE2 (AIN1), PWM -> PB6 (M0PWM0)
// 1 kHz PWM, 50ms loop, Heater duty max 16% for safety.

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_pwm.h"
#include "inc/hw_gpio.h"
#include "inc/hw_uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"

// ---------------- USER CONSTANTS ----------------
#define PWM_FREQUENCY_HZ     1000U
#define LOOP_PERIOD_S        0.05f
#define SAFE_DUTY_MAX        16.0f
#define VREF_V               3.3f
#define ADC_MAX              4096.0f
#define LM35_MV_PER_C        10.0f
#define SET_MIN_C            30.0f
#define SET_MAX_C            60.0f
#define SAFETY_CUTOFF_C      100.0f

// PID gains (stable)
static float Kp = 2.0f;
static float Ki = 0.5f;
static float Kd = 0.1f;

static uint32_t pwmLoad;

// Function prototypes
void Clock_Init(void);
void UART0_Init(void);
void ADC0_Init(void);
uint32_t ADC_ReadAvg(uint32_t ch, uint32_t n);
void PWM0_Init(void);
void PWM_SetDutyPB6(float duty_percent);
void Delay_ms(uint32_t ms);

// -------------------- MAIN --------------------------
int main(void)
{
    Clock_Init();
    UART0_Init();
    ADC0_Init();
    PWM0_Init();

    UARTprintf("\nStable PID Heater Controller\n");

    float integral = 0;
    float prev_err = 0;
    float prev_temp = 0;

    while(1)
    {
        // ----- Temperature -----
        uint32_t adc_temp = ADC_ReadAvg(0, 16);
        float volts_temp  = (adc_temp * VREF_V) / ADC_MAX;
        float tempC       = (volts_temp * 1000.0f) / LM35_MV_PER_C;

        // ----- Setpoint -----
        uint32_t adc_set = ADC_ReadAvg(1, 16);
        float vset       = (adc_set * VREF_V) / ADC_MAX;
        float setC       = SET_MIN_C + (SET_MAX_C - SET_MIN_C) * (vset / VREF_V);

        // ----- PID -----
        float err = setC - tempC;

        // Integral anti-windup
        if (fabsf(err) < 5.0f)           // only integrate near setpoint
            integral += err * LOOP_PERIOD_S;
        else
            integral = 0;

        // Clamp integral
        if (integral > 20.0f) integral = 20.0f;
        if (integral < -5.0f) integral = -5.0f;

        // Derivative using filtered temperature change
        float deriv = (tempC - prev_temp) / LOOP_PERIOD_S;
        prev_temp = tempC;

        float duty = Kp*err + Ki*integral - Kd*deriv;

        // ----- SAFETY -----
        if (tempC > SAFETY_CUTOFF_C) duty = 0;

        if (duty < 0) duty = 0;
        if (duty > SAFE_DUTY_MAX) duty = SAFE_DUTY_MAX;

        // ----- Apply PWM -----
        PWM_SetDutyPB6(duty);

        UARTprintf("Set:%d  Temp:%d  Duty:%d%%\n",
                    (int)setC, (int)tempC, (int)duty);

        Delay_ms((uint32_t)(LOOP_PERIOD_S * 1000));
    }
}

// ------------ CLOCK --------------
void Clock_Init(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                   SYSCTL_OSC_MAIN   | SYSCTL_XTAL_16MHZ);
}

// ------------ UART ---------------
void UART0_Init(void)
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

// ------------ ADC ----------------
void ADC0_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);
    ADCHardwareOversampleConfigure(ADC0_BASE, 16);
}

uint32_t ADC_ReadCh(uint32_t ch)
{
    uint32_t value;
    ADCSequenceDisable(ADC0_BASE, 3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ch | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);

    ADCIntClear(ADC0_BASE, 3);
    ADCProcessorTrigger(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false)) {}
    ADCSequenceDataGet(ADC0_BASE, 3, &value);

    return value & 0xFFF;
}

uint32_t ADC_ReadAvg(uint32_t ch, uint32_t n)
{
    uint32_t sum = 0;
    uint32_t i;
    for(i = 0; i < n; i++)
        sum += ADC_ReadCh(ch);

    return sum/n;
}

// ------------ PWM PB6 -----------
void PWM0_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTB_BASE + GPIO_O_CR) |= GPIO_PIN_6;
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = 0;

    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    uint32_t pwmClk = SysCtlClockGet() / 64;
    pwmLoad = (pwmClk / PWM_FREQUENCY_HZ) - 1;

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, pwmLoad);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 2);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

void PWM_SetDutyPB6(float duty)
{
    if (duty < 0) duty = 0;
    if (duty > 100) duty = 100;

    uint32_t cmp = (uint32_t)((duty / 100.0f) * pwmLoad);

    if (cmp >= pwmLoad) cmp = pwmLoad - 2;
    if (cmp < 2) cmp = 2;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, cmp);
}

// ------------ Delay -------------
void Delay_ms(uint32_t ms)
{
    SysCtlDelay((SysCtlClockGet()/3000)*ms);
}
