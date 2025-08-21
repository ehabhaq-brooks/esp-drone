#include "motors.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/ledc.h"
#include "log.h"
#include "param.h"

// Constants for ESC PWM
#define PWM_MIN_PULSE_US 1000
#define PWM_MAX_PULSE_US 2000
#define PWM_PERIOD_US    20000  // 50Hz -> 20ms period

uint32_t motor_ratios[] = {0, 0, 0, 0};

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

const MotorPerifDef **motorMap;

const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};
const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5};

static bool isInit = false;
static bool isTimerInit = false;

ledc_channel_config_t motors_channel[NBR_OF_MOTORS] = {
    {
        .channel = MOT_PWM_CH1,
        .duty = 0,
        .gpio_num = MOTOR1_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH2,
        .duty = 0,
        .gpio_num = MOTOR2_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH3,
        .duty = 0,
        .gpio_num = MOTOR3_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH4,
        .duty = 0,
        .gpio_num = MOTOR4_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
};

// Timer init for 50Hz ESC-compatible PWM
bool pwm_timmer_init()
{
    if (isTimerInit) {
        return true;
    }

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_14_BIT,  // 0–65535
        .freq_hz = 50,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
    };

    if (ledc_timer_config(&ledc_timer) == ESP_OK) {
        isTimerInit = true;
        return true;
    }

    return false;
}

void motorsInit(const MotorPerifDef **motorMapSelect)
{
    if (isInit) {
        return;
    }

    motorMap = motorMapSelect;

    if (!pwm_timmer_init()) {
        return;
    }

    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_channel_config(&motors_channel[i]);
    }

    isInit = true;
}

void motorsDeInit(const MotorPerifDef **motorMapSelect)
{
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_stop(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
    }
}

bool motorsTest(void)
{
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetRatio(MOTORS[i], 32768);  // 50% throttle
        vTaskDelay(M2T(500));
        motorsSetRatio(MOTORS[i], 0);
        vTaskDelay(M2T(300));
    }

    return isInit;
}

void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
    if (isInit) {
        ASSERT(id < NBR_OF_MOTORS);

        // Map ithrust (0–65535) to 1000–2000us pulse
        uint32_t pulse_us = PWM_MIN_PULSE_US +
            ((uint32_t)ithrust * (PWM_MAX_PULSE_US - PWM_MIN_PULSE_US)) / 65535;

        // Convert pulse_us to duty cycle for 16-bit resolution
        uint32_t duty = (pulse_us * ((1 << 14) - 1)) / PWM_PERIOD_US;

        ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, duty);
        ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
        motor_ratios[id] = ithrust;
    }
}

int motorsGetRatio(uint32_t id)
{
    ASSERT(id < NBR_OF_MOTORS);
    return motor_ratios[id];
}

// Simulated beep using 1000us/2000us toggle
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
    ASSERT(id < NBR_OF_MOTORS);

    uint32_t pulse_us = enable ? 2000 : 1000;
    uint32_t duty = (pulse_us * ((1 << 16) - 1)) / PWM_PERIOD_US;

    ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, duty);
    ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
}

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec)
{
    motorsBeep(MOTOR_M1, true, frequency, 0);
    motorsBeep(MOTOR_M2, true, frequency, 0);
    motorsBeep(MOTOR_M3, true, frequency, 0);
    motorsBeep(MOTOR_M4, true, frequency, 0);
    vTaskDelay(M2T(duration_msec));
    motorsBeep(MOTOR_M1, false, frequency, 0);
    motorsBeep(MOTOR_M2, false, frequency, 0);
    motorsBeep(MOTOR_M3, false, frequency, 0);
    motorsBeep(MOTOR_M4, false, frequency, 0);
}

void motorsPlayMelody(uint16_t *notes)
{
    int i = 0;
    uint16_t note;
    uint16_t duration;

    do {
        note = notes[i++];
        duration = notes[i++];
        motorsPlayTone(note, duration);
    } while (duration != 0);
}

LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT32, m1_pwm, &motor_ratios[0])
LOG_ADD(LOG_UINT32, m2_pwm, &motor_ratios[1])
LOG_ADD(LOG_UINT32, m3_pwm, &motor_ratios[2])
LOG_ADD(LOG_UINT32, m4_pwm, &motor_ratios[3])
LOG_GROUP_STOP(pwm)
