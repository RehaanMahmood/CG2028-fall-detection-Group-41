/******************************************************************************
 * @file           : main.c
 * @brief          : CG2028 Assignment - ElderCare Wearable Safety Companion
 * @author         : Hou Linxin
 * (c) CG2028 Teaching Team
 ******************************************************************************/

/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "stm32l4s5xx.h"
#include <math.h>
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/Components/lsm6dsl/lsm6dsl.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

/*--------------------------- Configuration ----------------------------------*/
#define EWMA_ALPHA_ACCEL_PERCENT 80
#define EWMA_ALPHA_GYRO_PERCENT 15
#define FALL_LED_DELAY_MS 150
#define FREEFALL_STREAK_MIN 3 // Tweak to adjust minimum drop duration
#define NORMAL_LED_DELAY_MS 1

static void UART1_Init(void);
static void UART_Send(const char *text);

extern int ewma_filter(int new_data, int old_output, int alpha_percent);
// int ewma_filter_C(int new_data, int old_output, int alpha_percent);

UART_HandleTypeDef huart1;

uint32_t start, end, cycles;

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

int main(void)
{
    HAL_Init();
    UART1_Init();

    BSP_LED_Init(LED2);
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_LED_Off(LED2);

    /* Previous EWMA outputs. The first test/application sample starts from 0. */
    int accel_ewma_asm[3] = {0, 0, 0};
    int gyro_ewma_asm[3] = {0, 0, 0};

    /* Reference C states are kept separately for assembly verification. */
    int accel_ewma_c[3] = {0, 0, 0};
    int gyro_ewma_c[3] = {0, 0, 0};

    float prev_accel_mps2[3];
    float prev_gyro_dps[3];

    unsigned long sample_number = 0;
    float current_distance_travelled = 0;
    int fall_detected = 0;
    int recovery_period = 20;
    int fall_period = 5;
    float prev_gyro_dps_magnitude = 0;
    float prev_accel_mps2_magnitude = 0;
    int prev_state = 0;
    int current_state = 0; // 0: "normal" state, 1: "released" state, 2: "free-fall" state
    static int freefall_streak = 0;

    float angular_displacement[3] = {0.0f, 0.0f, 0.0f};

    DWT_Init();

    while (1)
    {
        start = DWT->CYCCNT;
        int16_t accel_raw_i16[3] = {0, 0, 0};
        float gyro_raw_float[3] = {0.0f, 0.0f, 0.0f};
        int gyro_raw_int[3] = {0, 0, 0};

        BSP_ACCELERO_AccGetXYZ(accel_raw_i16);
        BSP_GYRO_GetXYZ(gyro_raw_float);

        /* The supplied BSP reports gyroscope readings as floating-point raw
         * values. Convert them to signed integers before passing them to the
         * integer assembly routine. */
        for (int axis = 0; axis < 3; axis++)
        {
            gyro_raw_int[axis] = (int)gyro_raw_float[axis];

            // for now Im using the simple c version of the filter, assembly version to be implemented later on

            accel_ewma_asm[axis] = ewma_filter(
                (int)accel_raw_i16[axis],
                accel_ewma_asm[axis],
                EWMA_ALPHA_ACCEL_PERCENT);

            gyro_ewma_asm[axis] = ewma_filter(
                gyro_raw_int[axis],
                gyro_ewma_asm[axis],
                EWMA_ALPHA_GYRO_PERCENT);

            // accel_ewma_c[axis] = ewma_filter_C(
            //     (int)accel_raw_i16[axis],
            //     accel_ewma_c[axis],
            //     EWMA_ALPHA_ACCEL_PERCENT);

            // gyro_ewma_c[axis] = ewma_filter_C(
            //     gyro_raw_int[axis],
            //     gyro_ewma_c[axis],
            //     EWMA_ALPHA_GYRO_PERCENT);
        }

        /* Accelerometer filtered readings are in meters per second squared. */
        float accel_mps2[3] = {
            accel_ewma_asm[0] * (9.80665f / 1000.0f),
            accel_ewma_asm[1] * (9.80665f / 1000.0f),
            accel_ewma_asm[2] * (9.80665f / 1000.0f)};

        /* Gyroscope filtered readings are in degrees per second. */
        float gyro_dps[3] = {
            gyro_ewma_asm[0] / 1000.0f,
            gyro_ewma_asm[1] / 1000.0f,
            gyro_ewma_asm[2] / 1000.0f};

        // get overall magnitude for accelerometer and gyroscope

        float accel_mps2_magnitude = sqrt(accel_mps2[0] * accel_mps2[0] + accel_mps2[1] * accel_mps2[1] + accel_mps2[2] * accel_mps2[2]);
        float gyro_dps_magnitude = sqrt(gyro_dps[0] * gyro_dps[0] + gyro_dps[1] * gyro_dps[1] + gyro_dps[2] * gyro_dps[2]);

        float accel_mps2_z = accel_mps2[2];
        char buffer[320];
        snprintf(buffer, sizeof(buffer),
                 "Sample %lu\r\n"
                 "Accel EWMA ASM [m/s^2]: X=%8.3f Y=%8.3f Z=%8.3f\r\n"
                 "Gyro  EWMA ASM [dps]  : X=%8.3f Y=%8.3f Z=%8.3f\r\n"
                 "current_accel_magnitude : %8.3f\r\n"
                 "current_distance_travelled : %8.3f\r\n"
                 "current_gyro_magnitude : %8.3f\r\n",
                 sample_number,
                 accel_mps2[0], accel_mps2[1], accel_mps2[2],
                 gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_mps2_magnitude, current_distance_travelled, gyro_dps_magnitude);
        UART_Send(buffer);

        /* Optional debugging check. This confirms that the assembly routine
         * matches the reference C routine for the current samples. */
        // if ((accel_ewma_asm[0] != accel_ewma_c[0]) ||
        //     (accel_ewma_asm[1] != accel_ewma_c[1]) ||
        //     (accel_ewma_asm[2] != accel_ewma_c[2]) ||
        //     (gyro_ewma_asm[0] != gyro_ewma_c[0]) ||
        //     (gyro_ewma_asm[1] != gyro_ewma_c[1]) ||
        //     (gyro_ewma_asm[2] != gyro_ewma_c[2]))
        // {
        //     UART_Send("WARNING: Assembly and C EWMA outputs do not match.\r\n");
        // }

        /**************** Elderly wearable state logic starts here************************
         * Compulsory requirements:
         * 1. Use filtered accelerometer AND gyroscope readings.
         * 2. Distinguish normal activity, near-fall movements, and a real fall.
         * 3. Use a slow LED blink for normal operation and a fast blink after
         *    a fall is detected.
         *********************************************************************/

        // this logic checks for decreases in acceleration (to detect if the board has been released)

        if ((sample_number >= 1) &&
            (accel_mps2_magnitude <= 9.8f) &&                          // |accel| <= g
            (current_state != 2) &&                                    // not free-falling
            (accel_mps2_magnitude <= 0.5 * prev_accel_mps2_magnitude)) // accel is less than half of prev_accel
        {
            /*angular_displacement[0] += gyro_dps[0] * 0.045f;
            angular_displacement[1] += gyro_dps[1] * 0.045f;
            angular_displacement[2] += gyro_dps[2] * 0.045f;*/
            current_state = 1; // it is released
        }
        else
        {
            current_state = 0; // it is normal
        }

        // if previously board was in the "released state", check if the vertical acceleration is high enough, and change it to the "free-fall" state
        // note that the acceleration measured by the board is the normal force acting against gravity so lower reading means higher downward acceleration

        // Counts the number of consecutive samples where the object is falling after being released. A minimum threshold prevents a drop of a few cm to cause fall detection.
        if (accel_mps2_magnitude <= 2.5f)
        {
            freefall_streak++;
        }
        else
        {
            freefall_streak = 0;
        }

        if (current_state == 0 && prev_state == 1 && freefall_streak >= FREEFALL_STREAK_MIN)
        {
            current_state = 2;
        }

        // Once the board is in "free-fall" state, we keep checking if there is a sudden increase in acceleration (impact by ground), or sudden change in gyroscope readings to check for vibrations due to the fall
        // Once any of these 2 conditions is met, fall can be detected
        if (current_state == 2)
        {
            if (((accel_mps2_magnitude - prev_accel_mps2_magnitude) >= (5.0f * prev_accel_mps2_magnitude) &&
                 accel_mps2_magnitude >= 25.0f) ||
                (gyro_dps_magnitude - prev_gyro_dps_magnitude) >= 10.0f)
            {
                fall_detected = 1;
            }
        }

        if (fall_detected == 1)
        {
            char buffer0[320];
            snprintf(buffer0, sizeof(buffer0), "FALL DETECTED!!!!!!!!\n");
            UART_Send(buffer0);
            current_state = 0;
        }

        // prev_accel_mps2 = accel_mps2;
        // prev_gyro_dps = gyro_dps;

        if (sample_number >= 1 && fabsf(prev_gyro_dps_magnitude - gyro_dps_magnitude) >= 5.0f && fall_detected == 1)
        {
            recovery_period -= 1;
            if (recovery_period <= 0)
            {
                fall_detected = 0;
            }
        }
        else
        {
            recovery_period = (recovery_period > 18) ? 20 : recovery_period + 1;
        }

        prev_gyro_dps_magnitude = gyro_dps_magnitude;
        prev_accel_mps2_magnitude = accel_mps2_magnitude;
        prev_state = current_state;

        // BSP_LED_Toggle(LED2);
        HAL_Delay(fall_detected ? FALL_LED_DELAY_MS : NORMAL_LED_DELAY_MS);
        end = DWT->CYCCNT;
        cycles = end - start;
        float time_us = (float)cycles / (SystemCoreClock / 1000000);

        /*char buffer1[320];
        snprintf(buffer1, sizeof(buffer1), "Time taken : %8.3f\n", time_us);
        UART_Send(buffer1);*/

        sample_number++;
    }
}

int ewma_filter_C(int new_data, int old_output, int alpha_percent)
{
    /* Reference implementation for verification only. The assembly routine
     * must be used in the actual sensor-processing and detection pipeline. */
    int numerator = alpha_percent * new_data + (100 - alpha_percent) * old_output;
    return numerator / 100;
}

static void UART_Send(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        while (1)
        {
        }
    }
}

/* Do not modify these lines. They suppress UART-related warnings. */
int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}
int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}
int _fstat(int file, struct stat *st)
{
    (void)file;
    (void)st;
    return 0;
}
int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}
int _isatty(int file)
{
    (void)file;
    return 1;
}
int _close(int file)
{
    (void)file;
    return -1;
}
int _getpid(void) { return 1; }
int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    return -1;
}
