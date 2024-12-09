#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "driver/adc.h"
#include "driver/gpio.h"

// Configuración de usuario
#define IDENTIFY_HALLS_ON_BOOT true
#define IDENTIFY_HALLS_REVERSE false

#define THROTTLE_LOW 600
#define THROTTLE_HIGH 2650
#define PHASE_MAX_CURRENT_MA 6000
#define BATTERY_MAX_CURRENT_MA 3000
#define CURRENT_CONTROL_LOOP_GAIN 200

#define PWM_FREQ 16000 // Frecuencia PWM en Hz
#define DUTY_CYCLE_MAX 90 // Máximo duty cycle (0-100%)

#define LED_PIN GPIO_NUM_2
#define HALL_A_PIN GPIO_NUM_13
#define HALL_B_PIN GPIO_NUM_14
#define HALL_C_PIN GPIO_NUM_15

#define GPIO_PWM0A_OUT GPIO_NUM_29   // Fase AH (high)
#define GPIO_PWM0B_OUT GPIO_NUM_25   // Fase AL (low)
#define GPIO_PWM1A_OUT GPIO_NUM_28   // Fase BH (high)
#define GPIO_PWM1B_OUT GPIO_NUM_29   // Fase BL (low)
#define GPIO_PWM2A_OUT GPIO_NUM_27   // Fase CH (high)
#define GPIO_PWM2B_OUT GPIO_NUM_26   // Fase CL (low)

#define LED1 GPIO_NUM_16
#define LED2 GPIO_NUM_17
#define LED3 GPIO_NUM_18
#define LED4 GPIO_NUM_19
#define LED5 GPIO_NUM_20
#define LED6 GPIO_NUM_21

#define ADC_CHANNEL_THROTTLE ADC1_CHANNEL_0
#define ADC_CHANNEL_ISENSE ADC1_CHANNEL_3
#define ADC_CHANNEL_VSENSE ADC1_CHANNEL_6

// Variables globales
uint8_t hallToMotor[8] = {255, 255, 255, 255, 255, 255, 255, 255}; // Tabla de estados del motor
int duty_cycle = 0;
int current_ma = 0, voltage_mv = 0, current_target_ma = 0;
static uint32_t hall_sensor_value = 0;

// Tabla de conmutación basada en sensores Hall
const int hall_table[8][3] = {
    {0, 0, 0},  // Estado inválido
    {0, 0, 1},  // Fase 1
    {0, 1, 0},  // Fase 2
    {0, 1, 1},  // Fase 3
    {1, 0, 0},  // Fase 4
    {1, 0, 1},  // Fase 5
    {1, 1, 0},  // Fase 6
    {1, 1, 1}   // Estado inválido
};

esp_err_t init_gpio(void);
void switch_phase(void);
int read_hall_sensors(void);

esp_err_t init_gpio(void)
{
    // Configurar pines GPIO para el puente H
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED5, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED6, GPIO_MODE_OUTPUT);

    
    // Configurar pines GPIO para los sensores Hall como entradas con pull-up interno
    gpio_config_t hall_config = {
        .pin_bit_mask = (1ULL << HALL_A_PIN) | (1ULL << HALL_B_PIN) | (1ULL << HALL_C_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&hall_config);
    return ESP_OK;
}

// Función para leer el estado de los sensores Hall
int read_hall_sensors(void)
{
    int hall_a = gpio_get_level(HALL_A_PIN);
    int hall_b = gpio_get_level(HALL_B_PIN);
    int hall_c = gpio_get_level(HALL_C_PIN);
    return (hall_a << 2) | (hall_b << 1) | hall_c;
}

void app_main(void)
{
    init_gpio();
    printf("Motor control iniciado\n");
    while (true){
        int hall_value = read_hall_sensors();
        printf("hall_sen val: %d\n", hall_value);
        if (hall_value == 2) {
            gpio_set_level(LED1,0);
            gpio_set_level(LED2,1);
            gpio_set_level(LED3,0);
            gpio_set_level(LED4,0);
            gpio_set_level(LED5,0);
            gpio_set_level(LED6,0);
        } else if (hall_value == 6) {
            gpio_set_level(LED1,0);
            gpio_set_level(LED2,0);
            gpio_set_level(LED3,0);
            gpio_set_level(LED4,0);
            gpio_set_level(LED5,0);
            gpio_set_level(LED5,1);
        } else if (hall_value == 4) {
            gpio_set_level(LED1,0);
            gpio_set_level(LED2,0);
            gpio_set_level(LED3,0);
            gpio_set_level(LED4,1);
            gpio_set_level(LED5,0);
            gpio_set_level(LED6,0);
        } else if (hall_value == 5) {
            gpio_set_level(LED1,0);
            gpio_set_level(LED2,0);
            gpio_set_level(LED3,0);
            gpio_set_level(LED4,0);
            gpio_set_level(LED5,1);
            gpio_set_level(LED6,0);
        } else if (hall_value == 1) {
            gpio_set_level(LED1,1);
            gpio_set_level(LED2,0);
            gpio_set_level(LED3,0);
            gpio_set_level(LED4,0);
            gpio_set_level(LED5,0);
            gpio_set_level(LED6,0);
        } else if (hall_sensor_value == 3) {
            gpio_set_level(LED1,0);
            gpio_set_level(LED2,0);
            gpio_set_level(LED3,1);
            gpio_set_level(LED4,0);
            gpio_set_level(LED5,0);
            gpio_set_level(LED6,0);
        } else {
            gpio_set_level(LED1,0);
            gpio_set_level(LED2,0);
            gpio_set_level(LED3,0);
            gpio_set_level(LED4,0);
            gpio_set_level(LED5,0);
            gpio_set_level(LED6,0);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}