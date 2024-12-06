#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "driver/mcpwm.h"
#include "esp_timer.h"

static const char* TAG = "Motor_Control";

// Pines de salida para el puente H trifásico
#define GPIO_PWM0A_OUT 33   // Fase AH (high)
#define GPIO_PWM0B_OUT 25   // Fase AL (low)
#define GPIO_PWM1A_OUT 32   // Fase BH (high)
#define GPIO_PWM1B_OUT 14   // Fase BL (low)
#define GPIO_PWM2A_OUT 27  // Fase CH (high)
#define GPIO_PWM2B_OUT 26   // Fase CL (low)

// Pines de entrada para los sensores Hall
#define GPIO_HALL_U    19
#define GPIO_HALL_V    18
#define GPIO_HALL_W    5

// Dead time en microsegundos (50 ms = 50000 us)
#define DEAD_TIME_US 50000

// Frecuencia PWM
#define PWM_FREQUENCY 20000  // 20 kHz

// Variables para el control de fase
volatile int current_phase = 0; // 0-5: 6 pasos de conmutación
int duty_cycle = 50; // Ciclo de trabajo inicial (0-100)

// Tabla de conmutación basada en sensores Hall
const int hall_table[8][3] = {
    {0, 0, 0},  // Estado inválido
    {0, 0, 1},  // Fase 1
    {0, 1, 0},  // Fase 2
    {0, 1, 1},  // Fase 3
    {1, 0, 0},  // Fase 4
    {1, 0, 1},  // Fase 5
    {1, 1, 0},  // Fase 6
    {0, 0, 0}   // Estado inválido
};

// Declaración de funciones
esp_err_t init_gpio(void);
esp_err_t init_mcpwm(void);
esp_err_t set_pwm_duty(int dutyA, int dutyB, int dutyC);
void IRAM_ATTR isr_handler(void* arg);
void switch_phase(void);

// Timer para las interrupciones de mitad de onda
esp_timer_handle_t half_wave_timer;

void app_main(void)
{
    init_gpio();
    init_mcpwm();

    // Configurar timer para interrupciones a mitad de onda
    esp_timer_create_args_t timer_config = {
        .callback = &isr_handler,
        .name = "half_wave_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_config, &half_wave_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(half_wave_timer, (1000000 / PWM_FREQUENCY) / 2)); // Interrupción a mitad de periodo

    ESP_LOGI(TAG, "Motor control iniciado");
}

esp_err_t init_gpio(void)
{
    // Configurar pines GPIO para el puente H
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_PWM2A_OUT);   // checar esto
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, GPIO_PWM2B_OUT);
    return ESP_OK;
}

esp_err_t init_mcpwm(void)
{
    // Configuración del MCPWM
    mcpwm_config_t pwm_config = {
        .frequency = PWM_FREQUENCY,
        .cmpr_a = 0,   // Inicialmente apagado
    //    .cmpr_b = 0,   // Inicialmente apagado (probablemente sin usar)
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_DOWN_COUNTER // Modo up-down para generar PWM simétrico
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // Configurar dead time
    mcpwm_deadtime_type_t deadtime_config = {
        .ucnt_a = MCPWM_TIMER_0,
        .dcnt_a = MCPWM_TIMER_0,
        .ucnt_b = MCPWM_TIMER_0,
        .dcnt_b = MCPWM_TIMER_0,
    };
    
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, &deadtime_config);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, &deadtime_config);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, &deadtime_config);
    
    mcpwm_set_deadtime(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, DEAD_TIME_US);
    mcpwm_set_deadtime(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, DEAD_TIME_US);
    mcpwm_set_deadtime(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, DEAD_TIME_US);

    return ESP_OK;
}

// Función para establecer el ciclo de trabajo PWM para cada fase
esp_err_t set_pwm_duty(int dutyA, int dutyB, int dutyC)
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyA);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100 - dutyA); // Complementario para fase baja
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dutyB);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 100 - dutyB); // Complementario para fase baja
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dutyC);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, 100 - dutyC); // Complementario para fase baja
    return ESP_OK;
}

// ISR para la interrupción de mitad de onda
void IRAM_ATTR isr_handler(void* arg)
{
    switch_phase();
}

// Función para cambiar la fase de conmutación del motor
void switch_phase(void)
{
    current_phase = (current_phase + 1) % 6; // 6 pasos de conmutación para un motor trifásico

    switch (current_phase) {
        case 0: // Fase A alta, B baja, C flotante
            set_pwm_duty(duty_cycle, 0, 0);
            break;
        case 1: // Fase A alta, C baja, B flotante
            set_pwm_duty(duty_cycle, 0, 100-duty_cycle);
            break;
        case 2: // Fase B alta, C baja, A flotante
             set_pwm_duty(0, duty_cycle, 100-duty_cycle);
            break;
        case 3: // Fase B alta, A baja, C flotante
            set_pwm_duty(0, duty_cycle, 0);
            break;
        case 4: // Fase C alta, A baja, B flotante
            set_pwm_duty(100-duty_cycle, 0, duty_cycle);
            break;
        case 5: // Fase C alta, B baja, A flotante
            set_pwm_duty(100-duty_cycle, duty_cycle, 0);
            break;
    }
}
