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
#define HALL_1_PIN GPIO_NUM_13
#define HALL_2_PIN GPIO_NUM_14
#define HALL_3_PIN GPIO_NUM_15

#define ADC_CHANNEL_THROTTLE ADC1_CHANNEL_0
#define ADC_CHANNEL_ISENSE ADC1_CHANNEL_3
#define ADC_CHANNEL_VSENSE ADC1_CHANNEL_6

// Variables globales
uint8_t hallToMotor[8] = {255, 255, 255, 255, 255, 255, 255, 255}; // Tabla de estados del motor. Overwrite this with the output of the hall auto-idedntification
int duty_cycle = 0;
int current_ma = 0, voltage_mv = 0, current_target_ma = 0;
int hall = 0;


mcpwm_config_t pwm_config;

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_PWM2A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, GPIO_PWM2B_OUT);
    
    Motor.TabelaLimite = 0;
    // Center aligned pwm mode complementary mode
    // 15Khz frequency
    // 95% initial duty cibly
    pwm_config.frequency = 30000;    //frequency 
    pwm_config.cmpr_a = 95.0;       // Duty en porcentaje
    //pwm_config.cmpr_b = pwm_config.cmpr_a;    (?)    
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   //Configure PWM1A & PWM1B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);   //Configure PWM2A & PWM2B with above settings   

	  // deadtime (see clock source changes in mcpwm.c file)
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 80, 80);   // 1us deadtime
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 80, 80);   
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 80, 80);
    
    // TEZ interrupts (TIMERS in ZERO before ascending)
    MCPWM[MCPWM_UNIT_0]->int_ena.val = BIT(3) | BIT(4) | BIT(5); // /*A PWM timer X TEZ event will trigger this interrupt*/  
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

    // ------ READ THIS
    // mcpwm_sync_signal_t is defined only to SYNC0/1/2 that are for GPIO sync
    // In order to sync Timer1 and Timer2 with Timer0 
    // Pass 1 to mcpwm_sync_signal_t sync_sig in mcpwm_sync_enable
    // 1 is equal to "PWM timer0 sync_out" in PWM_TIMER_SYNCI_CFG_REG.PWM_TIMER1/2_SYNCISEL
    // Phase (last parameter) is zero.
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, 1, 0);   
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, 1, 0);  

    // Timer0 is our sync out. When it is equal to zero and before ascending sync out is triggered, so Timer1 and Timer2 stands in sync_in with Timer0.
    // When Timer0 is zero Timer1 and Timer2 is forced to phase defined in last parameter in functions above (0).
    // Make Timer0 sync out in TEZ
    MCPWM[MCPWM_UNIT_0]->timer[MCPWM_TIMER_0].sync.out_sel = 1;


    // esp32_technical_reference_manual_en.pdf around pages 439
    // mcpwm.h
    // mcpwm_struct.h






/*
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
#define GPIO_HALL_A    19
#define GPIO_HALL_B    18
#define GPIO_HALL_C    5

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
    {1, 1, 1}   // Estado inválido
};

// Declaración de funciones
esp_err_t init_gpio(void);
esp_err_t init_mcpwm(void);
esp_err_t set_pwm_duty(int dutyA, int dutyB, int dutyC);
void IRAM_ATTR isr_handler(void* arg);
void switch_phase(void);
int read_hall_sensors(void);

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
    // Configurar pines GPIO para los sensores Hall como entradas con pull-up interno
    gpio_config_t hall_config = {
        .pin_bit_mask = (1ULL << GPIO_HALL_A) | (1ULL << GPIO_HALL_B) | (1ULL << GPIO_HALL_C),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&hall_config);
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
    int hall_state = read_hall_sensors();
    switch_phase(hall_state);
}

// Función para leer el estado de los sensores Hall
int read_hall_sensors(void)
{
    int hall_a = gpio_get_level(GPIO_HALL_A);
    int hall_b = gpio_get_level(GPIO_HALL_B);
    int hall_c = gpio_get_level(GPIO_HALL_C);
    return (hall_a << 2) | (hall_b << 1) | hall_c;
}

// Función para cambiar la fase de conmutación del motor basada en el estado de los sensores Hall
void switch_phase(int hall_state)
{
    switch (hall_state) {
        case 1: // 001
            set_pwm_duty(duty_cycle, 0, 100-duty_cycle); // Fase U alta, W baja, V flotante
            break;
        case 2: // 010
            set_pwm_duty(100-duty_cycle, duty_cycle, 0); // Fase W alta, V baja, U flotante
            break;
        case 3: // 011
            set_pwm_duty(0, duty_cycle, 100-duty_cycle); // Fase V alta, W baja, U flotante
            break;
        case 4: // 100
            set_pwm_duty(duty_cycle, 100-duty_cycle, 0); // Fase U alta, V baja, W flotante
            break;
        case 5: // 101
            set_pwm_duty(100-duty_cycle, 0, duty_cycle); // Fase W alta, U baja, V flotante
            break;
        case 6: // 110
            set_pwm_duty(0, 100-duty_cycle, duty_cycle); // Fase V alta, U baja, W flotante
            break;
        default: // Estado inválido o no esperado, detener el motor
            set_pwm_duty(0, 0, 0);
            break;
    }
}
*/
