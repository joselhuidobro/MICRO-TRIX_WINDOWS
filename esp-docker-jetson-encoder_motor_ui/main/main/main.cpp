
// 23 interrumpe, 26 y 27 no interrumpen solo leen encoder cuando 23 interrumpe
#include <PID_v1.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#ifndef Motor_right
    #define Motor_right GPIO_NUM_22
#endif

#ifndef Motor_left
    #define Motor_left GPIO_NUM_21
#endif

#ifndef encoder_initA
    #define encoder_initA GPIO_NUM_26
#endif

#ifndef encoder_initB
    #define encoder_initB GPIO_NUM_27
#endif

volatile int master_count = 0;
double grados = 0.0;
double vueltas = 0.0;

// Variable para guardar el estado actual del GPIO22
volatile bool estadoMotor_right = 0;

double SetpointA, InputA, OutputA;
double SetpointB, InputB, OutputB;
double Kp = 2, Ki = 5, Kd = 1;

PID myPID_A(&InputA, &OutputA, &SetpointA, Kp, Ki, Kd, DIRECT);
PID myPID_B(&InputB, &OutputB, &SetpointB, Kp, Ki, Kd, REVERSE);

void IRAM_ATTR Encoder_isr_handler(void* arg) {
    // Leer el estado actual de los pines A y B
    bool initA = gpio_get_level(encoder_initA);
    bool initB = gpio_get_level(encoder_initB);

    // Guardar el estado anterior de los pines A y B
    static bool prev_initA = false;
    static bool prev_initB = false;

    // Si el estado de A ha cambiado, compararlo con el estado anterior de B
    if (initA != prev_initA) {
        if (initA == prev_initB) {
            master_count++;
        } else {
            master_count--;
        }
    }
    // Si el estado de B ha cambiado, compararlo con el estado anterior de A
    if (initB != prev_initB) {
        if (initB == prev_initA) {
            master_count--;
        } else {
            master_count++;
        }
    }

    // Si master_count es positivo, entonces el encoder está girando a la derecha
    if (master_count > 0) {
        gpio_set_level(Motor_right, 1); // Encender GPIO22
        gpio_set_level(Motor_left, 0);  // Apagar GPIO21
    }
    // Si master_count es negativo, entonces el encoder está girando a la izquierda
    else if (master_count < 0) {
        gpio_set_level(Motor_right, 0); // Apagar GPIO22
        gpio_set_level(Motor_left, 1);  // Encender GPIO21
    }

    // Actualizar los estados previos para la próxima interrupción
    prev_initA = initA;
    prev_initB = initB;
}



void setup() {
    
    // Configurar interrupción 
    
    gpio_set_intr_type(encoder_initA, GPIO_INTR_ANYEDGE); 
    gpio_set_intr_type(encoder_initB, GPIO_INTR_ANYEDGE); 
    
    gpio_set_direction(encoder_initA, GPIO_MODE_INPUT);
    gpio_set_pull_mode(encoder_initA, GPIO_PULLUP_ONLY);
    gpio_set_direction(encoder_initB, GPIO_MODE_INPUT);
    gpio_set_pull_mode(encoder_initB, GPIO_PULLUP_ONLY);

    gpio_install_isr_service(0);
    
    
    gpio_isr_handler_add(encoder_initA, Encoder_isr_handler, NULL);
    gpio_isr_handler_add(encoder_initB, Encoder_isr_handler, NULL);

    
    // Configurar GPIO22 como salida
    gpio_set_direction(Motor_right, GPIO_MODE_OUTPUT);
    gpio_set_direction(Motor_left, GPIO_MODE_OUTPUT);
    
    // PID setup
    SetpointA = 0;
    SetpointB = 0;

    myPID_A.SetMode(AUTOMATIC);
    myPID_B.SetMode(AUTOMATIC);
}

void loop() {
    //... (código original) ...

    ESP_LOGI("MotorControl", "OutputA: %f", OutputA);
    ESP_LOGI("MotorControl", "OutputB: %f", OutputB);
    ESP_LOGI("La cuenta es:", "%d", master_count); 
    
    grados = (master_count * 360.0) / 600.0;
    vueltas = master_count / 600.0;

    ESP_LOGI("MotorControl", "Grados: %f", grados);
    ESP_LOGI("MotorControl", "Vueltas: %f", vueltas);

    InputA = grados;
    InputB = grados;

    //... (más código original si es necesario) ...
}

void loop_task(void *pvParameter) {
    while (1) {
        loop();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

extern "C" {
void app_main() {
    setup();
    xTaskCreate(loop_task, "loop_task", 2048, NULL, 5, NULL);
}
}

/* """motor
|-- main/
|   |-- motor_pid_control.cpp
|   |-- PID_v1/
|   |   |-- PID_v1.h
|   |   !-- PID_v1.cpp
|   `-- CMakeLists.txt
!
|-- components/
!   !- arduino/
|   |   |-- cores/
|   |   |-- libraries/
|   |   |-- variants/
|   |   |-- tools/
|   |   |-- platform.txt
|   |   |-- programmers.txt
|   |   |-- README.md
|   |   |-- boards.txt
|   |   `-- CMakeLists.txt
!`  |- driver/
!        |`-- include/
!        |    `-- driver/
!        |        `-- ledc.h
|-- CMakeLists.txt
|-- sdkconfig.defaults """" 




docker-compose build
docker-compose up -d
sudo docker exec -it robot  bash
idf.py build
idf.py -p /dev/ttyUSB0 flash

*/


