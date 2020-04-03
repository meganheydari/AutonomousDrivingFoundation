/*
	Copyright © 2019 Chase Clarke, Megan Heydari, Leila Lee. All rights reserved.

	This code is built for operation of an autonomous remote controlled car using the following:

		2 micro lidars - side sensing priximit sensor
		1 ultrasonic - front sensing proximity sensor
		1 photosensing LED pair - for wheel speed detection
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include <math.h>

//micro lidar
#include "driver/uart.h"
#include "driver/gpio.h"
// end lidar

//timer
#include "esp_types.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
//end timer

//udp
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
// end udp

// pcnt speed
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
// end pcnt speed


// defines for seven seg and ultrasonic
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_vfs_dev.h"
#include "esp_adc_cal.h"

// Serial
#include "esp_system.h"
#include "esp_log.h"

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_0;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

#define DEFAULT_VREF    1100
uint32_t ultra_distance; //cm

//display functions
static void i2c_example_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {printf("- initialized: yes\n\n");}

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// end seven seg and ultrasonic

//pcnt speed
#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      10000
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    10000
#define PCNT_THRESH0_VAL   -5
#define PCNT_INPUT_SIG_IO   34  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=count up, LOW=count down
#define LEDC_OUTPUT_IO      18 // Output GPIO of a sample 1 Hz pulse generator


xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle
int16_t count = 0;
int16_t prevCount = 0; 

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

// end pcnt speed


// udp
u16_t CONFIGPORT = 8080;
const char* IP = "192.168.1.108";

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR IP
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif
#define PORT CONFIGPORT

static const char *TAG = "example";
static const char *payload = "ACK";
static const char *initPayload = "INIT";
TaskHandle_t  UDPHandle = NULL;
//end udp

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

//micro lidar 1
#define ECHO_TEST_TXD  (17)
#define ECHO_TEST_RXD  (16)

#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define BUF_SIZE (128)
// end lidar

//microliadr 2
#define ECHO_TEST_TXD_2  (25) //a1
#define ECHO_TEST_RXD_2  (26) //a0

//end lidar 2

// timer
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (1.00) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (1.00)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;
TaskHandle_t  TimerHandle = NULL;
//end timer

uint32_t currentAngle = 0;
uint32_t targetAngle = 0;
TaskHandle_t turnHandle = NULL;

uint32_t currentPWM = 1400;
uint32_t targetPWM = 0;
TaskHandle_t  spinHandle = NULL;

//micro lidar 1
int distanceBack = 0;
TaskHandle_t  LidarHandle = NULL;

//micro lidar 2
//distance in mm
int distanceFront = 0;
TaskHandle_t  LidarHandle2 = NULL;

double dt = TIMER_INTERVAL1_SEC;
double previous_error = 0;     // Set up PID loop
double integral = 0;
//increase p reduces rise time
double Kp = 40.00;
double Ki = 1.00;
//increase d improves stability and decreases overshoot
double Kd = 0.01;

TaskHandle_t  pnctSpeedHandle = NULL;

double PID(double setpoint, double measured_value) {

    double error = setpoint - measured_value;
    integral = integral + error * dt;
    double derivative = (error - previous_error) / dt;
    double output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    return output;
}

//timer support function
void IRAM_ATTR timer_group0_isr(void *para)
{
    int timer_idx = (int) para;

    timer_intr_t timer_intr = timer_group_intr_get_in_isr(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    if (timer_intr & TIMER_INTR_T0) {
        evt.type = TEST_WITHOUT_RELOAD;
        timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_0);
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, timer_idx, timer_counter_value);
    } else if (timer_intr & TIMER_INTR_T1) {
        evt.type = TEST_WITH_RELOAD;
        timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_1);
    } else {
        evt.type = -1; // not supported even type
    }

    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

//init timer
static void example_tg0_timer_init(int timer_idx, bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    #ifdef CONFIG_IDF_TARGET_ESP32S2BETA
        config.clk_sel = TIMER_SRC_CLK_APB;
    #endif
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

//translate raw int to degree for turn
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

// micro lidar serial read function
void microLidar()
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 10 / portTICK_RATE_MS);

        int i = 0;
        int prev = 0;
        for(i = 0; i < len - 1; i++) {

            if (data[i] == 0x59 && prev == 0x59) {
                distanceFront = data[i + 1];
                //printf("data: %d\n", data[i + 1]);
            }

            prev = data[i];
        }
    }
}

// handles the creating and destroying of tasks that read the microLidar.
void lidarHandler()
{
    printf("running lidar handler\n");
    if(LidarHandle != NULL)
    {
        vTaskDelete(LidarHandle);
        LidarHandle = NULL;
    }

    xTaskCreate(microLidar, "microLidar", 4096, NULL, 5, &LidarHandle);  
    configASSERT(LidarHandle);
}

// micro lidar serial read function
void microLidar2()
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 10 / portTICK_RATE_MS);

        int i = 0;
        int prev = 0;
        for(i = 0; i < len - 1; i++) {

            if (data[i] == 0x59 && prev == 0x59) {
                distanceBack = data[i + 1];
                //printf("data: %d\n", data[i + 1]);
            }

            prev = data[i];
        }
    }
}

// handles the creating and destroying of tasks that read the microLidar.
void lidarHandler2()
{
    printf("running lidar handler\n");
    if(LidarHandle2 != NULL)
    {
        vTaskDelete(LidarHandle2);
        LidarHandle2 = NULL;
    }

    xTaskCreate(microLidar2, "microLidar2", 4096, NULL, 5, &LidarHandle2);  
    configASSERT(LidarHandle2);
}

// task that performs the wheel turning incrementally to avoid harware strain.
void perfromTurn()
{
    uint32_t angle, count;

    while (targetAngle != currentAngle) {
        if (targetAngle > currentAngle){
            for (count = currentAngle; count <= targetAngle; count++) {
                angle = servo_per_degree_init(count);
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
                
            }
            currentAngle = targetAngle;
        }
        else {
            for (count = currentAngle; count > targetAngle; count--) {
                angle = servo_per_degree_init(count);
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
                
            }
            currentAngle = targetAngle;
        }
    }
    turnHandle = NULL;
    vTaskDelete(turnHandle);
}

// handles the creating and destroying of tasks that turn the wheels.
void turnHandler(int target) 
{
    targetAngle = target;
    printf("Start turn thead. angle changing to: %d\n", targetAngle);

    if(turnHandle != NULL)
    {
        vTaskDelete(turnHandle);
        turnHandle = NULL;
    }

    xTaskCreate(perfromTurn, "perfromTurn", 4096, NULL, 5, &turnHandle);  
    configASSERT(turnHandle);
}

// initialize the connection to the crawler
void initDevice() 
{
    // 21 goes to spinning
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 21);    //Set GPIO 18 as PWM0A, to which servo is connected

    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    // 18 goes to servo steering
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 32);    //Set GPIO 18 as PWM0A, to which servo is connected
    
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;        //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;        //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    // for installing the microLidarc1
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    //  init second lidar
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, ECHO_TEST_TXD_2, ECHO_TEST_RXD_2, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    vTaskDelay(5000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
}

// test all turning and movement of crawler
void calibrate()
{
    printf("Perform wheel turn calibration:\n");

    // left
    turnHandler(90);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // right
    turnHandler(0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    //forward
    turnHandler(45);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Perform wheel spin calibration:\n");  
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
    vTaskDelay(500 / portTICK_PERIOD_MS);

        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
    vTaskDelay(500 / portTICK_PERIOD_MS);

        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
    vTaskDelay(500 / portTICK_PERIOD_MS);

}

// main logic for timer.
static void timer_example_evt_task(void *arg)
{
    double temp;
    double currentPID;
    while (1) {
        
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // temp = PID(distanceBack, distanceFront);

        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, currentAngle + temp);
        // currentAngle = currentAngle + temp;

        // printf("PID: %d, back dist: %d, front distance: %d\n", temp, distanceBack, distanceFront);

        pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
        // printf("Count per 10 sec: %d\n", count- prevCount);
        double newCount = count-prevCount;
        prevCount = count; 
        // 6 pulses per revolution, counting for 10 sec
        double rpm = (newCount/6) * 60/TIMER_INTERVAL1_SEC;  // divide by pulses per rev, mult by time blocks per min
        // 0.62 m per revolution 
        double mps = rpm*0.62 /60;

        temp = PID(0.4, mps);
        currentPID = 1290 + temp;

        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, currentPID);

        printf("== rpm: %0.2f, mps: %0.2f PWM: %f TEMP: %f\n", rpm, mps, currentPID, temp);
        //vTaskDelay(1000 / portTICK_RATE_MS);
       
    }
}

// handles the creating and destroying of tasks that start the timer.
void timerHandler()
{
    printf("starting timer process\n");

    if(TimerHandle != NULL)
    {
        vTaskDelete(TimerHandle);
        TimerHandle = NULL;
    }

    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    // use TEST_WITHOUT_RELOAD for without reload
    example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD,    TIMER_INTERVAL1_SEC);
    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, &TimerHandle);
    configASSERT(TimerHandle);
}

//pcnt speed
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

//pcnt speed
static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode       = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_1;
    ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT;
    ledc_timer.freq_hz          = 4;  // set output frequency at 1 Hz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.channel    = LEDC_CHANNEL_1;
    ledc_channel.timer_sel  = LEDC_TIMER_1;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num   = LEDC_OUTPUT_IO;
    ledc_channel.duty       = 100; // set duty at about 10%
    ledc_channel.hpoint     = 0;
    ledc_channel_config(&ledc_channel);
}

//pcnt speed
static void pcnt_example_init(void)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_TEST_UNIT,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
    pcnt_filter_enable(PCNT_TEST_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_TEST_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_TEST_UNIT);
}

// detection of wheel turn speed with LED pair
void pcntSpeed()
{
    /* Initialize LEDC to generate sample pulse signal */
    ledc_init();

    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init();

    // timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    // // example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
    // example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD, TIMER_INTERVAL1_SEC);
    // xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    printf("Speed will update every %.2f seconds\n", TIMER_INTERVAL1_SEC);
    
    pcnt_evt_t evt;
    //portBASE_TYPE res;
    while (1) {
        //res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
        xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
    }

    
    if(user_isr_handle) {
        //Free the ISR service handle.
        esp_intr_free(user_isr_handle);
        user_isr_handle = NULL;
    }
}

// handles the creation and deletion of tasks relating to speed detection
void pnctSpeedHandler() 
{
    printf("running pnctSpeedHandler handler\n");
    if(pnctSpeedHandle != NULL)
    {
        vTaskDelete(pnctSpeedHandle);
        pnctSpeedHandle = NULL;
    }

    xTaskCreate(pcntSpeed, "pcntSpeed", 4096, NULL, 5, &pnctSpeedHandle);  
    configASSERT(pnctSpeedHandle);
}

// main logic for udp connection to the webserver
static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

        #ifdef CONFIG_EXAMPLE_IPV4
                struct sockaddr_in dest_addr;
                dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
                dest_addr.sin_family = AF_INET;
                dest_addr.sin_port = htons(PORT);
                addr_family = AF_INET;
                ip_protocol = IPPROTO_IP;
                inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        #else // IPV6
                struct sockaddr_in6 dest_addr;
                inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
                dest_addr.sin6_family = AF_INET6;
                dest_addr.sin6_port = htons(PORT);
                addr_family = AF_INET6;
                ip_protocol = IPPROTO_IPV6;
                inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
        #endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        int err = sendto(sock, initPayload, strlen(initPayload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
        ESP_LOGI(TAG, "Init message sent");
            
        while (1) {
            
        
            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                printf("%s\n", rx_buffer);

                if (strcmp(rx_buffer,"/start") == 0) {
                    // START LOGIC
                    printf("START\n");

                    //start the movement of the crawler
                    timerHandler();


                    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                    if (err < 0) {
                        ESP_LOGE(TAG, "Error occurred during ACK sending: errno %d", errno);
                        break;
                    } ESP_LOGI(TAG, "START ACK message sent");


                }

                if (strcmp(rx_buffer,"/stop") == 0) {
                    // STOP LOGIC
                    printf("STOP\n");

                    // deleting the timing task
                    vTaskDelete(TimerHandle);
                    TimerHandle = NULL;
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
                    dt = TIMER_INTERVAL1_SEC;
                    previous_error = 0;   
                    integral = 0;


                    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                    if (err < 0) {
                        ESP_LOGE(TAG, "Error occurred during ACK sending: errno %d", errno);
                        break;
                    } ESP_LOGI(TAG, "STOP ACK message sent");

                }

                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

// handles creation and deletion of udp connection to webserver
void UDPHandler()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    printf("running udp handler\n");
    if(UDPHandle != NULL)
    {
        vTaskDelete(UDPHandle);
        UDPHandle = NULL;
    }
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, &UDPHandle);
    configASSERT(UDPHandle);
}

////////////////////////////////////////////////////////////////////////////////

// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

static void test_alpha_display() {
    // Debug
    uint16_t fonttable[11];
    uint16_t map[5];
    char buf[5];
    fonttable[0] = 0b0000110000111111; // 0
    fonttable[1] = 0b0000000000000110; // 1
    fonttable[2] = 0b0000000011011011; // 2
    fonttable[3] = 0b0000000010001111; // 3
    fonttable[4] = 0b0000000011100110; // 4
    fonttable[5] = 0b0010000001101001; // 5
    fonttable[6] = 0b0000000011111101; // 6
    fonttable[7] = 0b0000000000000111; // 7
    fonttable[8] = 0b0000000011111111; // 8
    fonttable[9] = 0b0000000011101111; // 9
    fonttable[10] = 0b0000000000000000;

    int ret;

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    while (1) {
      //printf("%d\n", ultra_distance);
      sprintf(buf,"%d",ultra_distance);
      //printf("%s\n", buf);

        for (int i = 0; i<buf[i]; i++) {
            int num = (int) buf[i];
            map[i] = fonttable[num-48];
        }

        // Write to characters to bufferASDFGHJKLs
        uint16_t displaybuffer[8];

        displaybuffer[0] = map[0];
        displaybuffer[1] = map[1];
        displaybuffer[2] = map[2];
        displaybuffer[3] = map[3];

        map[0] = fonttable[10];
        map[1] = fonttable[10];
        map[2] = fonttable[10];
        map[3] = fonttable[10];


      // Send commands characters to display over I2C
      i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
      i2c_master_start(cmd4);
      i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
      for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
      }
      i2c_master_stop(cmd4);
      ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd4);

    }
}

// reads ultrasonic data
void ultrasonic()
{
    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(channel, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars);
    //print_char_val_type(val_type);

    while (1) {
        int adc_reading = 0;
        //Multisampling
        for (int i = 0; i < 15; i++) {
            adc_reading+=adc1_get_raw((adc1_channel_t)channel);
        }
        adc_reading /= 15;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("Ultrasonic Distance: %.3fm\n", voltage / 6.4 * 2.54 / 100); //AN yields 6.4mV/in. for 3.3V
        ultra_distance = (uint32_t)voltage / 6.4 * 2.54;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// end alpha displays


void app_main(void)
{
    //initialize the connection to turning servo
    //  and wheel spinning motor
    initDevice();

    //calibrate connection to both
    calibrate();

    //initialize UDP connection to webserver
    UDPHandler();

    //initialize speed data gathering
    pnctSpeedHandler();


    // start data collection on the micro lidars
    lidarHandler();
    lidarHandler2();

    // start data collection on the ultrasonic
    i2c_example_master_init();
    xTaskCreate(ultrasonic, "ultrasonic", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(test_alpha_display,"test_alpha_display", 4096, NULL, configMAX_PRIORITIES-1, NULL);


    // logic below handles the wheel turning given sensor data.
    int turnError = 0;
    int difference = 16;
    double localAngle = 0;
    while(1) {

        if(ultra_distance < 50) {
            // deleting the timing task

            if(TimerHandle != NULL)
            {
                vTaskDelete(TimerHandle);
                TimerHandle = NULL;
            }            
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
        }

        printf("Dfront: %d, Dback: %d, Ultra: %d\n", distanceFront, distanceBack, ultra_distance);
        turnError = distanceFront - distanceBack;
        localAngle = (45/difference)*turnError + 45;

        if (localAngle < 0) {
            localAngle = 0;
        }
        if (localAngle > 90) {
            localAngle = 90;
        }

        printf("angle: %f\n", localAngle);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, servo_per_degree_init(localAngle));



        vTaskDelay(1000 / portTICK_RATE_MS);

    }
}
