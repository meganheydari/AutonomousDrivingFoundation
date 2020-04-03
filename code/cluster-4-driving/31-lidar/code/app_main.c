/*
  Adapted I2C example code to work with the Adafruit ADXL343 accelerometer. Ported and referenced a lot of code from the Adafruit_ADXL343 driver code.
  ----> https://www.adafruit.com/product/4097

  Emily Lam, Aug 2019 for BU EC444
*/
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"


// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
/////
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// LIDARLite
#define LIDARLite_ADDRESS                         0x62

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
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
  //if (err == ESP_OK) {
    //printf("- initialized: yes\n");
  //}
  vTaskDelay(100/portTICK_RATE_MS);
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy

int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
  int ret = 0;
  // YOUR CODE HERE
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);  // 1. Start
  i2c_master_write_byte(cmd, ( LIDARLite_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN); // 2.-3.
  //give the reg address
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // 4.-5.
  // Master writes the data and waits for ack from slave
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
  i2c_master_stop(cmd); // 11. Stop
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); // This starts the I2C communication
  i2c_cmd_link_delete(cmd);
  return ret;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
  //return ret;
  // YOUR CODE HERE
  uint8_t data1 = 0;
  uint8_t data2 = 0;
  int16_t  overall;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);  // 1. Start
  i2c_master_write_byte(cmd, ( LIDARLite_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN); // 2.-3.
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // 4.-5.
  i2c_master_start(cmd); // 6.
  i2c_master_write_byte(cmd, ( LIDARLite_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);  // 7.-8.
  i2c_master_read_byte(cmd, &data1, ACK_VAL); // 9.-10.
  i2c_master_read_byte(cmd, &data2, ACK_CHECK_DIS); // 9.-10.
  i2c_master_stop(cmd); // 11. Stop
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  // push data 2 into first 8 bits and
  overall = (  ((int16_t) data2)  << 8) + ((int16_t) data1)  ;
  //overall =  (~ ((int16_t) overall) + 1);
  return overall;
}

void app_main() {
  // Routine
  i2c_master_init();
  i2c_scanner();
  uint16_t distance;
  float num;
  while(1) {
    writeRegister(0x00,0x04);
    vTaskDelay(1000 / portTICK_RATE_MS);
    distance = read16(0x8f);
    num = (float)distance/100;
    printf("distance: %.1fcm\n",num);
  }
}
