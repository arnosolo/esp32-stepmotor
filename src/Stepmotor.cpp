// #include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
// #include "soc/mcpwm_periph.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/pcnt.h"
#include "Stepmotor.h"

xQueueHandle Stepmotor::pcnt_evt_queue = xQueueCreate(12, sizeof(pcnt_evt_t));
bool Stepmotor::status[6] = {false, false, false, false, false, false};

static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
  int pcnt_unit = (int)arg;
  pcnt_evt_t evt;
  evt.unit = pcnt_unit;

  pcnt_get_event_status((pcnt_unit_t)pcnt_unit, &evt.status);
  // xQueueSendFromISR(Stepmotor::pcnt_evt_queue, &evt, NULL); // defult way

  // int id = pcnt_unit;
  Stepmotor::status[pcnt_unit] = true;
  mcpwm_stop(motor_config[pcnt_unit].mcpwm_unit, motor_config[pcnt_unit].mcpwm_timer);
}

Stepmotor::Stepmotor(int idInput, int dirPinInput, int stepPinInput)
{
  id = idInput;
  dirPin = dirPinInput;
  stepPin = stepPinInput;

  // 1.初始化转动方向引脚
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = 1ULL << dirPin;
  io_conf.mode = GPIO_MODE_OUTPUT;
  gpio_config(&io_conf);
  gpio_set_level((gpio_num_t)dirPin, 1);

  // 2.初始化脉冲计数器
  pcnt_unit_t unit = motor_config[id].pcnt_unit;
  pcnt_config_t pcnt_config;
  // Set PCNT input signal and control GPIOs
  pcnt_config.pulse_gpio_num = stepPin,
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED,
  pcnt_config.channel = PCNT_CHANNEL_0,
  pcnt_config.unit = unit,
  // What to do on the positive / negative edge of pulse input?
  pcnt_config.pos_mode = PCNT_COUNT_INC, // Count up on the positive edge
  pcnt_config.neg_mode = PCNT_COUNT_DIS, // Keep the counter value on the negative edge
  // What to do when control input is low or high?
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP,
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP, // Keep the primary counter mode if high
  // Set the maximum and minimum limit values to watch
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL,
  // .counter_l_lim = PCNT_L_LIM_VAL,
  pcnt_unit_config(&pcnt_config);

  /* Configure and enable the input filter */
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);

  pcnt_event_enable(unit, PCNT_EVT_H_LIM);

  /* Initialize PCNT's counter */
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  /* Install interrupt service and add isr callback handler */
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(unit, pcnt_example_intr_handler, (void *)unit);

  /* Everything is set up, now go to counting */
  pcnt_counter_resume(unit);

  // 3.生成电机控制脉冲
  // 细分为16,所以一圈需要3200步. 每秒800个脉冲,则电机大概4s转一圈
  ESP_ERROR_CHECK(mcpwm_gpio_init(motor_config[id].mcpwm_unit, motor_config[id].mcpwm_io_signals, stepPin));

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000; //frequency = 50Hz
  pwm_config.cmpr_a = 50;      //duty cycle of PWMxA = 50.0%
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  ESP_ERROR_CHECK(mcpwm_init(motor_config[id].mcpwm_unit, motor_config[id].mcpwm_timer, &pwm_config));


  // 这么搞一下同一个引脚就既能输出pwm,又能读取脉冲数了...
  gpio_iomux_in(stepPin, PCNT_SIG_CH0_IN0_IDX);

  isReady = true;
  Stepmotor::status[id] = isReady;
  const char *TAG = "motor class";
  ESP_LOGI(TAG, "init complete");
};

/**
 * @brief Ask step motor to move
 *
 * @param stepInput step number, should >= 0. Negative values ​​are converted to positive values
 * @param speedInput speed, step/s
 */
void Stepmotor::move(int stepInput, int speedInput)
{
  isReady = false;
  Stepmotor::status[id] = isReady;
  speed = speedInput;

  if(stepInput < 0) {
    step = -stepInput;
  } else {
    step = stepInput;
  }

  if (speed == 0)
  {
    stop();
  }
  else
  {
    if (speed < 0)
    {
      gpio_set_level((gpio_num_t)dirPin, 0);
      speed = -speedInput;
    }
    else
    {
      gpio_set_level((gpio_num_t)dirPin, 1);
    }

    pcnt_unit_t unit = motor_config[id].pcnt_unit;
    pcnt_set_event_value(unit, PCNT_EVT_H_LIM, (int16_t)step);
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);

    mcpwm_set_frequency(motor_config[id].mcpwm_unit, motor_config[id].mcpwm_timer, speed);
    mcpwm_start(motor_config[id].mcpwm_unit, motor_config[id].mcpwm_timer);
  }
};

void Stepmotor::stop()
{
  // isReady = true;
  // Stepmotor::status[id] = isReady;
  mcpwm_stop(motor_config[id].mcpwm_unit, motor_config[id].mcpwm_timer);
};
