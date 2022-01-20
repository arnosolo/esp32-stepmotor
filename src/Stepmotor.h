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

#define STEPMOTOR_NUM 6
#define PCNT_H_LIM_VAL 1000

typedef struct
{
    int unit;        // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

typedef struct
{
  mcpwm_unit_t mcpwm_unit;
  mcpwm_timer_t mcpwm_timer;
  mcpwm_io_signals_t mcpwm_io_signals;
  pcnt_unit_t pcnt_unit;
  // pcnt_channel_t pcnt_channel;
} stepmotor_config_t;

static stepmotor_config_t motor_config[STEPMOTOR_NUM] = {
    {.mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_timer = MCPWM_TIMER_0,
    .mcpwm_io_signals = MCPWM0A,
    .pcnt_unit = PCNT_UNIT_0
    },
    {.mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_timer = MCPWM_TIMER_1,
    .mcpwm_io_signals = MCPWM1A,
    .pcnt_unit = PCNT_UNIT_1
    },
    {.mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_timer = MCPWM_TIMER_2,
    .mcpwm_io_signals = MCPWM2A,
    .pcnt_unit = PCNT_UNIT_2
    },
    {.mcpwm_unit = MCPWM_UNIT_1,
    .mcpwm_timer = MCPWM_TIMER_0,
    .mcpwm_io_signals = MCPWM0A,
    .pcnt_unit = PCNT_UNIT_3
    },
    {.mcpwm_unit = MCPWM_UNIT_1,
    .mcpwm_timer = MCPWM_TIMER_1,
    .mcpwm_io_signals = MCPWM1A,
    .pcnt_unit = PCNT_UNIT_4
    },
    {.mcpwm_unit = MCPWM_UNIT_1,
    .mcpwm_timer = MCPWM_TIMER_2,
    .mcpwm_io_signals = MCPWM2A,
    .pcnt_unit = PCNT_UNIT_5
    },
};

class Stepmotor
{
private:
public:
  int id;
  int dirPin;
  int stepPin;
  int speed;
  int step;
  bool isReady;

  static xQueueHandle pcnt_evt_queue;
  static bool status[6];

  Stepmotor(int idInput, int dirPinInput, int stepPinInput);
  void init();
  void move(int stepInput, int speedInput);
  void stop();

};
