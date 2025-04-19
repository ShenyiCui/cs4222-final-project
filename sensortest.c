#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "contiki.h"
#include "sys/rtimer.h"
#include "board-peripherals.h"
#include <stdint.h>

PROCESS(process_rtimer, "RTimer");
AUTOSTART_PROCESSES(&process_rtimer);

// Global variables
static struct rtimer timer_rtimer;
static rtimer_clock_t interval = RTIMER_SECOND / 4;

// Function declarations
static void init_opt_reading(void);
static void init_mpu_reading(void);
static int get_light_reading(void);
static float get_mpu_reading(void);

/* 
 * timer_callback() is invoked every 250ms.
 * It both polls sensors and advances the state machine.
 */
void timer_callback(struct rtimer *t, void *ptr) {
  int light;
  float mpu;

  light = get_light_reading();
  mpu = get_mpu_reading();

  // Print the Light readings
  printf("Light = %d.%02d\n", (int)light, (int)(light * 100) % 100);
  // Print the MPU readings
  printf("MPU = %d.%02d\n", (int)mpu, (int)(mpu * 100) % 100);
  
  // Schedule the next callback after 250ms.
  rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0, timer_callback, NULL);
}

static int get_light_reading() {
    int value = opt_3001_sensor.value(0);
    if(value != CC26XX_SENSOR_READING_ERROR) {
       init_opt_reading();
       return value / 100;
    } else {
       printf("Light sensor warming up\n");
       init_opt_reading();
       return -1;
    }
}

static float get_mpu_reading() {
    int ax = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X) / 100;
    int ay = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y) / 100;
    int az = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z) / 100;
    float x = (float)ax, y = (float)ay, z = (float)az;
    return sqrt(x * x + y * y + z * z);
}

static void init_opt_reading(void) {
    SENSORS_ACTIVATE(opt_3001_sensor);
}

static void init_mpu_reading(void) {
    mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

PROCESS_THREAD(process_rtimer, ev, data) {
    PROCESS_BEGIN();
    init_opt_reading();
    init_mpu_reading();
    
    while(1) {
        // Start the periodic callback (every 250 ms)
        rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0, timer_callback, NULL);
        PROCESS_YIELD();
    }

    PROCESS_END();
}