#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "contiki.h"
#include "sys/rtimer.h"
#include "board-peripherals.h"
#include <stdint.h>

#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include "net/linkaddr.h"
#include <string.h>
#include "node-id.h"

PROCESS(process_rtimer, "RTimer");
AUTOSTART_PROCESSES(&process_rtimer);

/* ------------ parameters ------------ */
#define SAMPLE_INTERVAL    CLOCK_SECOND          // 1 Hz
#define SAMPLES            60
#define CHUNK_SIZE         20                   // 3 chunks
#define SEND_CHUNK_INTERVAL      (RTIMER_SECOND / 4) // 250ms

/* ------------ packet types ------------ */
#define PKT_BEACON   0x01
#define PKT_DATA     0x03   // sensor chunk
#define PKT_ACK      0x04   // ack each chunk

// Global variables
static struct rtimer timer_rtimer;
static rtimer_clock_t interval = RTIMER_SECOND / 4; // SHOULD BE ONLY 1S

// Use broadcast address for neighbour discovery:
linkaddr_t dest_addr;

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  seq;
    int16_t  payload[CHUNK_SIZE*2];
  } data_pkt_t;

// Function declarations
static void init_opt_reading(void);
static void init_mpu_reading(void);
static int get_light_reading(void);
static float get_mpu_reading(void);

void send_chunks(struct rtimer *t, void *ptr);
static void receive_cb(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest);

/* ------------ buffers ------------ */
static int16_t light_buf[SAMPLES];     // store as 16‑bit to fit MTU
static int16_t motion_buf[SAMPLES];
static uint8_t sample_idx = 0;
// static uint8_t buffer_full = 0;

static int curr_chunk = 0;

/* ------------ link‑quality state ------------ */
typedef enum { LINK_SEARCHING = 0 , LINK_UP = 1 } link_state_t;
// static link_state_t link_state = LINK_SEARCHING;
static uint8_t good_cnt = 0;        // consecutive below‑threshold RSSI
#define RSSI_GOOD_THRESHOLD (-70) // dBm
static int peer_set = 0;
static linkaddr_t peer;  // peer address


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
  printf("Sample no. %d\n", sample_idx);
  printf("Light = %d.%02d\n", (int)light);
  // Print the MPU readings
  printf("MPU = %d.%02d\n", (int)mpu, (int)(mpu * 100) % 100);

  light_buf[sample_idx] = (int) light;
  motion_buf[sample_idx] = mpu;
    sample_idx++;
  
  // Schedule the next callback after 250ms.
  if (sample_idx < SAMPLES) {
      rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0, timer_callback, NULL);
  } else {
    // Send the chunk
    rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0, send_chunks, NULL);
  }
}

void receive_cb(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
    // Wait for a PKT_BEACON packet and get RSSI
    // If RSSI is good for 3 consecutive packets, set peer_set to 1
    if(len<1) return;

    uint8_t type = ((uint8_t*)data)[0];

    if(type == PKT_BEACON) {
        signed short rssi=(signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);
        printf("%lu RX_BEACON %02x:%02x RSSI %d\n",clock_seconds(),src->u8[0],src->u8[1],rssi);
        if(rssi>=RSSI_GOOD_THRESHOLD){
            if(!peer_set){ linkaddr_copy(&peer,src); peer_set=1; good_cnt=1; }
            else if(linkaddr_cmp(src,&peer)) good_cnt++;
        } else if(peer_set && linkaddr_cmp(src,&peer)) good_cnt=0;
    } else if (type == PKT_ACK) {
        uint8_t ackseq=((uint8_t*)data)[1];
        if(ackseq==curr_chunk){
            printf("ACK received for seq %d\n", ackseq);
            curr_chunk++;
        }
    }
}

void send_chunks(struct rtimer *t, void *ptr) {
    if (peer_set && good_cnt > 2) {
        data_pkt_t *pkt = (data_pkt_t *)ptr;
        pkt->type = PKT_DATA;
        pkt->seq = curr_chunk;
        for(uint8_t i=0; i<CHUNK_SIZE; i++){
            uint8_t idx = curr_chunk*CHUNK_SIZE + i;
            pkt->payload[2*i] = light_buf[idx];
            pkt->payload[2*i+1] = motion_buf[idx];
            printf("Sending chunk %d, sample %d: light %d, motion %d\n", curr_chunk, idx, pkt->payload[2*i], pkt->payload[2*i+1]);
        }
        nullnet_buf = (uint8_t *)pkt;
        nullnet_len = sizeof(data_pkt_t);
        printf("Sending chunk %d\n", curr_chunk);
        NETSTACK_NETWORK.output(&peer);
    } 
    rtimer_set(t, RTIMER_NOW() + SEND_CHUNK_INTERVAL, 0, send_chunks, ptr);
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

    nullnet_set_input_callback(receive_cb);
    
    rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0, timer_callback, NULL);

    PROCESS_END();
}