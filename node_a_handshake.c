// TRANSMITTER CODE

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

#define SAMPLES 60 // No. of samples we are collecting
#define CHUNK_SIZE 20 // Number of readings in each packet (chunk)
#define SEND_CHUNK_INTERVAL (RTIMER_SECOND / 4) // Interval between sending chunks
#define MAX_CHUNK_TRIES 20 // Max tries to send a chunk before giving up

#define WAKE_TIME (RTIMER_SECOND / 10)   // Wake time for neighbour discovery
#define SLEEP_SLOT (RTIMER_SECOND / 10)   // Sleep time between receiving

// Packet types
#define PKT_BEACON 0x01
#define PKT_REQUEST 0x02
#define PKT_DATA 0x03
#define PKT_ACK 0x04
#define PKT_REQ_ACK 0x05


typedef struct __attribute__((packed)) {
  uint8_t type;   // Packet type
  uint16_t src_id;
} req_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint16_t src_id;
  uint8_t seq;   // Chunk number
} ack_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint16_t src_id;
  uint8_t seq;
  int16_t payload[CHUNK_SIZE * 2]; // Light and motion data
} data_pkt_t;

static data_pkt_t data_packet;


typedef enum { LINK_SEARCHING = 0, LINK_UP = 1 } link_state_t;
static link_state_t link_state = LINK_SEARCHING;

static struct rtimer rt;
static rtimer_clock_t sampling_interval = RTIMER_SECOND; // Sampling interval for 1hz
static int16_t light_readings[SAMPLES];
static int16_t motion_readings[SAMPLES];
static uint8_t sample_idx = 0;
static int curr_chunk = 0;

static uint8_t good_cnt = 0; // No. of consecutive REQ_ACK packets with good RSSI
static uint8_t curr_chunk_tries = 0;
#define RSSI_GOOD_THRESHOLD (-70)
static int peer_set = 0;
static linkaddr_t peer;

static int awaiting_ack = 0; 
static int last_sent_seq = -1; 

static void send_chunks(struct rtimer *t, void *ptr);
static void send_request(struct rtimer *t, void *ptr);
static void end_listening(struct rtimer *t, void *ptr);
static void listen_chunk_ack(struct rtimer *t, void *ptr);

static void init_opt_reading(void) { 
  SENSORS_ACTIVATE(opt_3001_sensor); 
}
static void init_mpu_reading(void){ 
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL); 
}

static int get_light_reading(void){
  int val = opt_3001_sensor.value(0);
  init_opt_reading();
  if(val == CC26XX_SENSOR_READING_ERROR) {
    return -1;
  } else {
    return val/100;
  } 
}
static float get_mpu_reading(void){
  int ax = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X)/100;
  int ay = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y)/100;
  int az = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z)/100;
  return sqrtf((float)(ax*ax + ay*ay + az*az));
}

// Send request packets to discover neighbours
static void send_request(struct rtimer *t, void *ptr){
  if(link_state != LINK_SEARCHING) return;

  req_pkt_t req = { PKT_REQUEST, node_id };
  nullnet_buf = (uint8_t *)&req;
  nullnet_len = sizeof(req);
  NETSTACK_NETWORK.output(NULL);
  printf("Sending Request Packet\n");

  NETSTACK_RADIO.on();

  rtimer_set(t, RTIMER_NOW() + WAKE_TIME, 0, end_listening, NULL);
}

static void end_listening(struct rtimer *t, void *ptr){
  NETSTACK_RADIO.off();

  if(link_state == LINK_SEARCHING){
      // Didn't receive any REQ_ACK packets – sleep and schedule next send request
      rtimer_set(t, RTIMER_NOW() + SLEEP_SLOT, 0, send_request, NULL);
  } else if(link_state == LINK_UP){
      // Discoverd a neighbour – start sending chunks
      rtimer_set(t, RTIMER_NOW() + SEND_CHUNK_INTERVAL, 0, send_chunks, NULL);
  }
}

static void timer_callback(struct rtimer *t, void *ptr){
  light_readings[sample_idx] = get_light_reading();
  motion_readings[sample_idx] = (int)get_mpu_reading();
  printf("COLLECTING DATA: Sample %u light=%d mpu=%d\n", sample_idx, light_readings[sample_idx], motion_readings[sample_idx]);
  sample_idx++;

  if(sample_idx < SAMPLES){
    rtimer_set(&rt, RTIMER_NOW() + sampling_interval, 0, timer_callback, NULL);
  } else {
    curr_chunk = 0;
    link_state = LINK_SEARCHING;
    peer_set = 0;
    good_cnt = 0;
    rtimer_set(&rt, RTIMER_NOW() + sampling_interval, 0, send_request, NULL);
  }
}

static void receive_cb(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest){
  if(len == 0) {
    return;
  }
  uint8_t type = ((const uint8_t*)data)[0];

  if(type == PKT_REQ_ACK) {
    ack_pkt_t *ackp = (ack_pkt_t *)data;
    uint16_t   sender_id = ackp->src_id;

    signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);

    if(!peer_set){
        linkaddr_copy(&peer, src);
        peer_set = 1;
        good_cnt = 0;
    }
    if(linkaddr_cmp(src,&peer) && rssi >= RSSI_GOOD_THRESHOLD){
        good_cnt++;
    } else if(linkaddr_cmp(src,&peer)){
        good_cnt = 0;
    }

    printf("%lu DETECT node %u REQ_ACK cnt=%u rssi=%d\n", clock_seconds(), sender_id, good_cnt, rssi);

    if(good_cnt >= 3 && link_state == LINK_SEARCHING){
        link_state = LINK_UP;
        printf("Establishing good connection with neighbour - starting data transfer\n\n");
        // first chunk will be scheduled by end_listening()
    }
  } else if(type == PKT_ACK) {
    ack_pkt_t *ack = (ack_pkt_t *)data;
    uint8_t ackseq = ack->seq;
    signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);

    if(ackseq == last_sent_seq){
      awaiting_ack = 0;
    }

    if(ackseq == curr_chunk) {
      uint16_t sender_id = ack->src_id;
      printf("%lu DETECT node %u  PKT_ACK seq=%u  rssi=%d\n", clock_seconds(), sender_id, curr_chunk, rssi);

      if((curr_chunk + 1)*CHUNK_SIZE >= SAMPLES){
        printf("Transfer complete\n");
        memset(light_readings, 0, sizeof(light_readings));
        memset(motion_readings,0, sizeof(motion_readings));
        sample_idx = 0;
        peer_set   = 0;
        good_cnt   = 0;
        curr_chunk = -1;
      }else{
        curr_chunk++;
      }
    }
  }
}

static void send_chunks(struct rtimer *t, void *ptr) {
  if(link_state != LINK_UP || curr_chunk == -1){
    return;
  }
  // printf in the format: <timestamp_in_seconds> TRANSFER <nodeID>
  printf("%lu TRANSFER-FROM %u\n", clock_seconds(), node_id);

  last_sent_seq = curr_chunk;
  awaiting_ack = 1;

  data_packet.type = PKT_DATA;
  data_packet.src_id = node_id;
  data_packet.seq = curr_chunk;
  for(uint8_t i=0;i<CHUNK_SIZE;i++){
    uint8_t idx = curr_chunk*CHUNK_SIZE + i;
    data_packet.payload[2*i] = light_readings[idx];
    data_packet.payload[2*i +1 ] = motion_readings[idx];
  }
  nullnet_buf = (uint8_t*)&data_packet;
  nullnet_len = sizeof(data_packet);
  NETSTACK_NETWORK.output(&peer);

  NETSTACK_RADIO.on();

  rtimer_set(t, RTIMER_NOW() + WAKE_TIME, 0, listen_chunk_ack, NULL);
}

static void listen_chunk_ack(struct rtimer *t, void *ptr){
  NETSTACK_RADIO.off();

  if(link_state != LINK_UP) return;

  if(awaiting_ack){
    rtimer_set(t, RTIMER_NOW() + SLEEP_SLOT, 0, send_chunks, NULL);
  }else if(curr_chunk != -1){
    rtimer_set(t, RTIMER_NOW() + SEND_CHUNK_INTERVAL, 0, send_chunks, NULL);
  }
}


PROCESS_THREAD(process_rtimer, ev, data){
  PROCESS_BEGIN();
  init_opt_reading();
  init_mpu_reading();
  nullnet_set_input_callback(receive_cb);

  rtimer_set(&rt, RTIMER_NOW() + sampling_interval, 0, timer_callback, NULL);
  PROCESS_END();
}
