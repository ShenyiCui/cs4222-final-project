// ===============================================
// Node A  (sensor / transmitter)  –  node_a.c
// ===============================================

#include "contiki.h"
#include "net/nullnet/nullnet.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "board-peripherals.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ------------ parameters ------------ */
#define SAMPLE_INTERVAL    CLOCK_SECOND          // 1 Hz
#define SAMPLES            60
#define CHUNK_SIZE         20                   // 3 chunks
#define BEACON_PERIOD      (2*CLOCK_SECOND)
#define RSSI_THRESHOLD     (-70)                // dBm
#define GOOD_REQUIRED      3

/* ------------ packet types ------------ */
#define PKT_BEACON   0x01
#define PKT_REQUEST  0x02   // start transfer
#define PKT_DATA     0x03   // sensor chunk
#define PKT_ACK      0x04   // ack each chunk

/* ------------ sensor helpers (reuse Assignment‑2) ------------ */
static void init_opt(void) { SENSORS_ACTIVATE(opt_3001_sensor);} 
static void init_mpu(void) { mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);} 

static int get_light(void) { 
  return 50;
}
static int16_t get_motion_scaled(void) {
  return (int16_t) 50;
}

/* ------------ buffers ------------ */
static int16_t light_buf[SAMPLES];     // store as 16‑bit to fit MTU
static int16_t motion_buf[SAMPLES];
static uint8_t sample_idx = 0;
static uint8_t buffer_full = 0;

/* ------------ state vars ------------ */
static struct etimer sample_timer, beacon_timer;
static linkaddr_t peer;  static uint8_t peer_set=0;
static uint8_t good_cnt=0, sending=0, seq_idx=0;

/* data packet structure (fits in 127‑byte frame) */
typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  seq;
  int16_t  payload[CHUNK_SIZE*2]; // light,motion interleaved
} data_pkt_t;
static data_pkt_t dpkt;

/* forward */
static void send_beacon(void);
static void start_transfer(void);
static void send_chunk(uint8_t seq);

/* ------------ input callback ------------ */
static void rx_cb(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
  if(len==1 && *(uint8_t*) data == PKT_BEACON) {
    signed short rssi=(signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);
    printf("%lu RX_BEACON %02x:%02x RSSI %d\n",clock_seconds(),src->u8[0],src->u8[1],rssi);
    if(rssi>=RSSI_THRESHOLD){
      if(!peer_set){ linkaddr_copy(&peer,src); peer_set=1; good_cnt=1; }
      else if(linkaddr_cmp(src,&peer)) good_cnt++; }
    else if(peer_set && linkaddr_cmp(src,&peer)) good_cnt=0;

    if(buffer_full && peer_set && good_cnt>=GOOD_REQUIRED && !sending){
      printf("%lu DETECT %u\n",clock_seconds(),peer.u8[7]);
      uint8_t req=PKT_REQUEST; nullnet_buf=&req; nullnet_len=1; NETSTACK_NETWORK.output(&peer);
      start_transfer();
    }
  
  } else if(len==2 && *(uint8_t*)data==PKT_ACK && sending) {
    uint8_t ackseq=((uint8_t*)data)[1];
    if(ackseq==seq_idx){
      seq_idx++;
      if(seq_idx*CHUNK_SIZE>=SAMPLES){
        /* done */
        memset(light_buf,0,sizeof(light_buf));
        memset(motion_buf,0,sizeof(motion_buf));
        sample_idx=0; buffer_full=0; sending=0; seq_idx=0; good_cnt=0; peer_set=0;
        printf("%lu TRANSFER_COMPLETE\n",clock_seconds());
      } else {
        send_chunk(seq_idx);
      }
    }
  }
}

/* ------------ main process ------------ */
PROCESS(node_a_proc,"Node A TX");
AUTOSTART_PROCESSES(&node_a_proc);

PROCESS_THREAD(node_a_proc,ev,data){
  PROCESS_BEGIN();
  init_opt();
  init_mpu();

  nullnet_set_input_callback(rx_cb);

  etimer_set(&sample_timer,SAMPLE_INTERVAL);
  etimer_set(&beacon_timer,BEACON_PERIOD);

  while(1){
    PROCESS_WAIT_EVENT();
    if(etimer_expired(&sample_timer)) {
      light_buf[sample_idx] = get_light();
      motion_buf[sample_idx] = get_motion_scaled();
      sample_idx++;
      if(sample_idx>=SAMPLES){ buffer_full=1; sample_idx=SAMPLES; }
      etimer_reset(&sample_timer);
    }
    if(etimer_expired(&beacon_timer) && !sending){ send_beacon(); etimer_reset(&beacon_timer);}  }
  PROCESS_END();
}

/* ------------ helpers ------------ */
static void send_beacon(void){ 
  uint8_t b=PKT_BEACON; nullnet_buf=&b;
  nullnet_len=1;
  NETSTACK_NETWORK.output(NULL);
  printf("%lu BEACON_SENT\n",clock_seconds());
}

static void start_transfer(void){ 
  sending=1;
  seq_idx=0;
  send_chunk(0);
} 

static void send_chunk(uint8_t seq){
  dpkt.type=PKT_DATA; dpkt.seq=seq;
  uint8_t off=seq*CHUNK_SIZE;
  for(uint8_t i=0;i<CHUNK_SIZE;i++){ dpkt.payload[2*i]=light_buf[off+i]; dpkt.payload[2*i+1]=motion_buf[off+i]; }
  nullnet_buf=(uint8_t*)&dpkt; nullnet_len=2+CHUNK_SIZE*2*2; NETSTACK_NETWORK.output(&peer);
}
