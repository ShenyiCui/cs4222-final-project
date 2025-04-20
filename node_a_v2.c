/*
 * node_a_v2.c – Motion‑triggered data logger and uploader
 *
 * Behaviour
 * ----------
 * – IDLE: only MPU‑9250 active for motion sensing.
 * – On |motion| ≥ MOTION_THRESHOLD, switch to COLLECTING.
 * – COLLECTING: sample light + motion at 1 Hz for 60 s (SAMPLES = 60).
 * – Store each 60‑second set in a circular buffer that holds MAX_SETS = 5.
 * – When buffer not empty, enter SENDING state:
 *      1. Transmit PKT_REQUEST every duty‑cycle until three consecutive
 *         PKT_REQ_ACK frames have RSSI ≥ RSSI_GOOD_THRESHOLD.
 *      2. Send three PKT_DATA chunks (20 readings each) with ACKs.
 * – After all chunks ACKed, dequeue the set and repeat if more data.
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <stdint.h>
 #include <math.h>
 #include <string.h>
 #include "contiki.h"
 #include "sys/rtimer.h"
 #include "net/nullnet/nullnet.h"
 #include "net/netstack.h"
 #include "net/packetbuf.h"
 #include "node-id.h"
 #include "board-peripherals.h"
 
 /* ------------ parameters ------------ */
 #define MOTION_THRESHOLD        1           /* centi‑g */
 #define SAMPLES                 60          /* 60 s window           */
 #define CHUNK_SIZE              20          /* 3 chunks per set       */
 #define MAX_SETS                5           /* buffer capacity        */
 
 #define SAMPLE_INTERVAL         CLOCK_SECOND
 #define SEND_CHUNK_INTERVAL     (RTIMER_SECOND / 4)
 
 #define WAKE_TIME               (RTIMER_SECOND / 10)  /* 100 ms listen  */
 #define SLEEP_SLOT              (RTIMER_SECOND / 10)  /* 100 ms sleep   */
 
 #define RSSI_GOOD_THRESHOLD    (-70)        /* three ≥ threshold → good link */
 
 /* ------------ packet types ------------ */
 #define PKT_BEACON   0x01
 #define PKT_REQUEST  0x02
 #define PKT_DATA     0x03
 #define PKT_ACK      0x04
 #define PKT_REQ_ACK  0x05
 
 /* ------------ packet formats ------------ */
 typedef struct __attribute__((packed)) {
   uint8_t  type;
   uint16_t src_id;
 } req_pkt_t;               /* also beacon */
 
 typedef struct __attribute__((packed)) {
   uint8_t  type;
   uint16_t src_id;
   uint8_t  seq;
 } ack_pkt_t;               /* REQ_ACK or DATA_ACK */
 
 typedef struct __attribute__((packed)) {
   uint8_t  type;
   uint16_t src_id;
   uint8_t  seq;
   int16_t  payload[CHUNK_SIZE * 2];   /* light, motion interleaved */
 } data_pkt_t;
 
 /* ------------ sample‑set circular buffer ------------ */
 typedef struct {
   int16_t light[SAMPLES];
   int16_t motion[SAMPLES];
 } sample_set_t;
 
 static sample_set_t buffer[MAX_SETS];
 static uint8_t buf_head = 0;    /* dequeue index      */
 static uint8_t buf_tail = 0;    /* enqueue index      */
 static uint8_t buf_len  = 0;    /* how many sets      */
 
 static inline uint8_t buf_empty(void){ return buf_len == 0; }
 static inline uint8_t buf_full (void){ return buf_len == MAX_SETS; }
 
 /* ------------ runtime state ------------ */
 static enum { ST_IDLE = 0, ST_COLLECTING, ST_SENDING } state = ST_IDLE;
 static uint8_t  sample_idx   = 0;   /* 0‑59 within current set */
 static uint8_t  tx_seq       = 0;   /* 0,1,2 chunk counter     */
 static uint8_t  awaiting_ack = 0;
 static uint8_t  good_cnt     = 0;
 
 static struct etimer sample_timer;
 static struct rtimer rt;
 
 /* peer (Node B) link‑layer address – adjust if needed */
 static linkaddr_t peer = { .u8 = { 0x02, 0x00 } };
 
 /* ------------ helpers ------------ */
 static int16_t read_motion(void){
   int16_t ax = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
   int16_t ay = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
   int16_t az = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
   float g = sqrtf((float)(ax*ax + ay*ay + az*az)) / 16384.0f;
   return (int16_t)(g * 100);        /* centi‑g */
 }
 
 /* forward declarations of rtimer callbacks */
 static void rt_send_req(struct rtimer *t, void *ptr);
 static void rt_listen_end(struct rtimer *t, void *ptr);
 static void rt_send_chunk(struct rtimer *t, void *ptr);
 
 /* ------------ Nullnet input ------------ */
 static void input_callback(const void *data, uint16_t len,
                            const linkaddr_t *src, const linkaddr_t *dest)
 {
   if(len == 0) return;
   uint8_t type = ((uint8_t *)data)[0];
 
   if(type == PKT_REQ_ACK) {
     /* handshake ACK */
     int16_t rssi = (int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI);
     if(rssi >= RSSI_GOOD_THRESHOLD) good_cnt++; else good_cnt = 0;
     awaiting_ack = 0;
 
     if(good_cnt >= 3) {
       /* link good – start first data chunk */
       tx_seq = 0;
       rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND / 20, 0,
                  rt_send_chunk, NULL);
     }
 
   } else if(type == PKT_ACK) {
     /* data chunk ack */
     awaiting_ack = 0;
     tx_seq++;
 
     if(tx_seq < 3) {
       /* send next chunk */
       rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND / 20, 0,
                  rt_send_chunk, NULL);
     } else {
       /* set delivered */
       buf_head = (buf_head + 1) % MAX_SETS;
       buf_len--;
       printf("%lu Upload complete – buffer=%u\n", clock_seconds(), buf_len);
 
       /* more waiting? */
       if(!buf_empty()) {
         good_cnt = 0;
         rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND / 5, 0,
                    rt_send_req, NULL);
       } else {
         state = ST_IDLE;
       }
     }
   }
 }
 
 /* ------------ rtimer: send PKT_REQUEST ------------ */
 static void rt_send_req(struct rtimer *t, void *ptr)
 {
   if(buf_empty()) { state = ST_IDLE; return; }
 
   req_pkt_t req = { PKT_REQUEST, node_id };
   nullnet_buf = (uint8_t *)&req;
   nullnet_len = sizeof(req);
   NETSTACK_RADIO.on();
   NETSTACK_NETWORK.output(&peer);
   awaiting_ack = 1;
 
   /* stay awake WAKE_TIME to wait for ACK */
   rtimer_set(&rt, RTIMER_NOW() + WAKE_TIME, 0, rt_listen_end, NULL);
 }
 
 /* ------------ rtimer: radio off / retry if no ACK ------------ */
 static void rt_listen_end(struct rtimer *t, void *ptr)
 {
   NETSTACK_RADIO.off();
   if(awaiting_ack) {
     /* no ACK: sleep a slot then resend request */
     rtimer_set(&rt, RTIMER_NOW() + SLEEP_SLOT, 0, rt_send_req, NULL);
   }
 }
 
 /* ------------ rtimer: send data chunk ------------ */
 static void rt_send_chunk(struct rtimer *t, void *ptr)
 {
   data_pkt_t pkt;
   pkt.type   = PKT_DATA;
   pkt.src_id = node_id;
   pkt.seq    = tx_seq;
 
   const sample_set_t *set = &buffer[buf_head];
   for(uint8_t i = 0; i < CHUNK_SIZE; i++) {
     pkt.payload[2*i]     = set->light[tx_seq * CHUNK_SIZE + i];
     pkt.payload[2*i + 1] = set->motion[tx_seq * CHUNK_SIZE + i];
   }
 
   nullnet_buf = (uint8_t *)&pkt;
   nullnet_len = sizeof(pkt);
   NETSTACK_RADIO.on();
   NETSTACK_NETWORK.output(&peer);
   awaiting_ack = 1;
 
   rtimer_set(&rt, RTIMER_NOW() + WAKE_TIME, 0, rt_listen_end, NULL);
 }
 
 /* ------------ Contiki process ------------ */
 PROCESS(node_a_process, "Node-A motion logger");
 AUTOSTART_PROCESSES(&node_a_process);
 
 PROCESS_THREAD(node_a_process, ev, data)
 {
   PROCESS_BEGIN();
 
   nullnet_set_input_callback(input_callback);
   SENSORS_ACTIVATE(mpu_9250_sensor);
   SENSORS_ACTIVATE(opt_3001_sensor);
 
   etimer_set(&sample_timer, SAMPLE_INTERVAL);
 
   while(1) {
     PROCESS_WAIT_EVENT();
 
     if(ev == PROCESS_EVENT_TIMER && data == &sample_timer) {
 
       int16_t motion = read_motion();
 
       if(state == ST_IDLE) {
         if(abs(motion) >= MOTION_THRESHOLD && !buf_full()) {
           printf("%lu Motion detected - start collecting\n", clock_seconds());
           sample_idx = 0;
           state = ST_COLLECTING;
         }
 
       } else if(state == ST_COLLECTING) {
         /* collect light + motion */
         int16_t light = opt_3001_sensor.value(0);
         buffer[buf_tail].light[sample_idx]  = light;
         buffer[buf_tail].motion[sample_idx] = motion;
         sample_idx++;
 
         if(sample_idx >= SAMPLES) {
           /* complete set */
           buf_tail = (buf_tail + 1) % MAX_SETS;
           buf_len++;
           printf("%lu Set collected - buffer=%u\n", clock_seconds(), buf_len);
           state = ST_IDLE;
 
           /* trigger upload if we are not already sending */
           if(!buf_empty() && state != ST_SENDING) {
             state = ST_SENDING;
             rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND / 5, 0,
                        rt_send_req, NULL);
           }
         }
       }
 
       etimer_reset(&sample_timer);
     }
   }
 
   PROCESS_END();
 }