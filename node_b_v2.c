/*
 * node_b_v2.c – Receiver that acknowledges only when motionless
 *
 * – Listens in 100 ms windows (WAKE_TIME) every 100 ms (SLEEP_INTERVAL).
 * – On PKT_REQUEST, returns PKT_REQ_ACK only if |motion| < MOTIONLESS_THRESHOLD.
 * – On PKT_DATA, stores chunk and replies with PKT_ACK.
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
 #define MOTIONLESS_THRESHOLD   1     /* centi‑g */
 #define SAMPLES                60
 #define CHUNK_SIZE             20
 
 #define WAKE_TIME              (RTIMER_SECOND / 10)
 #define SLEEP_INTERVAL         (RTIMER_SECOND / 10)
 
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
 } req_pkt_t;
 
 typedef struct __attribute__((packed)) {
   uint8_t  type;
   uint16_t src_id;
   uint8_t  seq;
 } ack_pkt_t;
 
 typedef struct __attribute__((packed)) {
   uint8_t  type;
   uint16_t src_id;
   uint8_t  seq;
   int16_t  payload[CHUNK_SIZE * 2];
 } data_pkt_t;
 
 /* ------------ storage for one sample set ------------ */
 static int16_t light_buf[SAMPLES];
 static int16_t motion_buf[SAMPLES];
 static uint8_t chunks_rx = 0;          /* bitmask 0b00000111 */
 
 /* ------------ timers ------------ */
 static struct rtimer rt;
 
 /* ------------ helpers ------------ */
 static int16_t read_motion(void)
 {
   int16_t ax = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
   int16_t ay = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
   int16_t az = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
   float g = sqrtf((float)(ax*ax + ay*ay + az*az)) / 16384.0f;
   return (int16_t)(g * 100);          /* centi‑g */
 }
 
 /* ---- duty‑cycle callbacks ---- */
 static void start_listen(struct rtimer *t, void *ptr);
 static void end_listen(struct rtimer *t, void *ptr);
 
 static void end_listen(struct rtimer *t, void *ptr)
 {
   NETSTACK_RADIO.off();
   rtimer_set(&rt, RTIMER_NOW() + SLEEP_INTERVAL, 0, start_listen, NULL);
 }
 
 static void start_listen(struct rtimer *t, void *ptr)
 {
   NETSTACK_RADIO.on();
   rtimer_set(&rt, RTIMER_NOW() + WAKE_TIME, 0, end_listen, NULL);
 }
 
 /* ------------ Nullnet input ------------ */
 static void input_callback(const void *data, uint16_t len,
                            const linkaddr_t *src, const linkaddr_t *dest)
 {
   if(len == 0) return;
   uint8_t type = ((uint8_t *)data)[0];
 
   if(type == PKT_REQUEST && len == sizeof(req_pkt_t)) {
     if(abs(read_motion()) < MOTIONLESS_THRESHOLD) {
       ack_pkt_t ra = { PKT_REQ_ACK, node_id, 0 };
       nullnet_buf = (uint8_t *)&ra;
       nullnet_len = sizeof(ra);
       NETSTACK_NETWORK.output(src);
       printf("TX REQ_ACK (motionless)\n");
     } else {
       printf("Ignore REQ – moving\n");
     }
 
   } else if(type == PKT_DATA && len == sizeof(data_pkt_t)) {
     data_pkt_t pkt;
     memcpy(&pkt, data, len);
     uint8_t seq = pkt.seq;
     printf("RX DATA chunk %u\n", seq);
 
     for(uint8_t i = 0; i < CHUNK_SIZE; i++) {
       light_buf[seq * CHUNK_SIZE + i]  = pkt.payload[2*i];
       motion_buf[seq * CHUNK_SIZE + i] = pkt.payload[2*i + 1];
     }
     chunks_rx |= (1 << seq);
 
     /* send DATA_ACK */
     ack_pkt_t da = { PKT_ACK, node_id, seq };
     nullnet_buf = (uint8_t *)&da;
     nullnet_len = sizeof(da);
     NETSTACK_NETWORK.output(src);
     printf("TX DATA_ACK %u\n", seq);
 
     if(chunks_rx == 0x07) {
       printf("Full set received - 60 samples stored\n");
       chunks_rx = 0;
     }
   }
 }
 
 /* ------------ Contiki process ------------ */
 PROCESS(node_b_process, "Node-B receiver");
 AUTOSTART_PROCESSES(&node_b_process);
 
 PROCESS_THREAD(node_b_process, ev, data)
 {
   PROCESS_BEGIN();
 
   nullnet_set_input_callback(input_callback);
   SENSORS_ACTIVATE(mpu_9250_sensor);
 
   /* start duty‑cycled listening */
   NETSTACK_RADIO.off();
   rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND / 20, 0,
              start_listen, NULL);
 
   while(1) {
     PROCESS_YIELD();   /* nothing else to do */
   }
 
   PROCESS_END();
 }