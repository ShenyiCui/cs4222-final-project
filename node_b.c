// ===============================================
// Node B  (receiver)  –  node_b.c
// ===============================================

#include "contiki.h"
#include "net/nullnet/nullnet.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include <stdio.h>
#include <string.h>

#define PKT_BEACON  0x01
#define PKT_REQUEST 0x02
#define PKT_DATA    0x03
#define PKT_ACK     0x04

#define SAMPLES     60
#define CHUNK_SIZE  20
#define BEACON_PERIOD (2*CLOCK_SECOND)

static int16_t light_buf[SAMPLES];
static int16_t motion_buf[SAMPLES];
static uint8_t chunks_rx=0;
static linkaddr_t peer;
static uint8_t peer_set=0;
static uint8_t beacon_byte=PKT_BEACON;
static struct etimer beacon_timer;

static void send_beacon(void){ nullnet_buf=&beacon_byte; nullnet_len=1; NETSTACK_NETWORK.output(NULL); }
static void send_ack(const linkaddr_t *dest,uint8_t seq){ uint8_t ack[2]={PKT_ACK,seq}; nullnet_buf=ack; nullnet_len=2; NETSTACK_NETWORK.output(dest);} 

/* input */
static void rx_cb(const void *data,uint16_t len,const linkaddr_t *src,const linkaddr_t *dest){
  if(len<1) return;
  uint8_t type=((uint8_t*)data)[0];
  if(type==PKT_REQUEST){
    linkaddr_copy(&peer,src); peer_set=1; chunks_rx=0;
    printf("%lu DETECT %u RSSI: %d\n",clock_seconds(),src->u8[7],(signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI));
    /* respond by expecting data; nothing else to send */
  } else if(type==PKT_DATA && peer_set && linkaddr_cmp(src,&peer)){
    const data_pkt_t *p=data;
    for(uint8_t i=0;i<CHUNK_SIZE;i++){ uint8_t idx=p->seq*CHUNK_SIZE+i; light_buf[idx]=p->payload[2*i]; motion_buf[idx]=p->payload[2*i+1]; }
    send_ack(src,p->seq);
    chunks_rx++;
    if(chunks_rx*CHUNK_SIZE>=SAMPLES){
      printf("Light:"); for(uint8_t i=0;i<SAMPLES;i++) printf(i?", %d":" %d", light_buf[i]);
      printf("\nMotion:"); for(uint8_t i=0;i<SAMPLES;i++) printf(i?", %d":" %d", motion_buf[i]); printf("\n");
      peer_set=0;
    }
  }
}

PROCESS(node_b_proc,"Node B RX");
AUTOSTART_PROCESSES(&node_b_proc);

PROCESS_THREAD(node_b_proc,ev,data){
  PROCESS_BEGIN();
  nullnet_set_input_callback(rx_cb);
  etimer_set(&beacon_timer,BEACON_PERIOD);
  while(1){ PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&beacon_timer)); send_beacon(); etimer_reset(&beacon_timer);} PROCESS_END(); }
