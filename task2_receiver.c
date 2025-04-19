// ===============================
// File: task5_rx.c   (Node B)
// ===============================

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include <string.h>
#include <stdio.h>

#define SAMPLES 60
#define CHUNK_SIZE 20
#define RSSI_GOOD_THRESHOLD (-70)

#define PKT_BEACON   0x01
#define PKT_REQUEST  0x02
#define PKT_DATA     0x03
#define PKT_ACK      0x04

PROCESS(rx_main_proc, "Sensor RX (Node B)");
AUTOSTART_PROCESSES(&rx_main_proc);

static int light_buf[SAMPLES];
static int motion_buf[SAMPLES];
static uint8_t got_chunks = 0;
static linkaddr_t tx_addr;

typedef struct __attribute__((packed)){
  uint8_t  type;
  uint8_t  seq;
  uint8_t  offset;
  uint8_t  count;
  int32_t  payload[CHUNK_SIZE*2];
} packet_t;

static void send_request(const linkaddr_t *dest);
static void send_ack(const linkaddr_t *dest,uint8_t seq);

static void input_cb(const void *data,uint16_t len,const linkaddr_t *src,const linkaddr_t *dest)
{
  if(len<1) return;
  const uint8_t *p = data;
  if(p[0]==PKT_BEACON) {
    int rssi=(int)packetbuf_attr(PACKETBUF_ATTR_RSSI);
    if(rssi>RSSI_GOOD_THRESHOLD) {
      printf("%lu DETECT %u\n",clock_seconds(),src->u8[7]);
      linkaddr_copy(&tx_addr,src);
      send_request(src);
    }
  } else if(p[0]==PKT_DATA && linkaddr_cmp(src,&tx_addr)) {
    const packet_t *pkt=data;
    for(uint8_t i=0;i<pkt->count;i++) {
      uint8_t idx=pkt->offset+i;
      light_buf[idx]  = pkt->payload[2*i];
      motion_buf[idx] = pkt->payload[2*i+1];
    }
    send_ack(src,pkt->seq);
    got_chunks++;
    if(got_chunks*CHUNK_SIZE>=SAMPLES) {
      printf("Light:");
      for(uint8_t i=0;i<SAMPLES;i++) printf(i?", %d":" %d", light_buf[i]);
      printf("\nMotion:");
      for(uint8_t i=0;i<SAMPLES;i++) printf(i?", %d":" %d", motion_buf[i]);
      printf("\n");
      got_chunks=0;
    }
  }
}

PROCESS_THREAD(rx_main_proc,ev,data)
{
  PROCESS_BEGIN();
  nullnet_set_input_callback(input_cb);
  PROCESS_END();
}

static void send_request(const linkaddr_t *dest)
{
  uint8_t t=PKT_REQUEST;
  nullnet_buf=&t; nullnet_len=1; NETSTACK_NETWORK.output(dest);
  printf("%lu TRANSFER %u RSSI: %d\n", clock_seconds(), dest->u8[7], (int)packetbuf_attr(PACKETBUF_ATTR_RSSI));
}

static void send_ack(const linkaddr_t *dest,uint8_t seq)
{
  uint8_t pkt[2]={PKT_ACK,seq};
  nullnet_buf=pkt; nullnet_len=2; NETSTACK_NETWORK.output(dest);
}
