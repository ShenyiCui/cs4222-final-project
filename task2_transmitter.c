// ===============================
// File: task5_tx.c   (Node A)
// ===============================

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include "lib/random.h"
#include <string.h>
#include <stdio.h>
#include "node-id.h"

/* === User‑supplied sensor APIs === */
extern int   get_light_reading(void);      // lux
extern float get_mpu_reading(void);        // |acc| in g

#define SAMPLES               60
#define SAMPLE_INTERVAL       CLOCK_SECOND      // 1 Hz
#define CHUNK_SIZE            20                // send data in 3 chunks
#define RSSI_GOOD_THRESHOLD   (-70)             // dBm

/* Packet types */
#define PKT_BEACON     0x01   // broadcast from any node
#define PKT_REQUEST    0x02   // Node B → Node A : start transfer
#define PKT_DATA       0x03   // Node A → Node B : sensor chunk
#define PKT_ACK        0x04   // Node B → Node A : ACK each chunk

PROCESS(tx_main_proc, "Sensor TX (Node A)");
AUTOSTART_PROCESSES(&tx_main_proc);

static int light_buf[SAMPLES];
static float motion_buf[SAMPLES];
static uint8_t sample_idx = 0;
static uint8_t sending    = 0;
static uint8_t chunk_idx  = 0;
static linkaddr_t peer_addr;  // learnt from REQUEST

/* Packet on‑the‑wire layout */
typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  seq;        // for DATA / ACK
  uint8_t  offset;     // first sample index inside chunk
  uint8_t  count;      // #samples inside chunk
  int32_t  payload[CHUNK_SIZE*2]; // interleaved light,motion
} packet_t;

static packet_t pkt;

static void send_beacon(void);
static void start_transfer(void);
static void send_next_chunk(void);

static void input_cb(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest)
{
  if(len < 1) return;
  const uint8_t *p = data;
  switch(p[0]) {
    case PKT_REQUEST:
      linkaddr_copy(&peer_addr, src);
      sending=1; chunk_idx=0;
      start_transfer();
      break;
    case PKT_ACK:
      if(sending) {
        chunk_idx++;
        if(chunk_idx*CHUNK_SIZE >= SAMPLES) {
          sample_idx = 0;     // clear buffer after send
          sending = 0;
          printf("%lu TRANSFER %u RSSI: %d\n", clock_seconds(), src->u8[7], (int)packetbuf_attr(PACKETBUF_ATTR_RSSI));
        } else {
          send_next_chunk();
        }
      }
      break;
    default: break;
  }
}

PROCESS_THREAD(tx_main_proc, ev, data)
{
  static struct etimer sample_t;
  static struct etimer beacon_t;
  PROCESS_BEGIN();

  nullnet_set_input_callback(input_cb);
  etimer_set(&sample_t, SAMPLE_INTERVAL);
  etimer_set(&beacon_t, CLOCK_SECOND*2);

  while(1) {
    PROCESS_WAIT_EVENT();

    if(etimer_expired(&sample_t)) {
      light_buf[sample_idx]   = get_light_reading();
      motion_buf[sample_idx]  = (int)(get_mpu_reading()*1000);
      sample_idx = (sample_idx + 1) % SAMPLES;
      etimer_reset(&sample_t);
    }

    if(etimer_expired(&beacon_t) && !sending) {
      send_beacon();
      etimer_reset(&beacon_t);
    }
  }
  PROCESS_END();
}

static void send_beacon(void)
{
  uint8_t b = PKT_BEACON;
  nullnet_buf = &b; nullnet_len = 1; NETSTACK_NETWORK.output(NULL);
}

static void start_transfer(void)
{
  printf("%lu DETECT %u\n", clock_seconds(), peer_addr.u8[7]);
  send_next_chunk();
}

static void send_next_chunk(void)
{
  pkt.type   = PKT_DATA;
  pkt.seq    = chunk_idx;
  pkt.offset = chunk_idx*CHUNK_SIZE;
  pkt.count  = CHUNK_SIZE;
  for(uint8_t i=0;i<CHUNK_SIZE;i++) {
    pkt.payload[2*i]   = light_buf[pkt.offset+i];
    pkt.payload[2*i+1] = motion_buf[pkt.offset+i];
  }
  nullnet_buf = (uint8_t *)&pkt;
  nullnet_len = sizeof(pkt.type)+4+CHUNK_SIZE*2*4;
  NETSTACK_NETWORK.output(&peer_addr);
}