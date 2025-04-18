CONTIKI_PROJECT = nbr node_a_santosh node_b_shenyi
all: $(CONTIKI_PROJECT)

CONTIKI = ../..


MAKE_NET = MAKE_NET_NULLNET
include $(CONTIKI)/Makefile.include
