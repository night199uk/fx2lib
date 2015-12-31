FX2LIBDIR=../../../
BASENAME = debugdev_slave
SOURCES=debugdev_slave.c
A51_SOURCES=dscr.a51
PID=0x1004
CXX=g++
HOSTCC=gcc
CXXFLAGS=-Wall -Wextra -O2 -ggdb
HOSTCFLAGS=-Wall -Wextra -O2 -ggdb

all: build/debugdev_slave.ihx
include $(FX2LIBDIR)lib/fx2.mk

download: build/$(BASENAME).ihx
	  ./download.sh $<

%:		%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lusb-1.0
