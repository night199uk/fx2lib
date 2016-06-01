FX2LIBDIR=../../
PID=0x1004
CXX=g++
HOSTCC=gcc
CXXFLAGS=-Wall -Wextra -O2 -ggdb
HOSTCFLAGS=-Wall -Wextra -O2 -ggdb
SUBDIRS=master slave

all: test subdirs

subdirs: $(SUBDIRS)

.PHONY: all subdirs $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

terminal:	terminal.c
	$(HOSTCC) $(HOSTCFLAGS) -o $@ $< -lusb-1.0

%:		%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lusb-1.0
