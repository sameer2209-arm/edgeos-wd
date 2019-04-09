
LDFLAGS ?= -lpthread -pthread 
# TWSOVERSION is the compiler version...
# see http://rute.2038bug.com/node26.html.gz

CXX ?= g++ -g -O0 -fPIC -std=c++11
CC ?= gcc -g -O0 -fPIC
AR ?= ar

LIBUVDIR= libuv-v1.10.1

CXXFLAGS= -fPIC -std=c++11

ARCH ?=x86 
#ARCH=armel	
SYSCALLS= syscalls-$(ARCH).c

ALLOBJS= $($<:%.cpp=%.o)

DEBUG_OPTIONS=-rdynamic -D_TW_TASK_DEBUG_THREADS_ -DLOGGER_HEAVY_DEBUG
#-D_TW_BUFBLK_DEBUG_STACK_
CROSS_INCLUDE ?= /usr/include
CROSS_LDFLAGS ?= -L/usr/lib
CFLAGS=  $(GLIBCFLAG) -I./include -fPIC -I./deps/$(LIBUVDIR)/include -I./deps/build/include  -L./deps/build/lib

DEBUG_CFLAGS= -g -DERRCMN_DEBUG_BUILD -D__DEBUG  -D_TW_DEBUG $(DEBUG_OPTIONS)

ROOT_DIR=.
OUTPUT_DIR=.

EXTRA_TARGET=

CFLAGS+= -fPIC

GLIBCFLAG=-D_USING_GLIBC_
LD_TEST_FLAGS= -lgtest

## concerning the -whole-archive flags: http://stackoverflow.com/questions/14889941/link-a-static-library-to-a-shared-one-during-build
## originally we used that when creating the node module version of greaseLogger - but apparently needed for tcmalloc here also
LDFLAGS += -L./deps/build/lib
#  -lTW -luv -ldl

# -Wl,-whole-archive deps/build/lib/libtcmalloc_minimal.a -Wl,-no-whole-archive 
STATIC_LIB_FLAGS= -static deps/build/lib/libuv.a deps/build/lib/libTW.a

HRDS= include/TW/tw_bufblk.h  include/TW/tw_globals.h  include/TW/tw_object.h include/TW/tw_stack.h\
include/TW/tw_dlist.h   include/TW/tw_llist.h    include/TW/tw_socktask.h    include/TW/tw_syscalls.h\
include/TW/tw_macros.h include/TW/tw_globals.h include/TW/tw_alloc.h include/TW/tw_sparsehash.h include/TW/tw_densehash.h\
include/TW/tw_stringmap.h

#SRCS_CPP= error-common.cc logger.cc standalone_test_logsink.cc grease_lib.cc

SRCS_C= main.c
OBJS= $(SRCS_CPP:%.cc=$(OUTPUT_DIR)/%.o) $(SRCS_C:%.c=$(OUTPUT_DIR)/%.o)
OBJS_NAMES= $(SRCS_CPP:%.cc=$%.o) $(SRCS_C:%.c=%.o)

##tw_sparsehash.h

## The -fPIC option tells gcc to create position 
## independant code which is necessary for shared libraries. Note also, 
## that the object file created for the static library will be 
## overwritten. That's not bad, however, because we have a static 
## library that already contains the needed object file.

$(OUTPUT_DIR)/%.o: %.cc
	$(CXX) $(CXXFLAGS) $(CFLAGS) -c $< -o $@

$(OUTPUT_DIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@


deviceOSWD-dummy-cross-debug: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include  $(DEBUG_CFLAGS) -I$(CROSS_INCLUDE)
deviceOSWD-dummy-cross-debug: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/dummy-watchdog.o $(OUTPUT_DIR)/led_control/dummy-ledcontrol.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) $(CROSS_LDFLAGS) -lpthread -o deviceOSWD

deviceOSWD-a10-cross-debug: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include  $(DEBUG_CFLAGS) -I$(CROSS_INCLUDE)
deviceOSWD-a10-cross-debug: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/a10-watchdog.o  $(OUTPUT_DIR)/led_control/wwrelay-ledcontrol.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) $(CROSS_LDFLAGS) -lpthread -o deviceOSWD



deviceOSWD-dummy-debug: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include  $(DEBUG_CFLAGS)
deviceOSWD-dummy-debug: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/dummy-watchdog.o $(OUTPUT_DIR)/led_control/dummy-ledcontrol.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) -lpthread -o deviceOSWD

deviceOSWD-dummy: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include
deviceOSWD-dummy: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/dummy-watchdog.o $(OUTPUT_DIR)/led_control/dummy-ledcontrol.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) -lpthread -o deviceOSWD

deviceOSWD-a10: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include
deviceOSWD-a10: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/a10-watchdog.o $(OUTPUT_DIR)/led_control/dummy-ledcontrol.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) -lpthread -o deviceOSWD

deviceOSWD-a10-debug: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include  $(DEBUG_CFLAGS)
deviceOSWD-a10-debug: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/a10-watchdog.o $(OUTPUT_DIR)/led_control/dummy-ledcontrol.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) -lpthread -o deviceOSWD


deviceOSWD-a10-tiny85: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include
deviceOSWD-a10-tiny85: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/a10-watchdog-tiny85.o $(OUTPUT_DIR)/led_control/dummy-ledcontrol.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) -lpthread -o deviceOSWD

deviceOSWD-a10-tiny85-debug: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include  $(DEBUG_CFLAGS)
deviceOSWD-a10-tiny85-debug: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/a10-watchdog-tiny85.o $(OUTPUT_DIR)/led_control/dummy-ledcontrol.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) -lpthread -o deviceOSWD



deviceOSWD-a10-relay: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include
deviceOSWD-a10-relay: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/a10-watchdog.o $(OUTPUT_DIR)/led_control/wwrelay-ledcontrol.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) -lpthread -o deviceOSWD


deviceOSWD-a10-tiny841: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include
deviceOSWD-a10-tiny841: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/a10-watchdog.o $(OUTPUT_DIR)/led_control/tiny841-ledcontrol.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) -lpthread -o deviceOSWD

deviceOSWD-a10-tiny841-onrelay: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include -I/usr/lib -L/usr/lib $(DEBUG_CFLAGS)
deviceOSWD-a10-tiny841-onrelay: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/a10-watchdog.o $(OUTPUT_DIR)/led_control/tiny841-ledcontrol.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ deps/build/lib/libuv.a deps/build/lib/libTW.a -lpthread -o deviceOSWD


deviceOSWD-rpi-3bplus: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include
deviceOSWD-rpi-3bplus: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/watchdogpi.o $(OUTPUT_DIR)/led_control/LedControlPi.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) -lpthread -o deviceOSWD

deviceOSWD-rpi-3bplus-debug: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include  $(DEBUG_CFLAGS)
deviceOSWD-rpi-3bplus-debug: $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/watchdogpi.o $(OUTPUT_DIR)/led_control/LedControlPi.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) -lpthread -o deviceOSWD

deviceOSWD-rpi-3bplus-test: CFLAGS+= -DGREASE_LIB -I./deps/$(LIBUVDIR)/include -I./deps/twlib/include  $(DEBUG_CFLAGS)
deviceOSWD-rpi-3bplus-test: $(OUTPUT_DIR)/local_strdup.o $(OUTPUT_DIR)/error-common.o $(OUTPUT_DIR)/watchdogpi.o $(OUTPUT_DIR)/led_control/LedControlPi.o $(OUTPUT_DIR)/test/testRPi/testwatchdog.o
	$(CXX) $(CXXFLAGS) $(CFLAGS) $^ $(STATIC_LIB_FLAGS) -lpthread -o testdeviceOSWD

#install: tw_lib $(EXTRA_TARGET)
#	./install-sh $(TWSOVERSION) $(INSTALLPREFIX)
#	ln -sf $(INSTALLPREFIX)/lib/$(TWSONAME) $(INSTALLPREFIX)/lib/$(TWSOVERSION) && \
#	ln -sf $(INSTALLPREFIX)/lib/$(TWSONAME) $(INSTALLPREFIX)/lib/$(TWSOLIBNAME)


clean: 
	-rm -rf $(OUTPUT_DIR)/*.o $(OUTPUT_DIR)/led_control/*.o $(OUTPUT_DIR)/*.obj $(OUTPUT_DIR)/*.rpo $(OUTPUT_DIR)/*.idb $(OUTPUT_DIR)/*.lib $(OUTPUT_DIR)/*.exe $(OUTPUT_DIR)/*.a $(OUTPUT_DIR)/*~ $(OUTPUT_DIR)/core
	-rm -rf Debug
	-rm -f $(TWSOLIBNAME) $(TWSONAME) $(TWSOVERSION)
	-rm -f deviceOSWD
# DO NOT DELETE
