CFLAGS= -g -D__LINUX_ASLA -Wall
LIBS= -lm -lpthread -lasound -lX11

all:
	g++ -o scan super_scanner.cpp scanner_window.cpp audio_engine.cpp  $(CFLAGS) $(LIBS)
