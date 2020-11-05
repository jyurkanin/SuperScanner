CFLAGS= -g -D__LINUX_ASLA -Wall -O3
LIBS= -lm -lpthread -lasound -lX11

all: main.cpp super_scanner.o controller.o wavfile.o scanner_window.o audio_engine.o
	g++ -o scan main.cpp super_scanner.o controller.o wavfile.o scanner_window.o audio_engine.o  $(CFLAGS) $(LIBS)

super_scanner.o: super_scanner.cpp
	g++ -c super_scanner.cpp  -O3 $(CFLAGS) $(LIBS)

controller.o: controller.cpp
	g++ -c controller.cpp  $(CFLAGS) $(LIBS)

wavfile.o: wavfile.cpp
	g++ -c wavfile.cpp  $(CFLAGS) $(LIBS)

scanner_window.o: scanner_window.cpp
	g++ -c scanner_window.cpp  $(CFLAGS) $(LIBS)

audio_engine.o: audio_engine.cpp
	g++ -c audio_engine.cpp  $(CFLAGS) $(LIBS)	
