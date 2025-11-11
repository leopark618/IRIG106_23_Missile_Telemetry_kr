CC = gcc
CFLAGS = -Wall -O2 -lm
TARGET = missile_telemetry

SOURCES = src/1_sensor_acquisition.c \
          src/2_ldpc_encoder.c \
          src/3_ldpc_decoder.c \
          src/4_ldpc_randomizer.c \
          src/5_soqpsk_modulator.c \
          src/6_soqpsk_demodulator.c \
          src/7_main_integration.c

OBJECTS = $(SOURCES:.c=.o)
CFLAGS += -Iinclude

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) -o $@ $^ $(CFLAGS)

%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

clean:
	rm -f $(OBJECTS) $(TARGET)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run
