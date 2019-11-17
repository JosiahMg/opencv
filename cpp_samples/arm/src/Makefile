#ROOT=$(shell pwd)
ROOT=.
CFLAGS=-Wall -g -std=c++11 `pkg-config --cflags --libs opencv realsense2`
CC=g++
TARGET=vision

SRCS=$(shell ls $(ROOT)/*.cpp)

OBJS = $(SRCS:.cpp=.o)

$(TARGET):$(OBJS)
	@$(CC) -o $@ $^ $(CFLAGS) 


%.o:%.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	@rm -rf $(TARGET) $(OBJS)
