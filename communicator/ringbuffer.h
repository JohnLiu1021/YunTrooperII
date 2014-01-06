#ifndef RINGBUFFER_H
#define RINGBUFFER_H 1

#include <string.h>
#include <stdio.h>

namespace Ring {

struct RingBuffer {
	unsigned char *buffer;
	int head;
	int tail;
	int bufferSize;
};
void clear(struct RingBuffer *ring);
void initialize(struct RingBuffer *ring, int length);
int size(const struct RingBuffer *ring);
int capacity(const struct RingBuffer *ring);
int write(struct RingBuffer *ring, unsigned char *data, int length);
int read(struct RingBuffer *ring, unsigned char *buffer, int length);
int show(struct RingBuffer *ring);

enum SearchStart {
	FROM_HEAD,
	FROM_TAIL
};
int find(struct RingBuffer *ring, const unsigned char *target, int length, SearchStart start = FROM_TAIL);

};

#endif
