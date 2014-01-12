#include "ringbuffer.h"

void Ring::clear(struct RingBuffer *ring)
{
	ring->tail = 0;
	ring->head = 0;
}

void Ring::initialize(struct RingBuffer *ring, int length)
{
	ring->buffer = new unsigned char[length];
	ring->bufferSize = length;
	clear(ring);
}

int Ring::size(const struct RingBuffer *ring)
{
	int tail = ring->tail;
	int head = ring->head;

	return (head >= tail) ? head - tail : ring->bufferSize - (tail - head);
}


int Ring::capacity(const struct RingBuffer *ring)
{
	return ring->bufferSize - 1;
}

int Ring::write(struct RingBuffer *ring, unsigned char *data, int length)
{
	int freeSize = capacity(ring) - size(ring);
	if (length > freeSize) {
		fprintf(stderr, "Too many data!\n");
		return -1;
	}

	if (ring->head >= ring->tail) {
		int firstSize = 0;
		int remainSize = 0;
		if (ring->head + length > ring->bufferSize) {
			firstSize = ring->bufferSize - ring->head;
			remainSize = length - firstSize;
		} else {
			firstSize = length;
			remainSize = 0;
		}

		memcpy((ring->buffer + ring->head), data, firstSize);
		ring->head = (ring->head + firstSize) % ring->bufferSize;

		if (remainSize) {
			memcpy(ring->buffer, data + firstSize, remainSize);
			ring->head += remainSize;
		}

	} else {
		memcpy((ring->buffer + ring->head), data, length);
		ring->head += length;
	}
	return length;
}

int Ring::read(struct RingBuffer *ring, unsigned char *buffer, int length)
{
	if (length > size(ring)) {
		fprintf(stderr, "Ask for too many\n");
		return -1;
	}

	if (ring->head >= ring->tail) {
		memcpy(buffer, (ring->buffer) + ring->tail, length);
		ring->tail += length;
	} else {
		int firstSize = 0;
		int remainSize = 0;
		if (ring->tail + length > ring->bufferSize) {
			firstSize = ring->bufferSize - ring->tail;
			remainSize = length - firstSize;
		} else {
			firstSize = length;
			remainSize = 0;
		}

		memcpy(buffer, (ring->buffer + ring->tail), firstSize);
		ring->tail = (ring->tail + firstSize) % ring->bufferSize;

		if (remainSize) {
			memcpy((buffer + firstSize), ring->buffer, remainSize);
			ring->tail += remainSize;
		}
	}
	return length;
}

int Ring::show(struct RingBuffer *ring)
{
	for (int i=0; i<ring->bufferSize; i++) {
		printf("%02d : %02X", i, ring->buffer[i]);
		if (i == ring->head) {
			printf(" <- Head\n");
		} else if (i == ring->tail) {
			printf(" <- Tail\n");
		} else {
			printf("\n");
		}
	}
	return 0;
}

int Ring::find(struct RingBuffer *ring, const unsigned char *target, int length, SearchStart start)
{
	bool found;
	if (start == FROM_TAIL) {
		if (ring->head >= ring->tail) {
			for (int i=ring->tail; i <= (ring->head - length); i++) {
				for (int j=0; j<length; j++) {
					if (ring->buffer[i+j] == target[j]) {
						found = true;
					} else {
						found = false;
						break;
					}
				}
				if (found) {
					return i;
				}
			}
			return -1;

		} else {
			int endIndex = ring->head - length;
			if (endIndex < 0)
				endIndex = ring->bufferSize + endIndex;
			endIndex += 1;

			for (int i=ring->tail; i!=endIndex ;i=(i+1) % ring->bufferSize) {
				for (int j=0; j<length; j++) {
					int ringIndex = (i + j) % ring->bufferSize;
					if (ring->buffer[ringIndex] == target[j]) {
						found = true;
					} else {
						found = false;
						break;
					}
				}
				if (found) {
					return i;
				}
			}
			return -1;
		}

	} else if (start == FROM_HEAD) {
		if (ring->head >= ring->tail) {
			for (int i=(ring->head - length); i >= ring->tail; i--) {
				for (int j=0; j<length; j++) {
					if (ring->buffer[i+j] == target[j]) {
						found = true;
					} else {
						found = false;
						break;
					}
				}
				if (found) {
					return i;
				}
			}
			return -1;

		} else {
			int startIndex = ring->head - length;
			if (startIndex < 0)
				startIndex = ring->bufferSize + startIndex;

			for (int i=startIndex;i!=(ring->tail - 1);i--) {
				if (i<0)
					i = ring->bufferSize + i;
				for (int j=0; j<length; j++) {
					int ringIndex = (i + j) % ring->bufferSize;
					if (ring->buffer[ringIndex] == target[j]) {
						found = true;
					} else {
						found = false;
						break;
					}
				}
				if (found) {
					return i;
				}
			}
			return -1;
		}
	} else {
		return -1;
	}
}

