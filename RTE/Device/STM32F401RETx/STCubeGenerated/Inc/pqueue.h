#ifndef PQUEUE_H
#define PQUEUE_H

#include <stdbool.h>
#include <stddef.h>

struct Complex {
	float real;
	float imag;
};
struct priority_queue {
    float *heap;
    size_t length;
    size_t heap_size;
};
// Constructor: Initializes the priority queue with values existing
void priority_queue_from_array(struct priority_queue *pq, float input[], const size_t size);

// Build max-heap from the current array
void priority_queue_build_max_heap(struct priority_queue *pq);

// Insert an integer into the heap
void priority_queue_add(struct priority_queue *pq, float value);

// Pop the top value from the heap (returns true if successful, false if empty)
bool priority_queue_pop(struct priority_queue *pq, float *value);

// Check if the heap is empty
bool priority_queue_is_empty(const struct priority_queue *pq);

// Print the heap for debugging purposes
void priority_queue_print_heap(const struct priority_queue *pq);

#endif //PQUEUE_H