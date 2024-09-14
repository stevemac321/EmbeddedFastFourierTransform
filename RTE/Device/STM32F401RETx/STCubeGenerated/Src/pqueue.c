#include "pqueue.h"

// Internal helper: Returns the index of the parent of the element at index i
static size_t parent(size_t i) {
    return (i - 1) / 2;
}

// Internal helper: Returns the index of the left child of the element at index i
static size_t left(size_t i) {
    return 2 * i + 1;
}

// Internal helper: Returns the index of the right child of the element at index i
static size_t right(size_t i) {
    return 2 * i + 2;
}

// Swap two elements in the heap
static void swap(float *a, float *b) {
    float tmp = *a;
    *a = *b;
    *b = tmp;
}

// Maintains the max-heap property starting from index i
static void max_heapify(struct priority_queue *pq, size_t i) {
    size_t l = left(i);
    size_t r = right(i);
    size_t largest = i;

    if (l < pq->heap_size && pq->heap[l] > pq->heap[i]) {
        largest = l;
    }
    if (r < pq->heap_size && pq->heap[r] > pq->heap[largest]) {
        largest = r;
    }
    if (largest != i) {
        swap(&pq->heap[i], &pq->heap[largest]);
        max_heapify(pq, largest);
    }
}

// Initializes the priority queue with values from an array
void priority_queue_from_array(struct priority_queue *pq, float input[], const size_t size) {
    pq->length = size;
    pq->heap_size = size;
	#if 0
		for(size_t i = 0; i < size; i++) {
			pq->heap[i] = input[i];
		}
	#endif
}

// Build max-heap from the current array
void priority_queue_build_max_heap(struct priority_queue *pq) {
    for (size_t i = pq->heap_size / 2; i > 0; i--) {
        max_heapify(pq, i - 1);
    }
}

// Insert an integer into the heap
void priority_queue_add(struct priority_queue *pq, float value) {
    if (pq->heap_size == pq->length) {
        return;  // Heap is full
    }
    pq->heap[pq->heap_size] = value;
    pq->heap_size++;
    priority_queue_build_max_heap(pq);
}

// Pop the top value from the heap
bool priority_queue_pop(struct priority_queue *pq, float *value) {
    if (pq->heap_size == 0) {
        return false;  // No elements left to pop
    }

    *value = pq->heap[0];
    pq->heap[0] = pq->heap[pq->heap_size - 1];
    pq->heap_size--;
    max_heapify(pq, 0);

    return true;
}

// Check if the heap is empty
bool priority_queue_is_empty(const struct priority_queue *pq) {
    return pq->heap_size == 0;
}

// Print the heap for debugging purposes
void priority_queue_print_heap(const struct priority_queue *pq) {
    for (size_t i = 0; i < pq->heap_size; i++) {
        //printf("Index %zu: %u\n", i, pq->heap[i]);
    }
}
