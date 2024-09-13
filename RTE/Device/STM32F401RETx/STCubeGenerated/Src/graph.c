#include "graph.h"
// graph operations


void create_graph(struct Graph *g, const int sz) {
    g->numVertices = 0;
    g->graph_capacity=sz;

}
// Internal function to add a vertex
void add_vertex(struct Graph* g, const float mag) {
    // Ensure we don't exceed the array size
    if (g->numVertices >= g->graph_capacity) {
        // Handle full graph (e.g., return or log error)
        return;
    }
		 g->vertices[g->numVertices].magnitude = mag;
    // Initialize the adjacency list to 0 (no connections initially)
    g->vertices[g->numVertices].adjList = 0;

    // Increment the number of vertices after initialization
    g->numVertices++;
}

// Internal function to add an edge between two vertices
void add_edge(struct Graph* g, size_t v, size_t w) {
    if (v >= g->numVertices || w >= g->numVertices) return;


    // Set the edge bit in the adjacency list
    g->vertices[v].adjList  |= (1 << (w % g->graph_capacity));
}
//
void bfs(struct Graph* g, int start_vertex) {
    // Initialize all vertices
    for (int i = 0; i < g->numVertices; i++) {
        g->vertices[i].color =  WHITE;  // Mark all vertices as unvisited (white)
        g->vertices[i].distance = -1;  // Initialize distances to -1 (undefined)
        g->vertices[i].parent = -1;  // No parent yet
    }

    // Set up the start vertex
    g->vertices[start_vertex].color =  GRAY;  // Mark the start vertex as discovered
    g->vertices[start_vertex].distance = 0;  // Distance from the start vertex to itself is 0
    g->vertices[start_vertex].parent = -1;  // Start vertex has no parent

    // Initialize a queue
    int queue[g->graph_capacity];  // Simple array-based queue (you can adjust this size if needed)
		int front = 0, rear = 0;  // Queue indices
    queue[rear++] = start_vertex;  // Enqueue the start vertex

    // BFS loop
    while (front != rear) {  // While queue is not empty
        int u = queue[front++];  // Dequeue vertex `u`

        // Explore all adjacent vertices of `u`
        for (int v = 0; v < g->numVertices; v++) {
            if (g->vertices[u].adjList & (1 << v)) {  // Check if there is an edge (u, v)
                if (g->vertices[v].color == WHITE) {  // If vertex `v` is unvisited
                    g->vertices[v].color =GRAY;  // Mark vertex `v` as discovered
                    g->vertices[v].distance = g->vertices[u].distance + 1;  // Set distance
                    g->vertices[v].parent = u;  // Set parent
                    queue[rear++] = v;  // Enqueue vertex `v`
                }
            }
        }

        g->vertices[u].color = BLACK;  // Mark vertex `u` as fully explored
    }
}

