<<<<<<< HEAD
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <stdbool.h>

#define MAX_VERTICES 100
#define INF INT_MAX

// ==================== DATA STRUCTURES ====================

// Adjacency List Node - represents an edge in the graph
typedef struct AdjListNode {
    int dest;                      // Destination vertex of the edge
    int weight;                    // Weight/cost of the edge
    struct AdjListNode* next;      // Pointer to next node in the adjacency list
} AdjListNode;

// Adjacency List - head pointer for each vertex's adjacency list
typedef struct AdjList {
    AdjListNode* head;             // Points to the first adjacent vertex
} AdjList;

// Graph using Adjacency List representation
// Space complexity: O(V + E) where V = vertices, E = edges
typedef struct GraphList {
    int vertices;                  // Number of vertices in the graph
    AdjList* array;               // Array of adjacency lists, one for each vertex
} GraphList;

// Edge structure for edge list representation
// Used in algorithms like Kruskal's that work directly with edges
typedef struct Edge {
    int src, dest, weight;         // Source vertex, destination vertex, and edge weight
} Edge;

// Graph using Edge List representation
// Space complexity: O(E) - most memory efficient for sparse graphs
typedef struct GraphEdgeList {
    int vertices;                  // Number of vertices in the graph
    int edges;                    // Number of edges in the graph
    Edge* edgeArray;              // Array containing all edges
} GraphEdgeList;

// Union-Find (Disjoint Set) structure for Kruskal's algorithm
// Used to efficiently detect cycles and manage connected components
typedef struct UnionFind {
    int* parent;                  // Parent array for union-find structure
    int* rank;                    // Rank array for union by rank optimization
} UnionFind;

// Min-Heap Node for optimization in Dijkstra's and Prim's algorithms
// Stores vertex and its current distance/key value
typedef struct MinHeapNode {
    int vertex;                   // Vertex number
    int distance;                 // Current shortest distance (Dijkstra) or key value (Prim)
} MinHeapNode;

// Min-Heap structure for efficient minimum extraction
// Used to optimize Dijkstra's and Prim's algorithms from O(V²) to O((V+E)logV)
typedef struct MinHeap {
    int size;                     // Current number of elements in heap
    int capacity;                 // Maximum capacity of heap
    int* pos;                     // Array to store positions of vertices in heap (for decrease key)
    MinHeapNode** array;          // Array of pointers to heap nodes
} MinHeap;

// ==================== UTILITY FUNCTIONS ====================

// Create a new adjacency list node
// Used when adding edges to adjacency list representation
AdjListNode* newAdjListNode(int dest, int weight) {
    AdjListNode* newNode = (AdjListNode*)malloc(sizeof(AdjListNode));
    newNode->dest = dest;          // Set destination vertex
    newNode->weight = weight;      // Set edge weight
    newNode->next = NULL;          // Initialize next pointer
    return newNode;
}

// Create a graph with adjacency list representation
// Time complexity: O(V), Space complexity: O(V)
GraphList* createGraphList(int vertices) {
    GraphList* graph = (GraphList*)malloc(sizeof(GraphList));
    graph->vertices = vertices;
    // Allocate memory for array of adjacency lists
    graph->array = (AdjList*)malloc(vertices * sizeof(AdjList));
    
    // Initialize each adjacency list as empty
    for (int i = 0; i < vertices; i++)
        graph->array[i].head = NULL;
    
    return graph;
}

// Add edge to adjacency list graph (creates undirected edge)
// Time complexity: O(1), adds edge in both directions for undirected graph
void addEdgeList(GraphList* graph, int src, int dest, int weight) {
    // Add edge from src to dest
    AdjListNode* newNode = newAdjListNode(dest, weight);
    newNode->next = graph->array[src].head;    // Insert at beginning of list
    graph->array[src].head = newNode;
    
    // For undirected graph, add reverse edge from dest to src
    newNode = newAdjListNode(src, weight);
    newNode->next = graph->array[dest].head;   // Insert at beginning of list
    graph->array[dest].head = newNode;
}

// Create graph with edge list representation
// Most efficient for algorithms that process edges directly (like Kruskal's)
GraphEdgeList* createGraphEdgeList(int vertices, int edges) {
    GraphEdgeList* graph = (GraphEdgeList*)malloc(sizeof(GraphEdgeList));
    graph->vertices = vertices;
    graph->edges = edges;
    // Allocate memory for array of edges
    graph->edgeArray = (Edge*)malloc(edges * sizeof(Edge));
    return graph;
}

// Min-Heap functions for optimizing Dijkstra's and Prim's algorithms

// Create a min-heap with given capacity
MinHeap* createMinHeap(int capacity) {
    MinHeap* minHeap = (MinHeap*)malloc(sizeof(MinHeap));
    minHeap->pos = (int*)malloc(capacity * sizeof(int));       // Position array for decrease key
    minHeap->size = 0;                                         // Initially empty
    minHeap->capacity = capacity;                              // Set maximum capacity
    minHeap->array = (MinHeapNode**)malloc(capacity * sizeof(MinHeapNode*));
    return minHeap;
}

// Create a new min-heap node with vertex and distance
MinHeapNode* newMinHeapNode(int v, int dist) {
    MinHeapNode* minHeapNode = (MinHeapNode*)malloc(sizeof(MinHeapNode));
    minHeapNode->vertex = v;        // Vertex number
    minHeapNode->distance = dist;   // Distance/key value
    return minHeapNode;
}

// Utility function to swap two min-heap nodes
// Also updates position array to maintain consistency
void swapMinHeapNode(MinHeapNode** a, MinHeapNode** b) {
    MinHeapNode* t = *a;
    *a = *b;
    *b = t;
}

// Heapify function to maintain min-heap property
// Time complexity: O(log V)
void minHeapify(MinHeap* minHeap, int idx) {
    int smallest, left, right;
    smallest = idx;                    // Assume current node is smallest
    left = 2 * idx + 1;               // Left child index
    right = 2 * idx + 2;              // Right child index

    // Check if left child exists and is smaller than current smallest
    if (left < minHeap->size &&
        minHeap->array[left]->distance < minHeap->array[smallest]->distance)
        smallest = left;

    // Check if right child exists and is smaller than current smallest
    if (right < minHeap->size &&
        minHeap->array[right]->distance < minHeap->array[smallest]->distance)
        smallest = right;

    // If smallest is not the current node, swap and recursively heapify
    if (smallest != idx) {
        MinHeapNode* smallestNode = minHeap->array[smallest];
        MinHeapNode* idxNode = minHeap->array[idx];

        // Update position array to maintain consistency
        minHeap->pos[smallestNode->vertex] = idx;
        minHeap->pos[idxNode->vertex] = smallest;

        // Swap nodes and recursively heapify affected subtree
        swapMinHeapNode(&minHeap->array[smallest], &minHeap->array[idx]);
        minHeapify(minHeap, smallest);
    }
}

// Check if min-heap is empty
bool isEmpty(MinHeap* minHeap) {
    return minHeap->size == 0;
}

// Extract minimum element from min-heap
// Time complexity: O(log V)
MinHeapNode* extractMin(MinHeap* minHeap) {
    if (isEmpty(minHeap))
        return NULL;

    // Store the root node (minimum element)
    MinHeapNode* root = minHeap->array[0];
    
    // Replace root with last element
    MinHeapNode* lastNode = minHeap->array[minHeap->size - 1];
    minHeap->array[0] = lastNode;

    // Update position array
    minHeap->pos[root->vertex] = minHeap->size - 1;
    minHeap->pos[lastNode->vertex] = 0;

    // Reduce heap size and heapify root
    --minHeap->size;
    minHeapify(minHeap, 0);

    return root;
}

// Decrease key value of a vertex in min-heap
// Used when we find a shorter path to a vertex
// Time complexity: O(log V)
void decreaseKey(MinHeap* minHeap, int v, int dist) {
    // Get the index of vertex v in heap array
    int i = minHeap->pos[v];
    
    // Update the distance value
    minHeap->array[i]->distance = dist;

    // Travel up while the complete tree is not heapified
    // This is O(log V) loop
    while (i && minHeap->array[i]->distance < minHeap->array[(i - 1) / 2]->distance) {
        // Update position array
        minHeap->pos[minHeap->array[i]->vertex] = (i - 1) / 2;
        minHeap->pos[minHeap->array[(i - 1) / 2]->vertex] = i;
        
        // Swap current node with its parent
        swapMinHeapNode(&minHeap->array[i], &minHeap->array[(i - 1) / 2]);
        
        // Move to parent index
        i = (i - 1) / 2;
    }
}

// Check if a vertex is in min-heap (not yet processed)
bool isInMinHeap(MinHeap* minHeap, int v) {
    return minHeap->pos[v] < minHeap->size;
}

// Union-Find functions for Kruskal's algorithm
// These functions help detect cycles efficiently during MST construction

// Create Union-Find structure with path compression and union by rank
UnionFind* createUnionFind(int vertices) {
    UnionFind* uf = (UnionFind*)malloc(sizeof(UnionFind));
    uf->parent = (int*)malloc(vertices * sizeof(int));
    uf->rank = (int*)malloc(vertices * sizeof(int));
    
    // Initially, every vertex is its own parent (separate component)
    for (int i = 0; i < vertices; i++) {
        uf->parent[i] = i;     // Each vertex is its own parent
        uf->rank[i] = 0;       // Initial rank is 0
    }
    return uf;
}

// Find operation with path compression optimization
// Time complexity: O(α(n)) where α is inverse Ackermann function (practically constant)
int find(UnionFind* uf, int i) {
    // Path compression: make every node point directly to root
    if (uf->parent[i] != i)
        uf->parent[i] = find(uf, uf->parent[i]);
    return uf->parent[i];
}

// Union operation with union by rank optimization
// Time complexity: O(α(n))
void unionSets(UnionFind* uf, int x, int y) {
    int xroot = find(uf, x);    // Find root of x
    int yroot = find(uf, y);    // Find root of y
    
    // Union by rank: attach smaller ranked tree under root of higher ranked tree
    if (uf->rank[xroot] < uf->rank[yroot])
        uf->parent[xroot] = yroot;
    else if (uf->rank[xroot] > uf->rank[yroot])
        uf->parent[yroot] = xroot;
    else {
        // If ranks are same, make one root and increment its rank
        uf->parent[yroot] = xroot;
        uf->rank[xroot]++;
    }
}

// Compare function for qsort (used in Kruskal's)
int compareEdges(const void* a, const void* b) {
    Edge* a1 = (Edge*)a;
    Edge* b1 = (Edge*)b;
    return a1->weight > b1->weight;
}

// ==================== DIJKSTRA'S ALGORITHM ====================

/*
GENERAL DIJKSTRA'S ALGORITHM:
Purpose: Find shortest paths from a source vertex to all other vertices in a weighted graph with non-negative edge weights.

Algorithm Steps:
1. Initialize distance to source as 0 and all other distances as infinity
2. Create a set of unvisited vertices
3. While there are unvisited vertices:
   a) Select unvisited vertex with minimum distance
   b) Mark it as visited
   c) Update distances to all unvisited neighbors:
      - If (current_distance + edge_weight) < neighbor_distance:
          neighbor_distance = current_distance + edge_weight
4. Return array of shortest distances

Time Complexity: O(V²) with array, O((V+E)logV) with priority queue
Space Complexity: O(V)
Applications: GPS navigation, network routing, social networking
*/

/*
DIJKSTRA'S ALGORITHM (Adjacency Matrix):
- Uses 2D array to represent graph
- Simple implementation with O(V²) time complexity
- Best for dense graphs where E ≈ V²
- Memory usage: O(V²)
*/
void dijkstraMatrix(int graph[MAX_VERTICES][MAX_VERTICES], int vertices, int src) {
    int dist[MAX_VERTICES];
    bool sptSet[MAX_VERTICES];
    
    // Initialize distances and visited set
    for (int i = 0; i < vertices; i++) {
        dist[i] = INF;
        sptSet[i] = false;
    }
    dist[src] = 0;
    
    // Find shortest path for all vertices
    for (int count = 0; count < vertices - 1; count++) {
        // Find minimum distance vertex not in sptSet
        int u = -1;
        for (int v = 0; v < vertices; v++) {
            if (!sptSet[v] && (u == -1 || dist[v] < dist[u]))
                u = v;
        }
        
        sptSet[u] = true;
        
        // Update distances of adjacent vertices
        for (int v = 0; v < vertices; v++) {
            if (!sptSet[v] && graph[u][v] && dist[u] != INF && 
                dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
            }
        }
    }
    
    printf("\nDijkstra's Algorithm (Adjacency Matrix):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i]);
}

/*
DIJKSTRA'S ALGORITHM (Adjacency List):
- Uses linked list representation
- Optimized with min-heap for better performance
- Best for sparse graphs where E << V²
- Memory usage: O(V + E)
*/
void dijkstraList(GraphList* graph, int src) {
    int vertices = graph->vertices;
    int* dist = (int*)malloc(vertices * sizeof(int));
    
    MinHeap* minHeap = createMinHeap(vertices);
    
    // Initialize distances and heap
    for (int v = 0; v < vertices; v++) {
        dist[v] = INF;
        minHeap->array[v] = newMinHeapNode(v, dist[v]);
        minHeap->pos[v] = v;
    }
    
    minHeap->array[src] = newMinHeapNode(src, dist[src]);
    minHeap->pos[src] = src;
    dist[src] = 0;
    decreaseKey(minHeap, src, dist[src]);
    minHeap->size = vertices;
    
    while (!isEmpty(minHeap)) {
        MinHeapNode* minHeapNode = extractMin(minHeap);
        int u = minHeapNode->vertex;
        
        AdjListNode* pCrawl = graph->array[u].head;
        while (pCrawl != NULL) {
            int v = pCrawl->dest;
            
            if (isInMinHeap(minHeap, v) && dist[u] != INF && 
                pCrawl->weight + dist[u] < dist[v]) {
                dist[v] = dist[u] + pCrawl->weight;
                decreaseKey(minHeap, v, dist[v]);
            }
            pCrawl = pCrawl->next;
        }
    }
    
    printf("\nDijkstra's Algorithm (Adjacency List):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i]);
}

/*
DIJKSTRA'S ALGORITHM (Edge List):
- Uses array of edges
- Less efficient for Dijkstra's but included for completeness
- Requires conversion to adjacency representation or multiple edge scans
- Memory usage: O(E)
*/
void dijkstraEdgeList(GraphEdgeList* graph, int src) {
    int vertices = graph->vertices;
    int* dist = (int*)malloc(vertices * sizeof(int));
    bool* visited = (bool*)malloc(vertices * sizeof(bool));
    
    // Initialize distances and visited array
    for (int i = 0; i < vertices; i++) {
        dist[i] = INF;
        visited[i] = false;
    }
    dist[src] = 0;
    
    for (int count = 0; count < vertices - 1; count++) {
        // Find minimum distance unvisited vertex
        int u = -1;
        for (int v = 0; v < vertices; v++) {
            if (!visited[v] && (u == -1 || dist[v] < dist[u]))
                u = v;
        }
        
        visited[u] = true;
        
        // Update distances using edge list
        for (int i = 0; i < graph->edges; i++) {
            if (graph->edgeArray[i].src == u) {
                int v = graph->edgeArray[i].dest;
                int weight = graph->edgeArray[i].weight;
                if (!visited[v] && dist[u] != INF && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                }
            }
            // For undirected graphs, check reverse direction
            if (graph->edgeArray[i].dest == u) {
                int v = graph->edgeArray[i].src;
                int weight = graph->edgeArray[i].weight;
                if (!visited[v] && dist[u] != INF && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                }
            }
        }
    }
    
    printf("\nDijkstra's Algorithm (Edge List):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i]);
        
    free(dist);
    free(visited);
}

// ==================== PRIM'S ALGORITHM ====================

/*
GENERAL PRIM'S ALGORITHM:
Purpose: Find Minimum Spanning Tree (MST) of a connected, undirected, weighted graph.

Algorithm Steps:
1. Start with arbitrary vertex as initial MST
2. Initialize key values of all vertices as infinite, key of first vertex as 0
3. While MST doesn't include all vertices:
   a) Pick minimum key vertex not yet in MST
   b) Include picked vertex in MST
   c) Update key values of adjacent vertices:
      - For each adjacent vertex, if edge weight < current key:
          Update key = edge weight, parent = current vertex
4. Use parent array to construct MST

Time Complexity: O(V²) with array, O((V+E)logV) with priority queue
Space Complexity: O(V)
Applications: Network design, clustering, approximation algorithms
*/

/*
PRIM'S ALGORITHM (Adjacency Matrix):
- Simple implementation using 2D array
- Best for dense graphs
- Easy to understand and implement
*/
void primMatrix(int graph[MAX_VERTICES][MAX_VERTICES], int vertices) {
    int parent[MAX_VERTICES];
    int key[MAX_VERTICES];
    bool mstSet[MAX_VERTICES];
    
    // Initialize all keys as infinite and MST set as false
    for (int i = 0; i < vertices; i++) {
        key[i] = INF;
        mstSet[i] = false;
    }
    
    key[0] = 0;     // Start with first vertex
    parent[0] = -1; // First node is root of MST
    
    for (int count = 0; count < vertices - 1; count++) {
        // Find minimum key vertex not in MST
        int u = -1;
        for (int v = 0; v < vertices; v++) {
            if (!mstSet[v] && (u == -1 || key[v] < key[u]))
                u = v;
        }
        
        mstSet[u] = true;
        
        // Update key values of adjacent vertices
        for (int v = 0; v < vertices; v++) {
            if (graph[u][v] && !mstSet[v] && graph[u][v] < key[v]) {
                parent[v] = u;
                key[v] = graph[u][v];
            }
        }
    }
    
    printf("\nPrim's Algorithm (Adjacency Matrix):\n");
    printf("Edge   Weight\n");
    int totalWeight = 0;
    for (int i = 1; i < vertices; i++) {
        printf("%d - %d    %d\n", parent[i], i, graph[i][parent[i]]);
        totalWeight += graph[i][parent[i]];
    }
    printf("Total MST Weight: %d\n", totalWeight);
}

/*
PRIM'S ALGORITHM (Adjacency List):
- Optimized with min-heap
- Better performance for sparse graphs
- More complex implementation but efficient
*/
void primList(GraphList* graph) {
    int vertices = graph->vertices;
    int* parent = (int*)malloc(vertices * sizeof(int));
    int* key = (int*)malloc(vertices * sizeof(int));
    
    MinHeap* minHeap = createMinHeap(vertices);
    
    // Initialize heap with all vertices
    for (int v = 1; v < vertices; v++) {
        parent[v] = -1;
        key[v] = INF;
        minHeap->array[v] = newMinHeapNode(v, key[v]);
        minHeap->pos[v] = v;
    }
    
    key[0] = 0;
    minHeap->array[0] = newMinHeapNode(0, key[0]);
    minHeap->pos[0] = 0;
    minHeap->size = vertices;
    
    while (!isEmpty(minHeap)) {
        MinHeapNode* minHeapNode = extractMin(minHeap);
        int u = minHeapNode->vertex;
        
        AdjListNode* pCrawl = graph->array[u].head;
        while (pCrawl != NULL) {
            int v = pCrawl->dest;
            
            if (isInMinHeap(minHeap, v) && pCrawl->weight < key[v]) {
                key[v] = pCrawl->weight;
                parent[v] = u;
                decreaseKey(minHeap, v, key[v]);
            }
            pCrawl = pCrawl->next;
        }
    }
    
    printf("\nPrim's Algorithm (Adjacency List):\n");
    printf("Edge   Weight\n");
    int totalWeight = 0;
    for (int i = 1; i < vertices; i++) {
        printf("%d - %d    %d\n", parent[i], i, key[i]);
        totalWeight += key[i];
    }
    printf("Total MST Weight: %d\n", totalWeight);
}

/*
PRIM'S ALGORITHM (Edge List):
- Uses edge array with priority queue approach
- Less efficient than adjacency representations for Prim's
- Requires careful edge management
*/
void primEdgeList(GraphEdgeList* graph) {
    int vertices = graph->vertices;
    int* parent = (int*)malloc(vertices * sizeof(int));
    int* key = (int*)malloc(vertices * sizeof(int));
    bool* mstSet = (bool*)malloc(vertices * sizeof(bool));
    
    // Initialize arrays
    for (int i = 0; i < vertices; i++) {
        key[i] = INF;
        mstSet[i] = false;
    }
    
    key[0] = 0;
    parent[0] = -1;
    
    for (int count = 0; count < vertices - 1; count++) {
        // Find minimum key vertex not in MST
        int u = -1;
        for (int v = 0; v < vertices; v++) {
            if (!mstSet[v] && (u == -1 || key[v] < key[u]))
                u = v;
        }
        
        mstSet[u] = true;
        
        // Update key values using edge list
        for (int i = 0; i < graph->edges; i++) {
            int v = -1, weight = 0;
            
            if (graph->edgeArray[i].src == u) {
                v = graph->edgeArray[i].dest;
                weight = graph->edgeArray[i].weight;
            } else if (graph->edgeArray[i].dest == u) {
                v = graph->edgeArray[i].src;
                weight = graph->edgeArray[i].weight;
            }
            
            if (v != -1 && !mstSet[v] && weight < key[v]) {
                parent[v] = u;
                key[v] = weight;
            }
        }
    }
    
    printf("\nPrim's Algorithm (Edge List):\n");
    printf("Edge   Weight\n");
    int totalWeight = 0;
    for (int i = 1; i < vertices; i++) {
        printf("%d - %d    %d\n", parent[i], i, key[i]);
        totalWeight += key[i];
    }
    printf("Total MST Weight: %d\n", totalWeight);
    
    free(parent);
    free(key);
    free(mstSet);
}

// ==================== BELLMAN-FORD ALGORITHM ====================

/*
GENERAL BELLMAN-FORD ALGORITHM:
Purpose: Find shortest paths from source to all vertices, can handle negative weights and detect negative cycles.

Algorithm Steps:
1. Initialize distance to source as 0 and all others as infinity
2. Relax all edges |V|-1 times:
   - For each edge (u,v) with weight w:
     If distance[u] + w < distance[v], then distance[v] = distance[u] + w
3. Check for negative weight cycles:
   - Run one more iteration, if any distance can be reduced, negative cycle exists
4. Return distances or report negative cycle

Time Complexity: O(VE)
Space Complexity: O(V)
Applications: Currency exchange, network routing with constraints
*/

/*
BELLMAN-FORD ALGORITHM (Adjacency Matrix):
- Uses 2D matrix representation
- Systematically relaxes all edges
- Good for understanding the algorithm
*/
void bellmanFordMatrix(int graph[MAX_VERTICES][MAX_VERTICES], int vertices, int src) {
    int* dist = (int*)malloc(vertices * sizeof(int));
    
    // Initialize distances
    for (int i = 0; i < vertices; i++)
        dist[i] = INF;
    dist[src] = 0;
    
    // Relax all edges |V| - 1 times
    for (int i = 0; i < vertices - 1; i++) {
        for (int u = 0; u < vertices; u++) {
            for (int v = 0; v < vertices; v++) {
                if (graph[u][v] != 0 && dist[u] != INF && 
                    dist[u] + graph[u][v] < dist[v]) {
                    dist[v] = dist[u] + graph[u][v];
                }
            }
        }
    }
    
    // Check for negative-weight cycles
    for (int u = 0; u < vertices; u++) {
        for (int v = 0; v < vertices; v++) {
            if (graph[u][v] != 0 && dist[u] != INF && 
                dist[u] + graph[u][v] < dist[v]) {
                printf("Graph contains negative weight cycle\n");
                free(dist);
                return;
            }
        }
    }
    
    printf("\nBellman-Ford Algorithm (Adjacency Matrix):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i] == INF ? -1 : dist[i]);
        
    free(dist);
}

/*
BELLMAN-FORD ALGORITHM (Adjacency List):
- Uses linked list representation
- More memory efficient for sparse graphs
- Same algorithmic approach as matrix version
*/
void bellmanFordList(GraphList* graph, int src) {
    int vertices = graph->vertices;
    int* dist = (int*)malloc(vertices * sizeof(int));
    
    // Initialize distances
    for (int i = 0; i < vertices; i++)
        dist[i] = INF;
    dist[src] = 0;
    
    // Relax all edges |V| - 1 times
    for (int i = 0; i < vertices - 1; i++) {
        for (int u = 0; u < vertices; u++) {
            if (dist[u] != INF) {
                AdjListNode* pCrawl = graph->array[u].head;
                while (pCrawl != NULL) {
                    int v = pCrawl->dest;
                    int weight = pCrawl->weight;
                    if (dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                    }
                    pCrawl = pCrawl->next;
                }
            }
        }
    }
    
    // Check for negative-weight cycles
    for (int u = 0; u < vertices; u++) {
        if (dist[u] != INF) {
            AdjListNode* pCrawl = graph->array[u].head;
            while (pCrawl != NULL) {
                int v = pCrawl->dest;
                int weight = pCrawl->weight;
                if (dist[u] + weight < dist[v]) {
                    printf("Graph contains negative weight cycle\n");
                    free(dist);
                    return;
                }
                pCrawl = pCrawl->next;
            }
        }
    }
    
    printf("\nBellman-Ford Algorithm (Adjacency List):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i] == INF ? -1 : dist[i]);
        
    free(dist);
}

/*
BELLMAN-FORD ALGORITHM (Edge List):
- Most natural representation for Bellman-Ford
- Direct implementation of the algorithm
- Efficient and straightforward
*/
void bellmanFordEdgeList(GraphEdgeList* graph, int src) {
    int vertices = graph->vertices;
    int edges = graph->edges;
    int* dist = (int*)malloc(vertices * sizeof(int));
    
    // Initialize distances
    for (int i = 0; i < vertices; i++)
        dist[i] = INF;
    dist[src] = 0;
    
    // Relax all edges |V| - 1 times
    for (int i = 0; i < vertices - 1; i++) {
        for (int j = 0; j < edges; j++) {
            int u = graph->edgeArray[j].src;
            int v = graph->edgeArray[j].dest;
            int weight = graph->edgeArray[j].weight;
            if (dist[u] != INF && dist[u] + weight < dist[v])
                dist[v] = dist[u] + weight;
        }
    }
    
    // Check for negative-weight cycles
    for (int i = 0; i < edges; i++) {
        int u = graph->edgeArray[i].src;
        int v = graph->edgeArray[i].dest;
        int weight = graph->edgeArray[i].weight;
        if (dist[u] != INF && dist[u] + weight < dist[v]) {
            printf("Graph contains negative weight cycle\n");
            free(dist);
            return;
        }
    }
    
    printf("\nBellman-Ford Algorithm (Edge List):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i] == INF ? -1 : dist[i]);
        
    free(dist);
}

// ==================== KRUSKAL'S ALGORITHM ====================

/*
GENERAL KRUSKAL'S ALGORITHM:
Purpose: Find Minimum Spanning Tree using edge-based approach with Union-Find data structure.

Algorithm Steps:
1. Sort all edges in non-decreasing order of weight
2. Initialize Union-Find structure for all vertices
3. Initialize empty MST
4. For each edge in sorted order:
   a) Check if edge connects two different components using Union-Find
   b) If yes, add edge to MST and union the components
   c) If no, skip edge (would create cycle)
5. Continue until MST has V-1 edges

Time Complexity: O(E log E) for sorting + O(E α(V)) for Union-Find ≈ O(E log E)
Space Complexity: O(V + E)
Applications: Network design, clustering, circuit design
*/

/*
KRUSKAL'S ALGORITHM (Adjacency Matrix):
- Converts matrix to edge list internally for processing
- Less efficient due to conversion overhead O(V²) space for sparse graphs
- Algorithm steps:
  1. Extract all edges from adjacency matrix
  2. Sort edges by weight
  3. Use Union-Find to avoid cycles
  4. Add edges to MST until we have V-1 edges
*/
#define MAX_VERTICES 100

typedef struct {
    int src, dest, weight;         // Edge representation for Kruskal's algorithm
} Edge;

typedef struct {
    int parent, rank;              // Union-Find node structure
} Subset;

// Find with path compression - optimized version for Kruskal's
int find(Subset subsets[], int i) {
    if (subsets[i].parent != i)
        subsets[i].parent = find(subsets, subsets[i].parent);
    return subsets[i].parent;
}

// Union by rank - optimized version for Kruskal's
void Union(Subset subsets[], int x, int y) {
    int rootX = find(subsets, x);
    int rootY = find(subsets, y);

    // Attach smaller ranked tree under root of higher ranked tree
    if (subsets[rootX].rank < subsets[rootY].rank)
        subsets[rootX].parent = rootY;
    else if (subsets[rootX].rank > subsets[rootY].rank)
        subsets[rootY].parent = rootX;
    else {
        subsets[rootY].parent = rootX;
        subsets[rootX].rank++;
    }
}

// Compare edges for qsort - sorts in ascending order of weight
int compare(const void* a, const void* b) {
    return ((Edge*)a)->weight - ((Edge*)b)->weight;
}

void kruskalMatrix(int graph[MAX_VERTICES][MAX_VERTICES], int vertices) {
    Edge edges[MAX_VERTICES * MAX_VERTICES];  // Array to store all edges
    int edgeCount = 0;

    // Step 1: Convert adjacency matrix to edge list
    // Only consider upper triangle to avoid duplicate edges in undirected graph
    for (int i = 0; i < vertices; i++) {
        for (int j = i + 1; j < vertices; j++) {
            if (graph[i][j] != 0) {           // Non-zero weight indicates edge exists
                edges[edgeCount].src = i;
                edges[edgeCount].dest = j;
                edges[edgeCount].weight = graph[i][j];
                edgeCount++;
            }
        }
    }

    // Step 2: Sort edges by weight (greedy choice - always pick minimum weight edge)
    qsort(edges, edgeCount, sizeof(Edge), compare);

    // Step 3: Initialize Union-Find structure
    Subset subsets[MAX_VERTICES];
    for (int v = 0; v < vertices; v++) {
        subsets[v].parent = v;      // Each vertex is its own parent initially
        subsets[v].rank = 0;        // Initial rank is 0
    }

    printf("MST (Adjacency Matrix):\n");
    int mstWeight = 0, e = 0;       // e = number of edges in MST
    
    // Step 4: Process edges in sorted order
    for (int i = 0; e < vertices - 1 && i < edgeCount; i++) {
        int u = edges[i].src, v = edges[i].dest;

        int setU = find(subsets, u);    // Find root of u
        int setV = find(subsets, v);    // Find root of v

        // If u and v are in different components, adding this edge won't create cycle
        if (setU != setV) {
            printf("%d -- %d == %d\n", u, v, edges[i].weight);
            mstWeight += edges[i].weight;
            Union(subsets, setU, setV);     // Union the two components
            e++;                            // Increment edge count in MST
        }
        // If setU == setV, then u and v are already connected, skip this edge
    }
    printf("Total Weight = %d\n", mstWeight);
}

// Node structure for adjacency list in Kruskal's implementation
typedef struct Node {
    int dest, weight;              // Destination vertex and edge weight
    struct Node* next;             // Pointer to next node in adjacency list
} Node;

// Adjacency list structure
typedef struct {
    Node* head;                    // Head pointer for adjacency list
} AdjList;

// Graph structure for Kruskal's adjacency list implementation
typedef struct {
    int V;                         // Number of vertices
    AdjList* array;               // Array of adjacency lists
} Graph;

// Create graph with V vertices
Graph* createGraph(int V) {
    Graph* graph = (Graph*)malloc(sizeof(Graph));
    graph->V = V;
    graph->array = (AdjList*)malloc(V * sizeof(AdjList));
    
    // Initialize all adjacency lists as empty
    for (int i = 0; i < V; i++)
        graph->array[i].head = NULL;
    return graph;
}

// Add edge (undirected) - adds edge in both directions
void addEdge(Graph* graph, int src, int dest, int weight) {
    // Add edge from src to dest
    Node* newNode = (Node*)malloc(sizeof(Node));
    newNode->dest = dest;
    newNode->weight = weight;
    newNode->next = graph->array[src].head;
    graph->array[src].head = newNode;

    // Add edge from dest to src (for undirected graph)
    newNode = (Node*)malloc(sizeof(Node));
    newNode->dest = src;
    newNode->weight = weight;
    newNode->next = graph->array[dest].head;
    graph->array[dest].head = newNode;
}

void kruskalAdjList(Graph* graph) {
    int V = graph->V;
    Edge edges[1000];              // Array to store edges (assuming max 1000 edges)
    int edgeCount = 0;

    // Step 1: Convert adjacency list to edge list
    for (int i = 0; i < V; i++) {
        Node* temp = graph->array[i].head;
        while (temp) {
            // Only add edge once for undirected graph (i < temp->dest avoids duplicates)
            if (i < temp->dest) {
                edges[edgeCount].src = i;
                edges[edgeCount].dest = temp->dest;
                edges[edgeCount].weight = temp->weight;
                edgeCount++;
            }
            temp = temp->next;
        }
    }

    // Step 2: Sort all edges by weight
    qsort(edges, edgeCount, sizeof(Edge), compare);

    // Step 3: Initialize Union-Find structure
    Subset subsets[V];
    for (int v = 0; v < V; v++) {
        subsets[v].parent = v;      // Each vertex is its own parent
        subsets[v].rank = 0;        // Initial rank is 0
    }

    printf("MST (Adjacency List):\n");
    int mstWeight = 0, e = 0;       // e tracks number of edges in MST
    
    // Step 4: Process edges in sorted order until MST is complete
    for (int i = 0; e < V - 1 && i < edgeCount; i++) {
        int u = edges[i].src, v = edges[i].dest;
        int setU = find(subsets, u);    // Find component of u
        int setV = find(subsets, v);    // Find component of v

        // If u and v are in different components, add this edge to MST
        if (setU != setV) {
            printf("%d -- %d == %d\n", u, v, edges[i].weight);
            mstWeight += edges[i].weight;
            Union(subsets, setU, setV);     // Merge the two components
            e++;                            // Increment MST edge count
        }
    }
    printf("Total Weight = %d\n", mstWeight);
}
/*
KRUSKAL'S ALGORITHM (Edge List):
- Most efficient representation for Kruskal's algorithm
- Edges are already in array format, no conversion needed
- Direct implementation of the classic algorithm
- Time complexity: O(E log E) dominated by sorting
*/
void kruskalEdgeList(Edge edges[], int V, int E) {
    // Step 1: Sort all edges by weight (greedy approach)
    qsort(edges, E, sizeof(Edge), compare);

    // Step 2: Initialize Union-Find structure for cycle detection
    Subset subsets[V];
    for (int v = 0; v < V; v++) {
        subsets[v].parent = v;      // Initially, each vertex is its own parent
        subsets[v].rank = 0;        // All ranks start at 0
    }

    printf("MST (Edge List):\n");
    int mstWeight = 0, e = 0;       // e = number of edges added to MST so far
    
    // Step 3: Process edges in order of increasing weight
    for (int i = 0; e < V - 1 && i < E; i++) {
        int u = edges[i].src, v = edges[i].dest;
        int setU = find(subsets, u);    // Find root of component containing u
        int setV = find(subsets, v);    // Find root of component containing v

        // Step 4: If u and v are in different components, include this edge
        if (setU != setV) {
            printf("%d -- %d == %d\n", u, v, edges[i].weight);
            mstWeight += edges[i].weight;
            Union(subsets, setU, setV);     // Merge the two components
            e++;                            // Increment count of MST edges
        }
        // If setU == setV, including this edge would create a cycle, so skip it
    }
    printf("Total Weight = %d\n", mstWeight);
}
=======
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <stdbool.h>

#define MAX_VERTICES 100
#define INF INT_MAX

// ==================== DATA STRUCTURES ====================

// Adjacency List Node - represents an edge in the graph
typedef struct AdjListNode {
    int dest;                      // Destination vertex of the edge
    int weight;                    // Weight/cost of the edge
    struct AdjListNode* next;      // Pointer to next node in the adjacency list
} AdjListNode;

// Adjacency List - head pointer for each vertex's adjacency list
typedef struct AdjList {
    AdjListNode* head;             // Points to the first adjacent vertex
} AdjList;

// Graph using Adjacency List representation
// Space complexity: O(V + E) where V = vertices, E = edges
typedef struct GraphList {
    int vertices;                  // Number of vertices in the graph
    AdjList* array;               // Array of adjacency lists, one for each vertex
} GraphList;

// Edge structure for edge list representation
// Used in algorithms like Kruskal's that work directly with edges
typedef struct Edge {
    int src, dest, weight;         // Source vertex, destination vertex, and edge weight
} Edge;

// Graph using Edge List representation
// Space complexity: O(E) - most memory efficient for sparse graphs
typedef struct GraphEdgeList {
    int vertices;                  // Number of vertices in the graph
    int edges;                    // Number of edges in the graph
    Edge* edgeArray;              // Array containing all edges
} GraphEdgeList;

// Union-Find (Disjoint Set) structure for Kruskal's algorithm
// Used to efficiently detect cycles and manage connected components
typedef struct UnionFind {
    int* parent;                  // Parent array for union-find structure
    int* rank;                    // Rank array for union by rank optimization
} UnionFind;

// Min-Heap Node for optimization in Dijkstra's and Prim's algorithms
// Stores vertex and its current distance/key value
typedef struct MinHeapNode {
    int vertex;                   // Vertex number
    int distance;                 // Current shortest distance (Dijkstra) or key value (Prim)
} MinHeapNode;

// Min-Heap structure for efficient minimum extraction
// Used to optimize Dijkstra's and Prim's algorithms from O(V²) to O((V+E)logV)
typedef struct MinHeap {
    int size;                     // Current number of elements in heap
    int capacity;                 // Maximum capacity of heap
    int* pos;                     // Array to store positions of vertices in heap (for decrease key)
    MinHeapNode** array;          // Array of pointers to heap nodes
} MinHeap;

// ==================== UTILITY FUNCTIONS ====================

// Create a new adjacency list node
// Used when adding edges to adjacency list representation
AdjListNode* newAdjListNode(int dest, int weight) {
    AdjListNode* newNode = (AdjListNode*)malloc(sizeof(AdjListNode));
    newNode->dest = dest;          // Set destination vertex
    newNode->weight = weight;      // Set edge weight
    newNode->next = NULL;          // Initialize next pointer
    return newNode;
}

// Create a graph with adjacency list representation
// Time complexity: O(V), Space complexity: O(V)
GraphList* createGraphList(int vertices) {
    GraphList* graph = (GraphList*)malloc(sizeof(GraphList));
    graph->vertices = vertices;
    // Allocate memory for array of adjacency lists
    graph->array = (AdjList*)malloc(vertices * sizeof(AdjList));
    
    // Initialize each adjacency list as empty
    for (int i = 0; i < vertices; i++)
        graph->array[i].head = NULL;
    
    return graph;
}

// Add edge to adjacency list graph (creates undirected edge)
// Time complexity: O(1), adds edge in both directions for undirected graph
void addEdgeList(GraphList* graph, int src, int dest, int weight) {
    // Add edge from src to dest
    AdjListNode* newNode = newAdjListNode(dest, weight);
    newNode->next = graph->array[src].head;    // Insert at beginning of list
    graph->array[src].head = newNode;
    
    // For undirected graph, add reverse edge from dest to src
    newNode = newAdjListNode(src, weight);
    newNode->next = graph->array[dest].head;   // Insert at beginning of list
    graph->array[dest].head = newNode;
}

// Create graph with edge list representation
// Most efficient for algorithms that process edges directly (like Kruskal's)
GraphEdgeList* createGraphEdgeList(int vertices, int edges) {
    GraphEdgeList* graph = (GraphEdgeList*)malloc(sizeof(GraphEdgeList));
    graph->vertices = vertices;
    graph->edges = edges;
    // Allocate memory for array of edges
    graph->edgeArray = (Edge*)malloc(edges * sizeof(Edge));
    return graph;
}

// Min-Heap functions for optimizing Dijkstra's and Prim's algorithms

// Create a min-heap with given capacity
MinHeap* createMinHeap(int capacity) {
    MinHeap* minHeap = (MinHeap*)malloc(sizeof(MinHeap));
    minHeap->pos = (int*)malloc(capacity * sizeof(int));       // Position array for decrease key
    minHeap->size = 0;                                         // Initially empty
    minHeap->capacity = capacity;                              // Set maximum capacity
    minHeap->array = (MinHeapNode**)malloc(capacity * sizeof(MinHeapNode*));
    return minHeap;
}

// Create a new min-heap node with vertex and distance
MinHeapNode* newMinHeapNode(int v, int dist) {
    MinHeapNode* minHeapNode = (MinHeapNode*)malloc(sizeof(MinHeapNode));
    minHeapNode->vertex = v;        // Vertex number
    minHeapNode->distance = dist;   // Distance/key value
    return minHeapNode;
}

// Utility function to swap two min-heap nodes
// Also updates position array to maintain consistency
void swapMinHeapNode(MinHeapNode** a, MinHeapNode** b) {
    MinHeapNode* t = *a;
    *a = *b;
    *b = t;
}

// Heapify function to maintain min-heap property
// Time complexity: O(log V)
void minHeapify(MinHeap* minHeap, int idx) {
    int smallest, left, right;
    smallest = idx;                    // Assume current node is smallest
    left = 2 * idx + 1;               // Left child index
    right = 2 * idx + 2;              // Right child index

    // Check if left child exists and is smaller than current smallest
    if (left < minHeap->size &&
        minHeap->array[left]->distance < minHeap->array[smallest]->distance)
        smallest = left;

    // Check if right child exists and is smaller than current smallest
    if (right < minHeap->size &&
        minHeap->array[right]->distance < minHeap->array[smallest]->distance)
        smallest = right;

    // If smallest is not the current node, swap and recursively heapify
    if (smallest != idx) {
        MinHeapNode* smallestNode = minHeap->array[smallest];
        MinHeapNode* idxNode = minHeap->array[idx];

        // Update position array to maintain consistency
        minHeap->pos[smallestNode->vertex] = idx;
        minHeap->pos[idxNode->vertex] = smallest;

        // Swap nodes and recursively heapify affected subtree
        swapMinHeapNode(&minHeap->array[smallest], &minHeap->array[idx]);
        minHeapify(minHeap, smallest);
    }
}

// Check if min-heap is empty
bool isEmpty(MinHeap* minHeap) {
    return minHeap->size == 0;
}

// Extract minimum element from min-heap
// Time complexity: O(log V)
MinHeapNode* extractMin(MinHeap* minHeap) {
    if (isEmpty(minHeap))
        return NULL;

    // Store the root node (minimum element)
    MinHeapNode* root = minHeap->array[0];
    
    // Replace root with last element
    MinHeapNode* lastNode = minHeap->array[minHeap->size - 1];
    minHeap->array[0] = lastNode;

    // Update position array
    minHeap->pos[root->vertex] = minHeap->size - 1;
    minHeap->pos[lastNode->vertex] = 0;

    // Reduce heap size and heapify root
    --minHeap->size;
    minHeapify(minHeap, 0);

    return root;
}

// Decrease key value of a vertex in min-heap
// Used when we find a shorter path to a vertex
// Time complexity: O(log V)
void decreaseKey(MinHeap* minHeap, int v, int dist) {
    // Get the index of vertex v in heap array
    int i = minHeap->pos[v];
    
    // Update the distance value
    minHeap->array[i]->distance = dist;

    // Travel up while the complete tree is not heapified
    // This is O(log V) loop
    while (i && minHeap->array[i]->distance < minHeap->array[(i - 1) / 2]->distance) {
        // Update position array
        minHeap->pos[minHeap->array[i]->vertex] = (i - 1) / 2;
        minHeap->pos[minHeap->array[(i - 1) / 2]->vertex] = i;
        
        // Swap current node with its parent
        swapMinHeapNode(&minHeap->array[i], &minHeap->array[(i - 1) / 2]);
        
        // Move to parent index
        i = (i - 1) / 2;
    }
}

// Check if a vertex is in min-heap (not yet processed)
bool isInMinHeap(MinHeap* minHeap, int v) {
    return minHeap->pos[v] < minHeap->size;
}

// Union-Find functions for Kruskal's algorithm
// These functions help detect cycles efficiently during MST construction

// Create Union-Find structure with path compression and union by rank
UnionFind* createUnionFind(int vertices) {
    UnionFind* uf = (UnionFind*)malloc(sizeof(UnionFind));
    uf->parent = (int*)malloc(vertices * sizeof(int));
    uf->rank = (int*)malloc(vertices * sizeof(int));
    
    // Initially, every vertex is its own parent (separate component)
    for (int i = 0; i < vertices; i++) {
        uf->parent[i] = i;     // Each vertex is its own parent
        uf->rank[i] = 0;       // Initial rank is 0
    }
    return uf;
}

// Find operation with path compression optimization
// Time complexity: O(α(n)) where α is inverse Ackermann function (practically constant)
int find(UnionFind* uf, int i) {
    // Path compression: make every node point directly to root
    if (uf->parent[i] != i)
        uf->parent[i] = find(uf, uf->parent[i]);
    return uf->parent[i];
}

// Union operation with union by rank optimization
// Time complexity: O(α(n))
void unionSets(UnionFind* uf, int x, int y) {
    int xroot = find(uf, x);    // Find root of x
    int yroot = find(uf, y);    // Find root of y
    
    // Union by rank: attach smaller ranked tree under root of higher ranked tree
    if (uf->rank[xroot] < uf->rank[yroot])
        uf->parent[xroot] = yroot;
    else if (uf->rank[xroot] > uf->rank[yroot])
        uf->parent[yroot] = xroot;
    else {
        // If ranks are same, make one root and increment its rank
        uf->parent[yroot] = xroot;
        uf->rank[xroot]++;
    }
}

// Compare function for qsort (used in Kruskal's)
int compareEdges(const void* a, const void* b) {
    Edge* a1 = (Edge*)a;
    Edge* b1 = (Edge*)b;
    return a1->weight > b1->weight;
}

// ==================== DIJKSTRA'S ALGORITHM ====================

/*
GENERAL DIJKSTRA'S ALGORITHM:
Purpose: Find shortest paths from a source vertex to all other vertices in a weighted graph with non-negative edge weights.

Algorithm Steps:
1. Initialize distance to source as 0 and all other distances as infinity
2. Create a set of unvisited vertices
3. While there are unvisited vertices:
   a) Select unvisited vertex with minimum distance
   b) Mark it as visited
   c) Update distances to all unvisited neighbors:
      - If (current_distance + edge_weight) < neighbor_distance:
          neighbor_distance = current_distance + edge_weight
4. Return array of shortest distances

Time Complexity: O(V²) with array, O((V+E)logV) with priority queue
Space Complexity: O(V)
Applications: GPS navigation, network routing, social networking
*/

/*
DIJKSTRA'S ALGORITHM (Adjacency Matrix):
- Uses 2D array to represent graph
- Simple implementation with O(V²) time complexity
- Best for dense graphs where E ≈ V²
- Memory usage: O(V²)
*/
void dijkstraMatrix(int graph[MAX_VERTICES][MAX_VERTICES], int vertices, int src) {
    int dist[MAX_VERTICES];
    bool sptSet[MAX_VERTICES];
    
    // Initialize distances and visited set
    for (int i = 0; i < vertices; i++) {
        dist[i] = INF;
        sptSet[i] = false;
    }
    dist[src] = 0;
    
    // Find shortest path for all vertices
    for (int count = 0; count < vertices - 1; count++) {
        // Find minimum distance vertex not in sptSet
        int u = -1;
        for (int v = 0; v < vertices; v++) {
            if (!sptSet[v] && (u == -1 || dist[v] < dist[u]))
                u = v;
        }
        
        sptSet[u] = true;
        
        // Update distances of adjacent vertices
        for (int v = 0; v < vertices; v++) {
            if (!sptSet[v] && graph[u][v] && dist[u] != INF && 
                dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
            }
        }
    }
    
    printf("\nDijkstra's Algorithm (Adjacency Matrix):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i]);
}

/*
DIJKSTRA'S ALGORITHM (Adjacency List):
- Uses linked list representation
- Optimized with min-heap for better performance
- Best for sparse graphs where E << V²
- Memory usage: O(V + E)
*/
void dijkstraList(GraphList* graph, int src) {
    int vertices = graph->vertices;
    int* dist = (int*)malloc(vertices * sizeof(int));
    
    MinHeap* minHeap = createMinHeap(vertices);
    
    // Initialize distances and heap
    for (int v = 0; v < vertices; v++) {
        dist[v] = INF;
        minHeap->array[v] = newMinHeapNode(v, dist[v]);
        minHeap->pos[v] = v;
    }
    
    minHeap->array[src] = newMinHeapNode(src, dist[src]);
    minHeap->pos[src] = src;
    dist[src] = 0;
    decreaseKey(minHeap, src, dist[src]);
    minHeap->size = vertices;
    
    while (!isEmpty(minHeap)) {
        MinHeapNode* minHeapNode = extractMin(minHeap);
        int u = minHeapNode->vertex;
        
        AdjListNode* pCrawl = graph->array[u].head;
        while (pCrawl != NULL) {
            int v = pCrawl->dest;
            
            if (isInMinHeap(minHeap, v) && dist[u] != INF && 
                pCrawl->weight + dist[u] < dist[v]) {
                dist[v] = dist[u] + pCrawl->weight;
                decreaseKey(minHeap, v, dist[v]);
            }
            pCrawl = pCrawl->next;
        }
    }
    
    printf("\nDijkstra's Algorithm (Adjacency List):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i]);
}

/*
DIJKSTRA'S ALGORITHM (Edge List):
- Uses array of edges
- Less efficient for Dijkstra's but included for completeness
- Requires conversion to adjacency representation or multiple edge scans
- Memory usage: O(E)
*/
void dijkstraEdgeList(GraphEdgeList* graph, int src) {
    int vertices = graph->vertices;
    int* dist = (int*)malloc(vertices * sizeof(int));
    bool* visited = (bool*)malloc(vertices * sizeof(bool));
    
    // Initialize distances and visited array
    for (int i = 0; i < vertices; i++) {
        dist[i] = INF;
        visited[i] = false;
    }
    dist[src] = 0;
    
    for (int count = 0; count < vertices - 1; count++) {
        // Find minimum distance unvisited vertex
        int u = -1;
        for (int v = 0; v < vertices; v++) {
            if (!visited[v] && (u == -1 || dist[v] < dist[u]))
                u = v;
        }
        
        visited[u] = true;
        
        // Update distances using edge list
        for (int i = 0; i < graph->edges; i++) {
            if (graph->edgeArray[i].src == u) {
                int v = graph->edgeArray[i].dest;
                int weight = graph->edgeArray[i].weight;
                if (!visited[v] && dist[u] != INF && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                }
            }
            // For undirected graphs, check reverse direction
            if (graph->edgeArray[i].dest == u) {
                int v = graph->edgeArray[i].src;
                int weight = graph->edgeArray[i].weight;
                if (!visited[v] && dist[u] != INF && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                }
            }
        }
    }
    
    printf("\nDijkstra's Algorithm (Edge List):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i]);
        
    free(dist);
    free(visited);
}

// ==================== PRIM'S ALGORITHM ====================

/*
GENERAL PRIM'S ALGORITHM:
Purpose: Find Minimum Spanning Tree (MST) of a connected, undirected, weighted graph.

Algorithm Steps:
1. Start with arbitrary vertex as initial MST
2. Initialize key values of all vertices as infinite, key of first vertex as 0
3. While MST doesn't include all vertices:
   a) Pick minimum key vertex not yet in MST
   b) Include picked vertex in MST
   c) Update key values of adjacent vertices:
      - For each adjacent vertex, if edge weight < current key:
          Update key = edge weight, parent = current vertex
4. Use parent array to construct MST

Time Complexity: O(V²) with array, O((V+E)logV) with priority queue
Space Complexity: O(V)
Applications: Network design, clustering, approximation algorithms
*/

/*
PRIM'S ALGORITHM (Adjacency Matrix):
- Simple implementation using 2D array
- Best for dense graphs
- Easy to understand and implement
*/
void primMatrix(int graph[MAX_VERTICES][MAX_VERTICES], int vertices) {
    int parent[MAX_VERTICES];
    int key[MAX_VERTICES];
    bool mstSet[MAX_VERTICES];
    
    // Initialize all keys as infinite and MST set as false
    for (int i = 0; i < vertices; i++) {
        key[i] = INF;
        mstSet[i] = false;
    }
    
    key[0] = 0;     // Start with first vertex
    parent[0] = -1; // First node is root of MST
    
    for (int count = 0; count < vertices - 1; count++) {
        // Find minimum key vertex not in MST
        int u = -1;
        for (int v = 0; v < vertices; v++) {
            if (!mstSet[v] && (u == -1 || key[v] < key[u]))
                u = v;
        }
        
        mstSet[u] = true;
        
        // Update key values of adjacent vertices
        for (int v = 0; v < vertices; v++) {
            if (graph[u][v] && !mstSet[v] && graph[u][v] < key[v]) {
                parent[v] = u;
                key[v] = graph[u][v];
            }
        }
    }
    
    printf("\nPrim's Algorithm (Adjacency Matrix):\n");
    printf("Edge   Weight\n");
    int totalWeight = 0;
    for (int i = 1; i < vertices; i++) {
        printf("%d - %d    %d\n", parent[i], i, graph[i][parent[i]]);
        totalWeight += graph[i][parent[i]];
    }
    printf("Total MST Weight: %d\n", totalWeight);
}

/*
PRIM'S ALGORITHM (Adjacency List):
- Optimized with min-heap
- Better performance for sparse graphs
- More complex implementation but efficient
*/
void primList(GraphList* graph) {
    int vertices = graph->vertices;
    int* parent = (int*)malloc(vertices * sizeof(int));
    int* key = (int*)malloc(vertices * sizeof(int));
    
    MinHeap* minHeap = createMinHeap(vertices);
    
    // Initialize heap with all vertices
    for (int v = 1; v < vertices; v++) {
        parent[v] = -1;
        key[v] = INF;
        minHeap->array[v] = newMinHeapNode(v, key[v]);
        minHeap->pos[v] = v;
    }
    
    key[0] = 0;
    minHeap->array[0] = newMinHeapNode(0, key[0]);
    minHeap->pos[0] = 0;
    minHeap->size = vertices;
    
    while (!isEmpty(minHeap)) {
        MinHeapNode* minHeapNode = extractMin(minHeap);
        int u = minHeapNode->vertex;
        
        AdjListNode* pCrawl = graph->array[u].head;
        while (pCrawl != NULL) {
            int v = pCrawl->dest;
            
            if (isInMinHeap(minHeap, v) && pCrawl->weight < key[v]) {
                key[v] = pCrawl->weight;
                parent[v] = u;
                decreaseKey(minHeap, v, key[v]);
            }
            pCrawl = pCrawl->next;
        }
    }
    
    printf("\nPrim's Algorithm (Adjacency List):\n");
    printf("Edge   Weight\n");
    int totalWeight = 0;
    for (int i = 1; i < vertices; i++) {
        printf("%d - %d    %d\n", parent[i], i, key[i]);
        totalWeight += key[i];
    }
    printf("Total MST Weight: %d\n", totalWeight);
}

/*
PRIM'S ALGORITHM (Edge List):
- Uses edge array with priority queue approach
- Less efficient than adjacency representations for Prim's
- Requires careful edge management
*/
void primEdgeList(GraphEdgeList* graph) {
    int vertices = graph->vertices;
    int* parent = (int*)malloc(vertices * sizeof(int));
    int* key = (int*)malloc(vertices * sizeof(int));
    bool* mstSet = (bool*)malloc(vertices * sizeof(bool));
    
    // Initialize arrays
    for (int i = 0; i < vertices; i++) {
        key[i] = INF;
        mstSet[i] = false;
    }
    
    key[0] = 0;
    parent[0] = -1;
    
    for (int count = 0; count < vertices - 1; count++) {
        // Find minimum key vertex not in MST
        int u = -1;
        for (int v = 0; v < vertices; v++) {
            if (!mstSet[v] && (u == -1 || key[v] < key[u]))
                u = v;
        }
        
        mstSet[u] = true;
        
        // Update key values using edge list
        for (int i = 0; i < graph->edges; i++) {
            int v = -1, weight = 0;
            
            if (graph->edgeArray[i].src == u) {
                v = graph->edgeArray[i].dest;
                weight = graph->edgeArray[i].weight;
            } else if (graph->edgeArray[i].dest == u) {
                v = graph->edgeArray[i].src;
                weight = graph->edgeArray[i].weight;
            }
            
            if (v != -1 && !mstSet[v] && weight < key[v]) {
                parent[v] = u;
                key[v] = weight;
            }
        }
    }
    
    printf("\nPrim's Algorithm (Edge List):\n");
    printf("Edge   Weight\n");
    int totalWeight = 0;
    for (int i = 1; i < vertices; i++) {
        printf("%d - %d    %d\n", parent[i], i, key[i]);
        totalWeight += key[i];
    }
    printf("Total MST Weight: %d\n", totalWeight);
    
    free(parent);
    free(key);
    free(mstSet);
}

// ==================== BELLMAN-FORD ALGORITHM ====================

/*
GENERAL BELLMAN-FORD ALGORITHM:
Purpose: Find shortest paths from source to all vertices, can handle negative weights and detect negative cycles.

Algorithm Steps:
1. Initialize distance to source as 0 and all others as infinity
2. Relax all edges |V|-1 times:
   - For each edge (u,v) with weight w:
     If distance[u] + w < distance[v], then distance[v] = distance[u] + w
3. Check for negative weight cycles:
   - Run one more iteration, if any distance can be reduced, negative cycle exists
4. Return distances or report negative cycle

Time Complexity: O(VE)
Space Complexity: O(V)
Applications: Currency exchange, network routing with constraints
*/

/*
BELLMAN-FORD ALGORITHM (Adjacency Matrix):
- Uses 2D matrix representation
- Systematically relaxes all edges
- Good for understanding the algorithm
*/
void bellmanFordMatrix(int graph[MAX_VERTICES][MAX_VERTICES], int vertices, int src) {
    int* dist = (int*)malloc(vertices * sizeof(int));
    
    // Initialize distances
    for (int i = 0; i < vertices; i++)
        dist[i] = INF;
    dist[src] = 0;
    
    // Relax all edges |V| - 1 times
    for (int i = 0; i < vertices - 1; i++) {
        for (int u = 0; u < vertices; u++) {
            for (int v = 0; v < vertices; v++) {
                if (graph[u][v] != 0 && dist[u] != INF && 
                    dist[u] + graph[u][v] < dist[v]) {
                    dist[v] = dist[u] + graph[u][v];
                }
            }
        }
    }
    
    // Check for negative-weight cycles
    for (int u = 0; u < vertices; u++) {
        for (int v = 0; v < vertices; v++) {
            if (graph[u][v] != 0 && dist[u] != INF && 
                dist[u] + graph[u][v] < dist[v]) {
                printf("Graph contains negative weight cycle\n");
                free(dist);
                return;
            }
        }
    }
    
    printf("\nBellman-Ford Algorithm (Adjacency Matrix):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i] == INF ? -1 : dist[i]);
        
    free(dist);
}

/*
BELLMAN-FORD ALGORITHM (Adjacency List):
- Uses linked list representation
- More memory efficient for sparse graphs
- Same algorithmic approach as matrix version
*/
void bellmanFordList(GraphList* graph, int src) {
    int vertices = graph->vertices;
    int* dist = (int*)malloc(vertices * sizeof(int));
    
    // Initialize distances
    for (int i = 0; i < vertices; i++)
        dist[i] = INF;
    dist[src] = 0;
    
    // Relax all edges |V| - 1 times
    for (int i = 0; i < vertices - 1; i++) {
        for (int u = 0; u < vertices; u++) {
            if (dist[u] != INF) {
                AdjListNode* pCrawl = graph->array[u].head;
                while (pCrawl != NULL) {
                    int v = pCrawl->dest;
                    int weight = pCrawl->weight;
                    if (dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                    }
                    pCrawl = pCrawl->next;
                }
            }
        }
    }
    
    // Check for negative-weight cycles
    for (int u = 0; u < vertices; u++) {
        if (dist[u] != INF) {
            AdjListNode* pCrawl = graph->array[u].head;
            while (pCrawl != NULL) {
                int v = pCrawl->dest;
                int weight = pCrawl->weight;
                if (dist[u] + weight < dist[v]) {
                    printf("Graph contains negative weight cycle\n");
                    free(dist);
                    return;
                }
                pCrawl = pCrawl->next;
            }
        }
    }
    
    printf("\nBellman-Ford Algorithm (Adjacency List):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i] == INF ? -1 : dist[i]);
        
    free(dist);
}

/*
BELLMAN-FORD ALGORITHM (Edge List):
- Most natural representation for Bellman-Ford
- Direct implementation of the algorithm
- Efficient and straightforward
*/
void bellmanFordEdgeList(GraphEdgeList* graph, int src) {
    int vertices = graph->vertices;
    int edges = graph->edges;
    int* dist = (int*)malloc(vertices * sizeof(int));
    
    // Initialize distances
    for (int i = 0; i < vertices; i++)
        dist[i] = INF;
    dist[src] = 0;
    
    // Relax all edges |V| - 1 times
    for (int i = 0; i < vertices - 1; i++) {
        for (int j = 0; j < edges; j++) {
            int u = graph->edgeArray[j].src;
            int v = graph->edgeArray[j].dest;
            int weight = graph->edgeArray[j].weight;
            if (dist[u] != INF && dist[u] + weight < dist[v])
                dist[v] = dist[u] + weight;
        }
    }
    
    // Check for negative-weight cycles
    for (int i = 0; i < edges; i++) {
        int u = graph->edgeArray[i].src;
        int v = graph->edgeArray[i].dest;
        int weight = graph->edgeArray[i].weight;
        if (dist[u] != INF && dist[u] + weight < dist[v]) {
            printf("Graph contains negative weight cycle\n");
            free(dist);
            return;
        }
    }
    
    printf("\nBellman-Ford Algorithm (Edge List):\n");
    printf("Vertex   Distance from Source %d\n", src);
    for (int i = 0; i < vertices; i++)
        printf("%d        %d\n", i, dist[i] == INF ? -1 : dist[i]);
        
    free(dist);
}

// ==================== KRUSKAL'S ALGORITHM ====================

/*
GENERAL KRUSKAL'S ALGORITHM:
Purpose: Find Minimum Spanning Tree using edge-based approach with Union-Find data structure.

Algorithm Steps:
1. Sort all edges in non-decreasing order of weight
2. Initialize Union-Find structure for all vertices
3. Initialize empty MST
4. For each edge in sorted order:
   a) Check if edge connects two different components using Union-Find
   b) If yes, add edge to MST and union the components
   c) If no, skip edge (would create cycle)
5. Continue until MST has V-1 edges

Time Complexity: O(E log E) for sorting + O(E α(V)) for Union-Find ≈ O(E log E)
Space Complexity: O(V + E)
Applications: Network design, clustering, circuit design
*/

/*
KRUSKAL'S ALGORITHM (Adjacency Matrix):
- Converts matrix to edge list internally for processing
- Less efficient due to conversion overhead O(V²) space for sparse graphs
- Algorithm steps:
  1. Extract all edges from adjacency matrix
  2. Sort edges by weight
  3. Use Union-Find to avoid cycles
  4. Add edges to MST until we have V-1 edges
*/
#define MAX_VERTICES 100

typedef struct {
    int src, dest, weight;         // Edge representation for Kruskal's algorithm
} Edge;

typedef struct {
    int parent, rank;              // Union-Find node structure
} Subset;

// Find with path compression - optimized version for Kruskal's
int find(Subset subsets[], int i) {
    if (subsets[i].parent != i)
        subsets[i].parent = find(subsets, subsets[i].parent);
    return subsets[i].parent;
}

// Union by rank - optimized version for Kruskal's
void Union(Subset subsets[], int x, int y) {
    int rootX = find(subsets, x);
    int rootY = find(subsets, y);

    // Attach smaller ranked tree under root of higher ranked tree
    if (subsets[rootX].rank < subsets[rootY].rank)
        subsets[rootX].parent = rootY;
    else if (subsets[rootX].rank > subsets[rootY].rank)
        subsets[rootY].parent = rootX;
    else {
        subsets[rootY].parent = rootX;
        subsets[rootX].rank++;
    }
}

// Compare edges for qsort - sorts in ascending order of weight
int compare(const void* a, const void* b) {
    return ((Edge*)a)->weight - ((Edge*)b)->weight;
}

void kruskalMatrix(int graph[MAX_VERTICES][MAX_VERTICES], int vertices) {
    Edge edges[MAX_VERTICES * MAX_VERTICES];  // Array to store all edges
    int edgeCount = 0;

    // Step 1: Convert adjacency matrix to edge list
    // Only consider upper triangle to avoid duplicate edges in undirected graph
    for (int i = 0; i < vertices; i++) {
        for (int j = i + 1; j < vertices; j++) {
            if (graph[i][j] != 0) {           // Non-zero weight indicates edge exists
                edges[edgeCount].src = i;
                edges[edgeCount].dest = j;
                edges[edgeCount].weight = graph[i][j];
                edgeCount++;
            }
        }
    }

    // Step 2: Sort edges by weight (greedy choice - always pick minimum weight edge)
    qsort(edges, edgeCount, sizeof(Edge), compare);

    // Step 3: Initialize Union-Find structure
    Subset subsets[MAX_VERTICES];
    for (int v = 0; v < vertices; v++) {
        subsets[v].parent = v;      // Each vertex is its own parent initially
        subsets[v].rank = 0;        // Initial rank is 0
    }

    printf("MST (Adjacency Matrix):\n");
    int mstWeight = 0, e = 0;       // e = number of edges in MST
    
    // Step 4: Process edges in sorted order
    for (int i = 0; e < vertices - 1 && i < edgeCount; i++) {
        int u = edges[i].src, v = edges[i].dest;

        int setU = find(subsets, u);    // Find root of u
        int setV = find(subsets, v);    // Find root of v

        // If u and v are in different components, adding this edge won't create cycle
        if (setU != setV) {
            printf("%d -- %d == %d\n", u, v, edges[i].weight);
            mstWeight += edges[i].weight;
            Union(subsets, setU, setV);     // Union the two components
            e++;                            // Increment edge count in MST
        }
        // If setU == setV, then u and v are already connected, skip this edge
    }
    printf("Total Weight = %d\n", mstWeight);
}

// Node structure for adjacency list in Kruskal's implementation
typedef struct Node {
    int dest, weight;              // Destination vertex and edge weight
    struct Node* next;             // Pointer to next node in adjacency list
} Node;

// Adjacency list structure
typedef struct {
    Node* head;                    // Head pointer for adjacency list
} AdjList;

// Graph structure for Kruskal's adjacency list implementation
typedef struct {
    int V;                         // Number of vertices
    AdjList* array;               // Array of adjacency lists
} Graph;

// Create graph with V vertices
Graph* createGraph(int V) {
    Graph* graph = (Graph*)malloc(sizeof(Graph));
    graph->V = V;
    graph->array = (AdjList*)malloc(V * sizeof(AdjList));
    
    // Initialize all adjacency lists as empty
    for (int i = 0; i < V; i++)
        graph->array[i].head = NULL;
    return graph;
}

// Add edge (undirected) - adds edge in both directions
void addEdge(Graph* graph, int src, int dest, int weight) {
    // Add edge from src to dest
    Node* newNode = (Node*)malloc(sizeof(Node));
    newNode->dest = dest;
    newNode->weight = weight;
    newNode->next = graph->array[src].head;
    graph->array[src].head = newNode;

    // Add edge from dest to src (for undirected graph)
    newNode = (Node*)malloc(sizeof(Node));
    newNode->dest = src;
    newNode->weight = weight;
    newNode->next = graph->array[dest].head;
    graph->array[dest].head = newNode;
}

void kruskalAdjList(Graph* graph) {
    int V = graph->V;
    Edge edges[1000];              // Array to store edges (assuming max 1000 edges)
    int edgeCount = 0;

    // Step 1: Convert adjacency list to edge list
    for (int i = 0; i < V; i++) {
        Node* temp = graph->array[i].head;
        while (temp) {
            // Only add edge once for undirected graph (i < temp->dest avoids duplicates)
            if (i < temp->dest) {
                edges[edgeCount].src = i;
                edges[edgeCount].dest = temp->dest;
                edges[edgeCount].weight = temp->weight;
                edgeCount++;
            }
            temp = temp->next;
        }
    }

    // Step 2: Sort all edges by weight
    qsort(edges, edgeCount, sizeof(Edge), compare);

    // Step 3: Initialize Union-Find structure
    Subset subsets[V];
    for (int v = 0; v < V; v++) {
        subsets[v].parent = v;      // Each vertex is its own parent
        subsets[v].rank = 0;        // Initial rank is 0
    }

    printf("MST (Adjacency List):\n");
    int mstWeight = 0, e = 0;       // e tracks number of edges in MST
    
    // Step 4: Process edges in sorted order until MST is complete
    for (int i = 0; e < V - 1 && i < edgeCount; i++) {
        int u = edges[i].src, v = edges[i].dest;
        int setU = find(subsets, u);    // Find component of u
        int setV = find(subsets, v);    // Find component of v

        // If u and v are in different components, add this edge to MST
        if (setU != setV) {
            printf("%d -- %d == %d\n", u, v, edges[i].weight);
            mstWeight += edges[i].weight;
            Union(subsets, setU, setV);     // Merge the two components
            e++;                            // Increment MST edge count
        }
    }
    printf("Total Weight = %d\n", mstWeight);
}
/*
KRUSKAL'S ALGORITHM (Edge List):
- Most efficient representation for Kruskal's algorithm
- Edges are already in array format, no conversion needed
- Direct implementation of the classic algorithm
- Time complexity: O(E log E) dominated by sorting
*/
void kruskalEdgeList(Edge edges[], int V, int E) {
    // Step 1: Sort all edges by weight (greedy approach)
    qsort(edges, E, sizeof(Edge), compare);

    // Step 2: Initialize Union-Find structure for cycle detection
    Subset subsets[V];
    for (int v = 0; v < V; v++) {
        subsets[v].parent = v;      // Initially, each vertex is its own parent
        subsets[v].rank = 0;        // All ranks start at 0
    }

    printf("MST (Edge List):\n");
    int mstWeight = 0, e = 0;       // e = number of edges added to MST so far
    
    // Step 3: Process edges in order of increasing weight
    for (int i = 0; e < V - 1 && i < E; i++) {
        int u = edges[i].src, v = edges[i].dest;
        int setU = find(subsets, u);    // Find root of component containing u
        int setV = find(subsets, v);    // Find root of component containing v

        // Step 4: If u and v are in different components, include this edge
        if (setU != setV) {
            printf("%d -- %d == %d\n", u, v, edges[i].weight);
            mstWeight += edges[i].weight;
            Union(subsets, setU, setV);     // Merge the two components
            e++;                            // Increment count of MST edges
        }
        // If setU == setV, including this edge would create a cycle, so skip it
    }
    printf("Total Weight = %d\n", mstWeight);
}
>>>>>>> 204403d4b0c961e66f0a8b9515e658b8fa445814
