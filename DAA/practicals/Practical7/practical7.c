#include <stdio.h>
#include <stdlib.h>

#define MAX_ELEMENTS 100

// Structure to represent an element in the disjoint set
typedef struct {
    int parent;
    int rank;  // Used for union by rank optimization
} Element;

Element disjointSet[MAX_ELEMENTS];

// MakeSet: Initialize a singleton set for element x
void MakeSet(int x) {
    if (x >= 0 && x < MAX_ELEMENTS) {
        disjointSet[x].parent = x;  // Element is its own parent
        disjointSet[x].rank = 0;    // Initial rank is 0
    }
}

// Find: Return the root of the tree containing x
int Find(int x) {
    if (x < 0 || x >= MAX_ELEMENTS) {
        return -1;
    }
    
    // Path compression: Make all nodes point directly to root
    if (disjointSet[x].parent != x) {
        disjointSet[x].parent = Find(disjointSet[x].parent);
    }
    
    return disjointSet[x].parent;
}

// Union: Combine two disjoint sets containing x and y
void Union(int x, int y) {
    int rootX = Find(x);
    int rootY = Find(y);
    
    // If already in same set, do nothing
    if (rootX == rootY) {
        return;
    }
    
    // Union by rank: attach smaller tree under root of larger tree
    if (disjointSet[rootX].rank < disjointSet[rootY].rank) {
        disjointSet[rootX].parent = rootY;
    } else if (disjointSet[rootX].rank > disjointSet[rootY].rank) {
        disjointSet[rootY].parent = rootX;
    } else {
        disjointSet[rootY].parent = rootX;
        disjointSet[rootX].rank++;
    }
}

// Display the parent array
void displaySets(int n) {
    printf("\nElement: ");
    for (int i = 0; i < n; i++) {
        printf("%3d ", i);
    }
    printf("\nParent:  ");
    for (int i = 0; i < n; i++) {
        printf("%3d ", disjointSet[i].parent);
    }
    printf("\n");
}

int main() {
    int n, choice, x, y;
    
    printf("Enter the number of elements: ");
    scanf("%d", &n);
    
    // Initialize all elements as singleton sets
    for (int i = 0; i < n; i++) {
        MakeSet(i);
    }
    
    printf("\nInitial sets created for elements 0 to %d\n", n - 1);
    
    while (1) {
        printf("\n--- Disjoint Set Operations ---");
        printf("\n1. Find(x)");
        printf("\n2. Union(x, y)");
        printf("\n3. Display Sets");
        printf("\n4. Exit");
        printf("\nEnter your choice: ");
        scanf("%d", &choice);
        
        switch (choice) {
            case 1:
                printf("Enter element to find: ");
                scanf("%d", &x);
                if (x >= 0 && x < n) {
                    printf("Representative of set containing %d is: %d\n", x, Find(x));
                } else {
                    printf("Invalid element!\n");
                }
                break;
                
            case 2:
                printf("Enter two elements to union: ");
                scanf("%d %d", &x, &y);
                if (x >= 0 && x < n && y >= 0 && y < n) {
                    Union(x, y);
                    printf("Union of sets containing %d and %d performed.\n", x, y);
                } else {
                    printf("Invalid elements!\n");
                }
                break;
                
            case 3:
                displaySets(n);
                break;
                
            case 4:
                printf("Exiting program.\n");
                exit(0);
                
            default:
                printf("Invalid choice!\n");
        }
    }
    
    return 0;
}