#include <stdio.h>
#include <stdlib.h>

// Node structure for linked list
typedef struct Node {
    int data;
    struct Node* next;
    struct Set* set;  // Pointer to the set this node belongs to
} Node;

// Set structure with head and tail pointers
typedef struct Set {
    Node* head;
    Node* tail;
} Set;

// MakeSet: Create a singleton set containing element x
Set* MakeSet(int x) {
    Set* newSet = (Set*)malloc(sizeof(Set));
    Node* newNode = (Node*)malloc(sizeof(Node));
    
    newNode->data = x;
    newNode->next = NULL;
    newNode->set = newSet;
    
    newSet->head = newNode;
    newSet->tail = newNode;
    
    return newSet;
}

// Find: Return the representative (head) of the set containing x
int Find(Node* x) {
    if (x == NULL || x->set == NULL) {
        return -1;  // Invalid node
    }
    return x->set->head->data;
}

// Union: Merge sets containing x and y
void Union(Node* x, Node* y) {
    if (x == NULL || y == NULL || x->set == NULL || y->set == NULL) {
        return;
    }
    
    Set* setX = x->set;
    Set* setY = y->set;
    
    // If already in same set, do nothing
    if (setX == setY) {
        return;
    }
    
    // Append setY to setX
    setX->tail->next = setY->head;
    setX->tail = setY->tail;
    
    // Update set pointers for all nodes in setY
    Node* current = setY->head;
    while (current != NULL) {
        current->set = setX;
        current = current->next;
    }
    
    // Free the old set structure (not the nodes)
    free(setY);
}

// Helper function to print a set
void PrintSet(Set* set) {
    if (set == NULL) {
        printf("Empty set\n");
        return;
    }
    
    Node* current = set->head;
    printf("{ ");
    while (current != NULL) {
        printf("%d ", current->data);
        current = current->next;
    }
    printf("}\n");
}

// Main function to demonstrate operations
int main() {
    // Create individual sets
    Set* set1 = MakeSet(1);
    Set* set2 = MakeSet(2);
    Set* set3 = MakeSet(3);
    Set* set4 = MakeSet(4);
    
    printf("Initial sets:\n");
    printf("Set 1: "); PrintSet(set1);
    printf("Set 2: "); PrintSet(set2);
    printf("Set 3: "); PrintSet(set3);
    printf("Set 4: "); PrintSet(set4);
    
    // Find representatives
    printf("\nFind operations:\n");
    printf("Representative of 1: %d\n", Find(set1->head));
    printf("Representative of 2: %d\n", Find(set2->head));
    
    // Union operations
    printf("\nPerforming Union(1, 2):\n");
    Union(set1->head, set2->head);
    printf("Set 1: "); PrintSet(set1);
    printf("Representative of 2: %d\n", Find(set2->head));
    
    printf("\nPerforming Union(3, 4):\n");
    Union(set3->head, set4->head);
    printf("Set 3: "); PrintSet(set3);
    
    printf("\nPerforming Union(1, 3):\n");
    Union(set1->head, set3->head);
    printf("Set 1: "); PrintSet(set1);
    printf("Representative of 4: %d\n", Find(set4->head));
    
    return 0;
}