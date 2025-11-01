#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>  // For DBL_MAX
#include <iomanip> // For formatting output (setw, setprecision)

// Node structure for the Binary Search Tree
struct Node {
    char* key;
    struct Node *left, *right;
};

// Global tables
double** e; // e[i][j] = min cost for keys i to j
double** w; // w[i][j] = sum of probs for keys i to j
int** root; // root[i][j] = root of the optimal tree for keys i to j

/**
 * 1. Build the OBST tables
 */
void compute_obst(int n, double p[], double q[], char* keys[]) {
    // ---- Allocate Memory for Tables ----
    // e and w need [1..n+1][0..n]
    // root needs [1..n][1..n]
    
    e = (double**)malloc((n + 2) * sizeof(double*));
    w = (double**)malloc((n + 2) * sizeof(double*));
    for (int i = 0; i <= n + 1; i++) {
        e[i] = (double*)malloc((n + 1) * sizeof(double));
        w[i] = (double*)malloc((n + 1) * sizeof(double));
    }

    root = (int**)malloc((n + 1) * sizeof(int*));
    for (int i = 0; i <= n; i++) {
        root[i] = (int*)malloc((n + 1) * sizeof(int));
    }

    // 1. Initialize base cases (subtrees of length 0)
    for (int i = 1; i <= n + 1; ++i) {
        e[i][i - 1] = q[i - 1];
        w[i][i - 1] = q[i - 1];
    }

    // 2. Loop by chain length, l (from 1 to n)
    for (int l = 1; l <= n; ++l) {
        // Loop for all possible subtrees of length l
        for (int i = 1; i <= n - l + 1; ++i) {
            int j = i + l - 1;

            // Initialize cost to infinity
            e[i][j] = DBL_MAX;
            
            // Calculate weight w[i, j]
            w[i][j] = w[i][j - 1] + p[j] + q[j];

            // 3. Find the optimal root (r) for the range [i, j]
            for (int r = i; r <= j; ++r) {
                // Cost = cost(left subtree) + cost(right subtree) + weight(current)
                double cost = e[i][r - 1] + e[r + 1][j] + w[i][j];

                if (cost < e[i][j]) {
                    e[i][j] = cost;
                    root[i][j] = r;
                }
            }
        }
    }
}

/**
 * 2. Helper to print a double 2D table
 */
void print_double_table(char* title, int n, double** table) {
    printf("--- %s Table ---\n", title);
    printf("     ");
    for (int j = 0; j <= n; j++) {
        printf("   j=%-4d", j);
    }
    printf("\n----");
    for (int j = 0; j <= n; j++) {
        printf("+---------");
    }
    printf("+\n");

    for (int i = 1; i <= n + 1; i++) {
        printf("i=%-2d |", i);
        for (int j = 0; j <= n; j++) {
            if (j < i - 1) {
                printf("    -    |"); // Print empty
            } else {
                printf(" %-8.2f|", table[i][j]);
            }
        }
        printf("\n");
    }
    printf("\n");
}

/**
 * 3. Helper to print the int 2D root table
 */
void print_root_table(char* title, int n, int** table, char* keys[]) {
    printf("--- %s Table (Key Index) ---\n", title);
    printf("     ");
    for (int j = 1; j <= n; j++) {
        printf("   j=%-4d", j);
    }
    printf("\n----");
    for (int j = 1; j <= n; j++) {
        printf("+---------");
    }
    printf("+\n");

    for (int i = 1; i <= n; i++) {
        printf("i=%-2d |", i);
        for (int j = 1; j <= n; j++) {
            if (j < i) {
                printf("    -    |"); // Print empty
            } else {
                printf(" %-8d|", table[i][j]);
            }
        }
        printf("\n");
    }
    printf("\n");
}

/**
 * 4. Construct the tree from the root table
 */
struct Node* construct_tree(int i, int j, char* keys[]) {
    if (i > j) {
        return NULL; // Base case: represents a dummy key
    }

    // Get the root of this subtree
    int r = root[i][j];

    // Create a new node for key k_r
    struct Node* node = (struct Node*)malloc(sizeof(struct Node));
    node->key = keys[r];

    // Recursively build the left and right subtrees
    node->left = construct_tree(i, r - 1, keys);
    node->right = construct_tree(r + 1, j, keys);

    return node;
}

/**
 * 5. Print the tree structure (In-order traversal with indentation)
 */
void print_tree_structure(struct Node* node, int indent) {
    if (node == NULL) {
        return;
    }
    // Recurse on right child first for a more visual tree
    print_tree_structure(node->right, indent + 4);

    for (int i = 0; i < indent; i++) {
        printf(" ");
    }
    printf("%s\n", node->key);

    print_tree_structure(node->left, indent + 4);
}

/**
 * 6. Search for a key in the OBST
 */
int search_obst(struct Node* node, char* key) {
    // 1. Base case: Key not found or tree is empty
    if (node == NULL) {
        return 0; // Not found
    }

    // 2. Key found
    int cmp = strcmp(key, node->key);
    if (cmp == 0) {
        return 1; // Found
    }

    // 3. Recurse left
    if (cmp < 0) {
        return search_obst(node->left, key);
    }
    // 4. Recurse right
    else {
        return search_obst(node->right, key);
    }
}

/**
 * 7. Free all allocated memory
 */
void free_memory(int n, struct Node* root_node) {
    // Free tables
    for (int i = 0; i <= n + 1; i++) {
        free(e[i]);
        free(w[i]);
    }
    free(e);
    free(w);

    for (int i = 0; i <= n; i++) {
        free(root[i]);
    }
    free(root);

    // Free tree (simple post-order traversal)
    if (root_node != NULL) {
        free_memory(n, root_node->left);
        free_memory(n, root_node->right);
        free(root_node);
    }
}


/**
 * 8. Main execution
 */
int main() {
    // Standard 4-key example
    int n = 4;

    // Keys (using 1-based indexing, keys[0] is a dummy)
    char* keys[] = {"", "case", "else", "if", "while"};
    
    // Successful search probabilities (p[0] is a dummy)
    double p[] = {0.0, 0.04, 0.06, 0.08, 0.02};
    
    // Unsuccessful search probabilities (q[0] to q[n])
    double q[] = {0.06, 0.06, 0.06, 0.06, 0.05};

    printf("========[1] OBST CONSTRUCTION ANALYSIS ========\n");
    printf("Number of Keys (n): %d\n\n", n);

    // --- [1] Compute Tables ---
    compute_obst(n, p, q, keys);

    // --- [2] Print Analysis Tables ---
    print_double_table("Expected Cost (e)", n, e);
    print_double_table("Weight (w)", n, w);
    print_root_table("Root", n, root, keys);

    // --- [3] Print Final Cost ---
    printf("========[2] FINAL COST AND TREE ========\n");
    printf("Minimum Expected Search Cost: %.2f\n\n", e[1][n]);

    // --- [4] Construct and Print Tree ---
    printf("--- Constructed Optimal Tree Structure ---\n");
    printf("(Printed sideways, root is at the left)\n\n");
    struct Node* obst_root = construct_tree(1, n, keys);
    print_tree_structure(obst_root, 0);

    // --- [5] Verification ---
    printf("\n========[3] VERIFICATION AND METRICS ========\n");
    
    char* key_to_find_success = "if";
    int found = search_obst(obst_root, key_to_find_success);
    printf("Searching for \"%s\": %s\n", key_to_find_success, found ? "FOUND (Successful)" : "NOT FOUND");

    char* key_to_find_fail = "char";
    found = search_obst(obst_root, key_to_find_fail);
    printf("Searching for \"%s\": %s\n", key_to_find_fail, found ? "FOUND" : "NOT FOUND (Unsuccessful)");
    
    printf("\nVerification: Tree constructed and search complete.\n");

    // --- [6] Cleanup ---
    // Pass n=0 to free_memory when called from main to avoid re-freeing tables
    free_memory(n, obst_root); 

    return 0;
}