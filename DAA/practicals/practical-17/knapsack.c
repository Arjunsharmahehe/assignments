#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Helper function to find the maximum of two integers
int max(int a, int b) {
    return (a > b) ? a : b;
}

/**
 * 1. Build the Knapsack DP table
 * This function allocates and returns the 2D table K.
 */
int** compute_knapsack_table(int V[], int WT[], int n, int W) {
    int i, w;

    // --- Allocate Memory for Tables ---
    // Create a 2D table K[0..n][0..W]
    int** K = (int**)malloc((n + 1) * sizeof(int*));
    for (i = 0; i <= n; i++) {
        K[i] = (int*)malloc((W + 1) * sizeof(int));
    }

    // --- 1. Initialize base cases ---
    for (i = 0; i <= n; i++) {
        for (w = 0; w <= W; w++) {
            if (i == 0 || w == 0)
                K[i][w] = 0;
            // --- 2. Fill the table ---
            // Check if the current item (i) fits
            // Note: We use WT[i-1] and V[i-1] because our arrays are 0-indexed
            // but our loop logic (i=1 to n) matches the 1-indexed pseudocode.
            else if (WT[i - 1] <= w) {
                // --- 3. Core DP step ---
                K[i][w] = max(V[i - 1] + K[i - 1][w - WT[i - 1]], K[i - 1][w]);
            } else {
                // Item is too heavy, cannot include it
                K[i][w] = K[i - 1][w];
            }
        }
    }

    return K; // Return the computed table
}

/**
 * 2. Helper to print the dynamic programming table
 */
void print_table(int** K, int n, int W) {
    printf("--- Dynamic Programming Table (K) ---\n");
    printf("     ");
    for (int w = 0; w <= W; w++) {
        printf(" w=%-3d", w);
    }
    printf("\n----");
    for (int w = 0; w <= W; w++) {
        printf("+------");
    }
    printf("+\n");

    for (int i = 0; i <= n; i++) {
        printf("i=%-2d |", i);
        for (int w = 0; w <= W; w++) {
            printf(" %-5d|", K[i][w]);
        }
        printf("\n");
    }
    printf("\n");
}

/**
 * 3. Backtrack to find selected items
 */
void find_selected_items(int** K, int V[], int WT[], int n, int W) {
    int i = n;
    int w = W;
    int total_value = 0;
    int total_weight = 0;

    printf("--- Selected Items (Backtracking) ---\n");

    // Start from the bottom-right corner
    while (i > 0 && w > 0) {
        // Check if item 'i' was included
        // We use K[i][w] != K[i-1][w] to check
        if (K[i][w] != K[i - 1][w]) {
            // Item 'i' was included. (Arrays are 0-indexed, so we print i-1)
            printf("  - Item %d (Value: %d, Weight: %d)\n", i, V[i - 1], WT[i - 1]);
            
            total_value += V[i - 1];
            total_weight += WT[i - 1];

            // Move "up and left" by subtracting the item's weight
            w = w - WT[i - 1];
            i = i - 1;
        } else {
            // Item 'i' was not included.
            // Move "up" one row.
            i = i - 1;
        }
    }
    
    printf("\n--- Verification ---\n");
    printf("Total Value of Selected Items: %d\n", total_value);
    printf("Total Weight of Selected Items: %d (Capacity was %d)\n", total_weight, W);
}

/**
 * 4. Free all allocated memory
 */
void free_memory(int** K, int n) {
    for (int i = 0; i <= n; i++) {
        free(K[i]);
    }
    free(K);
}

/**
 * 5. Main execution
 */
int main() {
    // --- Input Data ---
    int n = 4; // Number of items
    int W = 7; // Knapsack Capacity
    
    // Values (1-indexed in logic, 0-indexed in array)
    int V[] = {1, 4, 5, 7};
    
    // Weights (1-indexed in logic, 0-indexed in array)
    int WT[] = {1, 3, 4, 5};

    printf("========[1] 0/1 KNAPSACK ANALYSIS ========\n");
    printf("Number of Items (n): %d\n", n);
    printf("Knapsack Capacity (W): %d\n\n", W);

    // --- [1] Compute Table ---
    int** K = compute_knapsack_table(V, WT, n, W);

    // --- [2] Print Analysis Table ---
    print_table(K, n, W);

    // --- [3] Print Final Value ---
    printf("========[2] FINAL VALUE AND ITEMS ========\n");
    printf("Maximum Value Achievable: %d\n\n", K[n][W]);

    // --- [4] Verification ---
    find_selected_items(K, V, WT, n, W);
    
    // --- [5] Cleanup ---
    free_memory(K, n);

    return 0;
}
