#include <stdio.h>
#include <stdlib.h>

// Structure to store item data
typedef struct {
    int value;
    int weight;
    double ratio;
    int original_index; // To track which item it was
} Item;

/**
 * 1. Comparison function for qsort
 * Sorts items in descending order of their ratio.
 */
int compareItems(const void* a, const void* b) {
    Item* itemA = (Item*)a;
    Item* itemB = (Item*)b;
    
    // Sort by ratio (highest to lowest)
    if (itemB->ratio > itemA->ratio) {
        return 1;
    } else if (itemB->ratio < itemA->ratio) {
        return -1;
    } else {
        return 0; // Ratios are equal
    }
}

/**
 * 2. Function to compute the maximum fractional knapsack value
 */
void compute_fractional_knapsack(int V[], int WT[], int n, int W) {
    
    // --- 1. Create Item array ---
    Item* items = (Item*)malloc(n * sizeof(Item));
    
    for (int i = 0; i < n; i++) {
        items[i].value = V[i];
        items[i].weight = WT[i];
        items[i].ratio = (double)V[i] / WT[i];
        items[i].original_index = i;
    }

    // --- 2. Sort items by ratio ---
    // 
    qsort(items, n, sizeof(Item), compareItems);

    // --- 3. Initialize knapsack variables ---
    double total_value = 0.0;
    int current_weight = 0;
    // Array to store the fraction of each item taken (using original index)
    double* fractions_taken = (double*)calloc(n, sizeof(double));

    printf("--- Greedy Selection Process (Sorted by Ratio) ---\n");

    // --- 4. Loop through sorted items and fill the knapsack ---
    for (int i = 0; i < n; i++) {
        Item item = items[i];

        if (current_weight + item.weight <= W) {
            // Case 1: The entire item fits
            current_weight += item.weight;
            total_value += item.value;
            fractions_taken[item.original_index] = 1.0; // Took 100%
            
            printf("  - Took Item %d (Ratio: %.2f) [100%%]\n", 
                   item.original_index + 1, item.ratio);
            
        } else {
            // Case 2: Only a fraction of the item fits
            int remaining_weight = W - current_weight;
            
            // Calculate the fraction
            double fraction = (double)remaining_weight / item.weight;
            
            // Add the value of that fraction
            total_value += item.value * fraction;
            current_weight += remaining_weight; // Knapsack is now full
            fractions_taken[item.original_index] = fraction;

            printf("  - Took Item %d (Ratio: %.2f) [%.0f%%]\n", 
                   item.original_index + 1, item.ratio, fraction * 100);

            // Knapsack is full, so we can stop
            break;
        }
    }

    printf("\n========[2] FINAL VALUE AND ITEMS ========\n");
    printf("Maximum Value Achievable: %.2f\n\n", total_value);
    
    printf("--- Included Items & Fractions ---\n");
    for(int i=0; i<n; i++) {
        if(fractions_taken[i] > 0) {
            printf("  - Item %d: %.0f%% (Value: %d, Weight: %d)\n", 
                   i + 1, fractions_taken[i] * 100, V[i], WT[i]);
        }
    }

    // --- 5. Cleanup ---
    free(items);
    free(fractions_taken);
}

/**
 * 3. Main execution
 */
int main() {
    // --- Input Data ---
    int n = 3; // Number of items
    int W = 50; // Knapsack Capacity
    
    int V[] = {60, 100, 120};
    int WT[] = {10, 20, 30};

    printf("========[1] FRACTIONAL KNAPSACK ANALYSIS ========\n");
    printf("Number of Items (n): %d\n", n);
    printf("Knapsack Capacity (W): %d\n\n", W);
    
    printf("--- Original Items ---\n");
    for(int i=0; i<n; i++) {
        printf("  - Item %d (Value: %d, Weight: %d, Ratio: %.2f)\n", 
               i+1, V[i], WT[i], (double)V[i]/WT[i]);
    }
    printf("\n");

    // --- [1] Compute ---
    compute_fractional_knapsack(V, WT, n, W);

    return 0;
}