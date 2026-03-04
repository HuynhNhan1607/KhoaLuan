#ifndef __LPF_H__
#define __LPF_H__

#include <stdbool.h>
#include "bno055.h"

// Maximum filter order supported
#define LPF_MAX_ORDER 10

typedef struct
{
    int order;                                       // Filter order
    bool initialized;                                // Initialization flag
    float a[LPF_MAX_ORDER + 1];                      // Feedback coefficients (a0 is assumed to be 1.0)
    float b[LPF_MAX_ORDER + 1];                      // Feedforward coefficients
    bno055_vec3_t input_history[LPF_MAX_ORDER + 1];  // Input history
    bno055_vec3_t output_history[LPF_MAX_ORDER + 1]; // Output history
} lpf_filter_t;

/**
 * @brief Initialize a low-pass filter with specified order and coefficients
 *
 * @param filter Pointer to filter structure
 * @param order Filter order (1 to LPF_MAX_ORDER)
 * @param a_coeff Array of feedback coefficients (a0 is assumed to be 1.0)
 * @param b_coeff Array of feedforward coefficients
 * @return true if initialization successful
 * @return false if invalid parameters
 */
bool lpf_init(lpf_filter_t *filter, int order, float *a_coeff, float *b_coeff);

/**
 * @brief Initialize a default second-order low-pass filter
 *
 * @param filter Pointer to filter structure
 * @return true if initialization successful
 * @return false if invalid parameters
 */
bool lpf_init_default_second_order(lpf_filter_t *filter);

/**
 * @brief Apply the low-pass filter to a 3D vector
 *
 * @param filter Pointer to filter structure
 * @param vec Pointer to vector to filter (input and output)
 */
void lpf_apply_vec3(lpf_filter_t *filter, bno055_vec3_t *vec);

/**
 * @brief Reset filter state
 *
 * @param filter Pointer to filter structure
 */
void lpf_reset(lpf_filter_t *filter);

#endif // __LPF_H__