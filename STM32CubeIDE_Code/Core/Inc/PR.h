#ifndef PR_H
#define PR_H

#ifdef __cplusplus
extern "C"{
#endif

typedef struct{
    float kp ;
    float kr ;
    float wi ;
    float reference ;
    float ts ;
    float output_of_backward_integrator ;
    float output_of_feedback ;
    float output_of_forward_integrator ;
    float last_input_of_forward_integrator ;
}PR_t;

/**
 * @brief Initialize the PR_t struct for a PR controller
 *
 * @param s pointer to PR_t struct
 * @param kp_set Value of kp to be set
 * @param kr_set Value of kr to be set, which is the gain for fundamental signal
 * @param wi_set Vaule of wi to be set, which determines the bandwidth at fundamental frequency, in rad/s
 * @param ts sampling period to be set, in seconds
 */
void pr_init(PR_t *s, float kp, float kr, float wi, float ts) ;

/**
 * @brief Second Order Generalized Integratior(SOGI) implementation for frequency adaptive resonance controller
 *
 * @param s pointer to PR_t struct
 * @param reference pointer to PR_t struct
 * @param feedback feedback signal
 * @param wg fundamental angular frequency, in rad/s.
 * @return output of PR controller
 */
float pr_calc(PR_t *s, float reference, float feedback, float wg);

#ifdef __cplusplus
}
#endif

#endif
