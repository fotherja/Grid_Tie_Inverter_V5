#include <pr.h>

#ifdef __cplusplus
extern "C"{
#endif

void pr_init(PR_t *s, float kp_set, float kr_set, float wi_set, float ts){
    s->kp = kp_set ;
    s->kr = kr_set ;
    s->wi = wi_set ;
    s->ts = ts ;
    s->output_of_feedback = 0.0f ;
    s->output_of_backward_integrator = 0.0f ;
    s->output_of_forward_integrator = 0.0f ;
    s->reference = 0.0f ;
}

float pr_calc(PR_t *s, float reference, float feedback, float wg){

    s->reference = reference;

    float error = reference - feedback ;
    float input_of_forward_integrator = 2 * s->wi * s->kr * error - s->output_of_feedback;
    // Forward integrator :
    s->output_of_forward_integrator += s->ts *  input_of_forward_integrator;

    // Backward integrator:
    s->output_of_backward_integrator += s->ts * s->output_of_forward_integrator * wg * wg ;

    s->output_of_feedback = s->output_of_backward_integrator + 2 * s->wi * s->output_of_forward_integrator ;

    return s->output_of_forward_integrator + s->kp * error;
}

#ifdef __cplusplus
}
#endif
