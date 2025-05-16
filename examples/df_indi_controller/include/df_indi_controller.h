#pragma once


#include "math3d.h"

#include "filter.h"
#include <math.h>
#include "stabilizer_types.h"
#include "filter.h"
#include "math3d.h"
#include "log.h"
#include "param.h"
#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT (float) 1.0f/ATTITUDE_UPDATE_RATE
#define SMOOTHING_ALPHA 0.1f


// these parameters are used in the filtering of the angular acceleration [Hz]
#define STABILIZATION_INDI_FILT_CUTOFF 8.0f

// the yaw sometimes requires more filtering
#define STABILIZATION_INDI_FILT_CUTOFF_R STABILIZATION_INDI_FILT_CUTOFF

// Control effectiveness coefficients values Volodscoi
#define STABILIZATION_INDI_G1_P 0.0066146f
#define STABILIZATION_INDI_G1_Q 0.0052125f
#define STABILIZATION_INDI_G1_R 0.001497f
#define STABILIZATION_INDI_G2_R 0.000043475f

// Control effectiveness coefficients values Max Kemmeren, these will become the new ones, not finalized yet
// #define STABILIZATION_INDI_G1_P 0.0032502f
// #define STABILIZATION_INDI_G1_Q 0.0027555f
// #define STABILIZATION_INDI_G1_R 0.00068154f
// #define STABILIZATION_INDI_G2_R 0.00001725f

//Proportional gains inner INDI, attitude error
#define STABILIZATION_INDI_REF_ERR_P 5.0f
#define STABILIZATION_INDI_REF_ERR_Q 5.0f
#define STABILIZATION_INDI_REF_ERR_R 5.0f

//Derivative gains inner INDI, attitude rate error
#define STABILIZATION_INDI_REF_RATE_P 24.0f
#define STABILIZATION_INDI_REF_RATE_Q 24.0f
#define STABILIZATION_INDI_REF_RATE_R 24.0f

// Actuator model coefficient
#define STABILIZATION_INDI_ACT_DYN_P 0.03149f
#define STABILIZATION_INDI_ACT_DYN_Q 0.03149f
#define STABILIZATION_INDI_ACT_DYN_R 0.03149f

static float thrust_threshold = 300.0f;
static float bound_control_input = 32000.0f;
static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;
struct FloatRates body_rates;
static vector_t refOuterINDI;				// Reference values from outer loop INDI
static bool outerLoopActive = true ; 		// if 1, outer loop INDI is activated
/**
 * @brief angular rates
 * @details Units: rad/s */
 struct FloatRates {
    float p; ///< in rad/s
    float q; ///< in rad/s
    float r; ///< in rad/s
  };
  
  struct ReferenceSystem {
    float err_p;
    float err_q;
    float err_r;
    float rate_p;
    float rate_q;
    float rate_r;
  };
  
  struct IndiVariables {
    float thrust;
    struct FloatRates angular_accel_ref;
    struct FloatRates du;
    struct FloatRates u_in;
    struct FloatRates u_act_dyn;
    float rate_d[3];
  
    Butterworth2LowPass u[3];
    Butterworth2LowPass rate[3];
    struct FloatRates g1;
    float g2;
  
    struct ReferenceSystem reference_acceleration;
    struct FloatRates act_dyn;
    float filt_cutoff;
    float filt_cutoff_r;
  };

static struct IndiVariables indi = {
    .g1 = {STABILIZATION_INDI_G1_P, STABILIZATION_INDI_G1_Q, STABILIZATION_INDI_G1_R},
    .g2 = STABILIZATION_INDI_G2_R,
    .reference_acceleration = {
            STABILIZATION_INDI_REF_ERR_P,
            STABILIZATION_INDI_REF_ERR_Q,
            STABILIZATION_INDI_REF_ERR_R,
            STABILIZATION_INDI_REF_RATE_P,
            STABILIZATION_INDI_REF_RATE_Q,
            STABILIZATION_INDI_REF_RATE_R
    },
    .act_dyn = {STABILIZATION_INDI_ACT_DYN_P, STABILIZATION_INDI_ACT_DYN_Q, STABILIZATION_INDI_ACT_DYN_R},
    .filt_cutoff = STABILIZATION_INDI_FILT_CUTOFF,
    .filt_cutoff_r = STABILIZATION_INDI_FILT_CUTOFF_R,
};

static inline float sechf(float x) {
    return 1.0f / coshf(x);
}

static inline float signf(float a){
    return isnan(a / fabsf(a)) ? 0.0f : a / fabsf(a);
}

static inline float minf(float a, float b) {
    return (a < b) ? a : b;
}

static inline float maxf(float a, float b) {
    return (a > b) ? a : b;
}


static inline struct vec signpow(struct vec v,float m) {
    return mkvec(signf(v.x)*powf(fabsf(v.x),m), signf(v.y)*powf(fabsf(v.y),m), signf(v.z)*powf(fabsf(v.z),m));
}

static inline struct vec vsign(struct vec v) {
    return mkvec(signf(v.x), signf(v.y), signf(v.z));
}


static inline struct vec psifunc(float rho, struct vec s) {
    return mkvec((expf(rho*s.x)-1)/(expf(rho*s.x)+1), (expf(rho*s.y)-1)/(expf(rho*s.y)+1), (expf(rho*s.z)-1)/(expf(rho*s.z)+1));
}


static inline float sgnplus(float a){
    return isnan(a / fabsf(a)) ? 1.0f : a / fabsf(a);
}


static inline struct vec vpow(struct vec v,float a){
    return mkvec(powf(v.x,a), powf(v.y,a), powf(v.z,a));
}


//  function vec = mvee(skewsymmetric)
//      The mvee function maps a skew-symmetric
//      matrix to a vector
static inline struct vec mvee(struct mat33 m) {
    return mkvec(m.m[2][1],m.m[0][2],m.m[1][0]);
}

// function m = vhat(vec)
// %   The hat function maps a vector in R^3 to its
// %   skew symmetric matrix in R^(3x3)

static inline struct mat33 vhat(struct vec v) {
	struct mat33 m = mzero();
	m.m[0][1] = -v.z;
	m.m[0][2] =  v.y;
	m.m[1][0] =  v.z;
	m.m[1][2] = -v.x;
    m.m[2][0] = -v.y;
	m.m[2][1] =  v.x;
	return m;
}

static inline struct quat qscl(float s, struct quat q){
    return mkquat(s*q.x,s*q.y,s*q.z,s*q.w);
}

static inline struct vec vtanhf(struct vec v){
    return mkvec(tanhf(v.x), tanhf(v.y), tanhf(v.z));
}

static inline struct vec vsechf(struct vec v){
    return mkvec(sechf(v.x), sechf(v.y), sechf(v.z));
}

static inline struct vec vsech2f(struct vec v){
    return mkvec(sechf(v.x)*sechf(v.x), sechf(v.y)*sechf(v.y), sechf(v.z)*sechf(v.z));
}

static inline float calculateKdot( float K,float Gamma, float s, float epsilon) {
    if (K > 1e-6f) {
        return Gamma * fabsf(s) * signf(fabsf(s) - epsilon);
    } else {
        return 1e-6f;
    }
}

static inline struct vec veltmul3(struct vec a, struct vec b, struct vec c) {
	return mkvec(a.x * b.x * c.x, a.y * b.y * c.y, a.z * b.z * c.z);
}


static inline struct vec vnormalize1diff(struct vec a, struct vec a_dot) {
    struct vec out = vsub(vdiv(a_dot,vmag(a)),vdiv(vscl(vdot(a,a_dot),a),powf(vmag(a),3.0f)));
    return out;
}

static inline struct vec vnormalize2diff(struct vec a, struct vec a_dot, struct vec a_ddot) {
	struct vec out = vadd3(vdiv(a_ddot,vmag(a)),
                           vscl(-2.0f*vdot(a,a_dot)/(powf(vmag(a),3.0f)),a_dot),
                           vscl(-1.0f*(powf(vmag(a_dot),2.0f)+vdot(a,a_ddot))/(powf(vmag(a),3.0f)) + 3.0f*powf(vdot(a,a_dot),2.0f)/powf(vmag(a),5.0f),a));
    return out;
}


static inline void float_rates_zero(struct FloatRates *fr) {
	fr->p = 0.0f;
	fr->q = 0.0f;
	fr->r = 0.0f;
}

void indi_init_filters(void)
{
	// tau = 1/(2*pi*Fc)
	float tau = 1.0f / (2.0f * M_PI_F * indi.filt_cutoff);
	float tau_r = 1.0f / (2.0f * M_PI_F * indi.filt_cutoff_r);
	float tau_axis[3] = {tau, tau, tau_r};
	float sample_time = 1.0f / ATTITUDE_RATE;
	// Filtering of gyroscope and actuators
	for (int8_t i = 0; i < 3; i++) {
		init_butterworth_2_low_pass(&indi.u[i], tau_axis[i], sample_time, 0.0f);
		init_butterworth_2_low_pass(&indi.rate[i], tau_axis[i], sample_time, 0.0f);
	}
}

/**
 * @brief Update butterworth filter for p, q and r of a FloatRates struct
 *
 * @param filter The filter array to use
 * @param new_values The new values
 */
static inline void filter_pqr(Butterworth2LowPass *filter, struct FloatRates *new_values)
{
	update_butterworth_2_low_pass(&filter[0], new_values->p);
	update_butterworth_2_low_pass(&filter[1], new_values->q);
	update_butterworth_2_low_pass(&filter[2], new_values->r);
}

/**
 * @brief Caclulate finite difference form a filter array
 * The filter already contains the previous values
 *
 * @param output The output array
 * @param filter The filter array input
 */
static inline void finite_difference_from_filter(float *output, Butterworth2LowPass *filter)
{
	for (int8_t i = 0; i < 3; i++) {
		output[i] = (filter[i].o[0] - filter[i].o[1]) * ATTITUDE_RATE;
	}
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

