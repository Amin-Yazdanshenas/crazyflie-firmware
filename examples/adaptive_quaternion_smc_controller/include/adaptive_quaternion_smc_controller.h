#pragma once


#include "math3d.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT (float) 1.0f/ATTITUDE_UPDATE_RATE
#define SMOOTHING_ALPHA 0.1f
#include <math.h>

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

static inline float calculateKdot( float K,float Gamma, float s,float rho, float epsilon, float mu, float K0) {
    if (K > K0) {
        return Gamma * fabsf(s) * tanhf(fabsf(s)*rho - epsilon);
    } else {
        return mu;
    }
}

static inline float calculateKddot( float K,float Gamma, float s,float sdot, float rho,float epsilon, float mu, float K0) {
    if (K > K0) {
        return Gamma * signf(s)*sdot *( tanhf(fabsf(s)*rho - epsilon) +fabsf(s)*rho * sechf(fabsf(s)*rho - epsilon)* sechf(fabsf(s)*rho - epsilon));
    } else {
        return 0;
    }
}

static inline float K_windup( float K,float upperbound, float K0) {
    if (K >= upperbound) {
        return K0;
    } else {
        return K;
    }
}


static inline struct vec veltmul3(struct vec a, struct vec b, struct vec c) {
	return mkvec(a.x * b.x * c.x, a.y * b.y * c.y, a.z * b.z * c.z);
}
static inline struct vec vsub3(struct vec a, struct vec b, struct vec c ,struct vec d) {
	return mkvec(a.x - b.x - c.x - d.x, a.y - b.y - c.y - d.y, a.z - b.z - c.z -d.z);
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

static inline struct mat33 mmul3(struct mat33 a, struct mat33 b, struct mat33 c) {
    // First multiply a and b
    struct mat33 ab;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float accum = 0;
            for (int k = 0; k < 3; ++k) {
                accum += a.m[i][k] * b.m[k][j];
            }
            ab.m[i][j] = accum;
        }
    }

    // Now multiply the result with c: (ab * c)
    struct mat33 abc;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float accum = 0;
            for (int k = 0; k < 3; ++k) {
                accum += ab.m[i][k] * c.m[k][j];
            }
            abc.m[i][j] = accum;
        }
    }

    return abc;
}
static inline struct vec compute_derivative(struct vec current,struct vec prev) {

    // Calculate dxdt
    struct vec dxdt = vscl(ATTITUDE_UPDATE_DT, vsub(current, prev));

    return dxdt;
}