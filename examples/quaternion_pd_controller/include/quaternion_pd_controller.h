#pragma once


#include "math3d.h"


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