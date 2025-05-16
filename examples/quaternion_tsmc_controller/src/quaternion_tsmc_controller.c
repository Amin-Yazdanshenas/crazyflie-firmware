#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "controller_brescianini.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"
#include "quaternion_tsmc_controller.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT (float) 1.0f/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT (float) 1.0f/POS_UPDATE_RATE
#define PI 3.1415f


static const float gravity = 9.81f;  // [m/s^2]
static const float mass = 0.032f;  // [kg]  or 0.037
// static const float PI = 3.1415f;


static float lambda_q_x = 8.0f;
static float lambda_q_y = 8.0f;
static float lambda_q_z = 8.0f;
// static float K_q_x = 0.008f;
// static float K_q_y = 0.008f;
// static float K_q_z = 0.008f;
static float K_q_x = 400.0f;
static float K_q_y = 400.0f;
static float K_q_z = 500.0f;

// static float rho_q = 1.0f;
static float rho_q_x = 0.25f;
static float rho_q_y = 0.25f;
static float rho_q_z = 0.5f;




static float lambda_xi_x = 1.25f;
static float lambda_xi_y = 1.25f;
static float lambda_xi_z = 1.25f;
static float K_xi_x = 5.0f;
static float K_xi_y = 5.0f;
static float K_xi_z = 5.0f;
// static float rho_xi = 1.0f;
static float rho_xi_x = 1.0f;
static float rho_xi_y = 1.0f;
static float rho_xi_z = 1.0f;

// static float omega_scale = 1.0f;


// static struct vec s_q;
// static struct vec s_xi;
static struct vec Moment;
static float force;
static struct quat q_e;
static struct quat q_d;
static struct vec acc_d ;
static float counter;
// struct vec xi_dddot_des ;

// static struct mat33 Rd;
// static struct vec omega_dot_d;
// static struct vec omega_d;
// static struct vec omega_e;

// static struct mat33 J=
//     {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
//       {0.83e-6f, 16.6e-6f, 1.8e-6f},
//       {0.72e-6f, 1.8e-6f, 29.3e-6f}}};

static struct mat33 J=
    {{{16.6e-6f, 0.0f, 0.0f},
      {0.0f, 16.6e-6f, 0.0f},
      {0.0f, 0.0f, 29.3e-6f}}};

// Struct for logging position information
static bool isInit = false;
// static jmp_buf buf;


void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  while (1)
  {
    vTaskDelay(M2T(2000));

  }
}


void controllerOutOfTreeInit(void) {
  // s_q = vzero();
  // s_xi = vzero();
  q_e = qeye();
  q_d = qeye();
  Moment = vzero();
  force = 0.0f;
  acc_d = vzero();
  counter = 0.0f;
  // xi_dddot_des = vzero();
  // Rd = meye();
  // omega_dot_d = vzero();
  // omega_d= vzero();
  // omega_e= vzero();
  if (isInit) {
    return;
  }

  isInit = true;
}



void controllerOutOfTree(control_t *control,
                                 const setpoint_t *setpoint,
                                 const sensorData_t *sensors,
                                 const state_t *state,
                                 const stabilizerStep_t stabilizerStep) {

  // if (setjmp(buf)) {
  //   // Handle error
  //   DEBUG_PRINT("An error occurred. Execution halted.\n");
  //   return;
  // }
    static struct mat33 Rd;
    static struct vec omega_dot_d;
    static struct vec omega_d;
    static struct vec omega_e;

    struct vec omega = mkvec(
      radians(sensors->gyro.x),
      radians(sensors->gyro.y),
      radians(sensors->gyro.z)
      );

      // current attitude
    struct quat attitude = mkquat(
      state->attitudeQuaternion.x,
      state->attitudeQuaternion.y,
      state->attitudeQuaternion.z,
      state->attitudeQuaternion.w);
    attitude = qnormalize(attitude);
    struct mat33 R = quat2rotmat(attitude);

    float psi_d = radians(setpoint->attitude.yaw);
    float psi_dot_d = radians(setpoint->attitudeRate.yaw);
    float psi_ddot_d = 0.0f;

  //  Position Control Sliding Mode Control
  // if (RATE_DO_EXECUTE(POS_UPDATE_RATE, stabilizerStep)) {
  
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, stabilizerStep)) {



    // computing the position, velocity, acceleration and jerk errors
    struct vec xi_e = mkvec(state->position.x - setpoint->position.x ,
                            state->position.y - setpoint->position.y ,
                            state->position.z - setpoint->position.z);

    struct vec v_e = mkvec(state->velocity.x - setpoint->velocity.x,
                           state->velocity.y - setpoint->velocity.y,
                           state->velocity.z - setpoint->velocity.z);

    struct vec a_e = mkvec(state->acc.x - setpoint->acceleration.x,
                           state->acc.y - setpoint->acceleration.y,
                           state->acc.z - setpoint->acceleration.z);

    struct vec jerk_d = mkvec(setpoint->jerk.x ,setpoint->jerk.y ,setpoint->jerk.z);
    struct vec snap_d = mkvec(setpoint->snap.x ,setpoint->snap.y ,setpoint->snap.z);

    struct vec j_e = vzero();
    struct vec e1 = mkvec(1.0f,0.0f,0.0f);
    struct vec e2 = mkvec(0.0f,1.0f,0.0f);
    struct vec e3 = mkvec(0.0f,0.0f,1.0f);


    struct vec b1r = mkvec(cosf(psi_d),sinf(psi_d),0.0f);
    struct vec b1r_dot = mkvec(-sinf(psi_d)*psi_dot_d,cosf(psi_d)*psi_dot_d,0.0f);
    struct vec b1r_ddot =mkvec(-cosf(psi_d)*powf(psi_dot_d,2.0),-sinf(psi_d)*powf(psi_dot_d,2.0f),0.0f);

    // creating gain matrix 
    struct mat33 lambda_xi = mdiag(lambda_xi_x,lambda_xi_y,lambda_xi_z);
    struct mat33 lambda_v = mdiag(lambda_v_x,lambda_v_y,lambda_v_z);
    struct mat33 K_xi = mdiag(K_xi_x,K_xi_y,K_xi_z);
    struct mat33 rho_xi = mdiag(rho_xi_x,rho_xi_y,rho_xi_z);

    acc_d = mkvec(setpoint->acceleration.x,
                  setpoint->acceleration.y,
                  setpoint->acceleration.z);
    

    struct vec s_xi = vadd(v_e , mvmul(lambda_xi, xi_e));

    struct vec rho_mvmult_s_xi = mvmul(rho_xi,s_xi);
    
    struct vec kappa = vscl(mass,vadd4(acc_d,vneg(mvmul(lambda_xi, v_e)),vscl(gravity,e3),vneg(mvmul(K_xi,vtanhf(rho_mvmult_s_xi)))));
    
    struct vec Re1 = mvmul(R,e1);
    struct vec Re2 = mvmul(R,e2);
    struct vec Re3 = mvmul(R,e3);


    force = vdot(kappa,Re3);
    
// Trajectory generation for the attiude controller
    struct vec b3d = vnormalize(kappa);
    struct vec nu  = vcross(b3d,b1r);
    struct vec b2d = vnormalize(nu);
    struct vec b1d  = vcross(b2d,b3d);
    Rd = mcolumns(b1d,b2d,b3d);

    float fdot = mass * vdot(jerk_d, Re3);
    float fddot = mass * (vdot(snap_d, Re3) + vdot(vcross(omega, Re3),jerk_d));
    struct vec h_omega = vdiv(vsub(vscl(mass,jerk_d),vscl(fdot,Re3)),force);
    omega_d = mkvec(-1.0f*vdot(h_omega, Re1),
                    vdot(h_omega, Re1),
                    psi_dot_d*vdot(e3, Re3));

    struct vec h_alpha = vadd4(vscl(mass/force,snap_d),
                               vscl(-fddot/force,Re3),
                               vscl(-2.0f*fdot/f,vcross(omega,Re3)),
                               vneg(vcross(omega,vcross(omega,Re3))));
    omega_dot_d = mkvec(-1.0f*vdot(h_alpha, Re1),
                        vdot(h_alpha, Re1),
                        psi_ddot_d*vdot(e3, Re3));
  }

  
  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = 0.0f;
    control->torqueX =  0.0f;
    control->torqueY =  0.0f;
    control->torqueZ =  0.0f;
    counter = 0.0f;
  } else {

// //  Attitude Control sliding mode control
    if (counter < 10){
      Rd = meye();
      counter ++;
    } 
    // Rd = meye();
    omega_dot_d = vzero();
    omega_d = vzero();
    // force =0.0f;

    q_d = mat2quat(Rd);

    // creating gain matrix 
    struct mat33 lambda_q = mdiag(lambda_q_x,lambda_q_y,lambda_q_z);
    struct mat33 K_q = mdiag(K_q_x,K_q_y,K_q_z);   
    struct mat33 rho_q = mdiag(rho_q_x,rho_q_y,rho_q_z); 
    struct mat33 inv_lambda_omega_gamma_omega = mdiag(1.0f/(gamma_omega*lambda_omega_x),
                                                      1.0f/(gamma_omega*lambda_omega_y),
                                                      1.0f/(gamma_omega*lambda_omega_z));
    // computing the angular velocity and quaternion errors
    omega_e = vsub(omega, omega_d);
    q_e = qqmul(qinv(q_d),attitude);

    // computing the quaternion dot error
    struct quat q_e_dot = qscl(0.5,qqmul(q_e,mkquat(omega_e.x,omega_e.y,omega_e.z,0.0f)));
    struct vec q_e_vec = mkvec(q_e.x,q_e.y,q_e.z);
    struct vec q_e_dot_vec = mkvec(q_e_dot.x,q_e_dot.y,q_e_dot.z);
    // computing the sliding surface
    struct vec s_q = vadd3(vscl(sgnplus(q_e.w),q_e_vec),mvmul(lambda_q,signpow(q_e_vec,gamma_q)),mvmul(lambda_omega,signpow(omega_e,gamma_omega)));

    // computing the Moments

    Moment = vadd4(mvmul(J,omega_dot_d),
                   vcross(omega,mvmul(J,omega)),
                   vneg(veltmul(mvmul(mmul(J,inv_beta_q_gamma4),vpow(vabsf(omega_e),(1.0f-gamma_omega))),
                                veltmul(q_e_dot_vec,
                                        vadd(vrepeat(sgnplus(q_e.w)),mvmul(mscl(gamma_q,lambda_q),vpow(vabsf(q_e_vec),(lambda_q-1.0f))))))),
                   vneg(mvmul(K_q,vtanhf(mkvec(rho_q_x*s_q.x,rho_q_y*s_q.y,rho_q_z*s_q.z)))));
    
    //updating the Force and Moments
    control->thrustSi = force;
    control->torqueX = Moment.x;
    control->torqueY = Moment.y;
    control->torqueZ = Moment.z;
    // DEBUG_PRINT("Force: %f\n", (double)force);
    // DEBUG_PRINT("Moment: [%f, %f, %f]\n", (double)Moment.x, (double)Moment.y, (double)Moment.z);
  }

  // Setting up the Control Mode
  control->controlMode = controlModeForceTorque;
}

bool controllerOutOfTreeTest(void) {
  return true;
}

// Parameter setup
PARAM_GROUP_START(ctrlQSMC)
PARAM_ADD(PARAM_FLOAT, lambda_att_x, &lambda_q_x)
PARAM_ADD(PARAM_FLOAT, lambda_att_y, &lambda_q_y)
PARAM_ADD(PARAM_FLOAT, lambda_att_z, &lambda_q_z)
PARAM_ADD(PARAM_FLOAT, K_att_x, &K_q_x)
PARAM_ADD(PARAM_FLOAT, K_att_y, &K_q_y)
PARAM_ADD(PARAM_FLOAT, K_att_z, &K_q_z)
PARAM_ADD(PARAM_FLOAT, lambda_pos_x, &lambda_xi_x)
PARAM_ADD(PARAM_FLOAT, lambda_pos_y, &lambda_xi_y)
PARAM_ADD(PARAM_FLOAT, lambda_pos_z, &lambda_xi_z)
PARAM_ADD(PARAM_FLOAT, K_pos_x, &K_xi_x)
PARAM_ADD(PARAM_FLOAT, K_pos_y, &K_xi_y)
PARAM_ADD(PARAM_FLOAT, K_pos_z, &K_xi_z)
PARAM_ADD(PARAM_FLOAT, rho_xi_x, &rho_xi_x)
PARAM_ADD(PARAM_FLOAT, rho_xi_y, &rho_xi_y)
PARAM_ADD(PARAM_FLOAT, rho_xi_z, &rho_xi_z)
PARAM_ADD(PARAM_FLOAT, rho_q_x, &rho_q_x)
PARAM_ADD(PARAM_FLOAT, rho_q_y, &rho_q_y)
PARAM_ADD(PARAM_FLOAT, rho_q_z, &rho_q_z)
// PARAM_ADD(PARAM_FLOAT, omega_scale, &omega_scale)
PARAM_GROUP_STOP(ctrlQSMC)


// Logging info setup
LOG_GROUP_START(ctrlLog)
LOG_ADD(LOG_FLOAT, qe_x, &q_e.x)
LOG_ADD(LOG_FLOAT, qe_y, &q_e.y)
LOG_ADD(LOG_FLOAT, qe_z, &q_e.z)
LOG_ADD(LOG_FLOAT, qe_w, &q_e.w)
LOG_ADD(LOG_FLOAT, qd_x, &q_d.x)
LOG_ADD(LOG_FLOAT, qd_y, &q_d.y)
LOG_ADD(LOG_FLOAT, qd_z, &q_d.z)
LOG_ADD(LOG_FLOAT, qd_w, &q_d.w)
LOG_ADD(LOG_FLOAT, a_x, &acc_d.x)
LOG_ADD(LOG_FLOAT, a_y, &acc_d.y)
LOG_ADD(LOG_FLOAT, a_z, &acc_d.z)
// LOG_ADD(LOG_FLOAT, j_x, &xi_dddot_des.x)
// LOG_ADD(LOG_FLOAT, j_y, &xi_dddot_des.y)
// LOG_ADD(LOG_FLOAT, j_z, &xi_dddot_des.z)
// LOG_ADD(LOG_FLOAT, omega_ex, &omega_e.x)
// LOG_ADD(LOG_FLOAT, omega_ey, &omega_e.y)
// LOG_ADD(LOG_FLOAT, omega_ez, &omega_e.z)
// LOG_ADD(LOG_FLOAT, s_att_x, &s_q.x)
// LOG_ADD(LOG_FLOAT, s_att_y, &s_q.y)
// LOG_ADD(LOG_FLOAT, s_att_z, &s_q.z)
// LOG_ADD(LOG_FLOAT, s_pos_x, &s_xi.x)
// LOG_ADD(LOG_FLOAT, s_pos_y, &s_xi.y)
// LOG_ADD(LOG_FLOAT, s_pos_z, &s_xi.z)
LOG_ADD(LOG_FLOAT, moment_x, &Moment.x)
LOG_ADD(LOG_FLOAT, moment_y, &Moment.y)
LOG_ADD(LOG_FLOAT, moment_z, &Moment.z)
LOG_ADD(LOG_FLOAT, force, &force)
LOG_GROUP_STOP(ctrlLog)

