/**
 * Authored by Michael Hamer (http://www.mikehamer.info), November 2016.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ============================================================================
 *
 * The controller implemented in this file is based on the paper:
 *
 * "Quaternion-Based Sliding Mode Control for Six Degrees of Freedom Flight
Control of Quadrotors"
 * https://arxiv.org/pdf/2403.10934
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @article{yazdanshenas2024quaternion,
        title={Quaternion-Based Sliding Mode Control for Six Degrees of Freedom Flight Control of Quadrotors},
        author={Yazdanshenas, Amin and Faieghi, Reza},
        journal={arXiv preprint arXiv:2403.10934},
        year={2024}
      }
 *
 * ============================================================================
 */

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
#include "geometric_controller.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT (float) 1.0f/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT (float) 1.0f/POS_UPDATE_RATE
#define PI 3.1415f

static const float gravity = 9.81f;  // [m/s^2]
static const float mass = 0.032f;  // [kg]  or 0.037
// static const float PI = 3.1415f;



static float K_xi_x = 7.0f;
static float K_xi_y = 7.0f;
static float K_xi_z = 7.0f;

static float K_v_x = 2.0f;
static float K_v_y = 2.0f;
static float K_v_z = 4.0f;


static float K_R_x = 421.7f;
static float K_R_y = 421.7f;
static float K_R_z = 273.0f;

static float K_Omega_x = 69.27f;
static float K_Omega_y = 69.27f;
static float K_Omega_z = 68.25f;

// static float K_inte_R_x = 1807.2f;
// static float K_inte_R_y = 1807.2f;
// static float K_inte_R_z = 1023.9f;

static struct vec Moment;
static float force;
static struct quat q_e;


// static struct mat33 Rd;
// static struct vec omega_dot_d;
// static struct vec omega_d;
// static struct vec omega_e;
// static struct vec inte_R_e;

static struct mat33 J=
    {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
      {0.83e-6f, 16.6e-6f, 1.8e-6f},
      {0.72e-6f, 1.8e-6f, 29.3e-6f}}};

// static struct mat33 J=
//     {{{16.6e-6f, 0.0f, 0.0f},
//       {0.0f, 16.6e-6f, 0.0f},
//       {0.0f, 0.0f, 29.3e-6f}}};

// Struct for logging position information
static bool isInit = false;

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  while (1)
  {
    vTaskDelay(M2T(2000));

  }
}


void controllerOutOfTreeInit(void) {
  Moment = vzero();
  force = 0.0f;
  q_e = qeye();
  // Rd = meye();
  // omega_dot_d = vzero();
  // omega_d= vzero();
  // omega_e= vzero();
  // inte_R_e  = vzero();

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
    struct mat33 R = quat2rotmat(attitude);

    float psi_d = radians(setpoint->attitude.yaw);
    float psi_dot_d = radians(setpoint->attitudeRate.yaw);

  //  Position Control Sliding Mode Control
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

    struct vec j_e = vzero();
    struct vec e3 = mkvec(0.0f,0.0f,1.0f);


    struct vec b1r = mkvec(cosf(psi_d),sinf(psi_d),0.0f);
    struct vec b1r_dot = mkvec(-sinf(psi_d)*psi_dot_d,cosf(psi_d)*psi_dot_d,0.0f);
    struct vec b1r_ddot =mkvec(-cosf(psi_d)*powf(psi_dot_d,2.0),-sinf(psi_d)*powf(psi_dot_d,2.0f),0.0f);

    struct vec xi_ddot_des = mkvec(setpoint->acceleration.x ,setpoint->acceleration.y ,setpoint->acceleration.z );


    struct mat33 K_xi = mscl(mass,mdiag(K_xi_x,K_xi_y,K_xi_z));
    struct mat33 K_v  = mscl(mass,mdiag(K_v_x,K_v_y,K_v_z));
    //
    struct vec kappa = vadd4(vneg(mvmul(K_xi,xi_e)),vneg(mvmul(K_v,v_e)),vscl(gravity*mass,e3),vscl(mass,xi_ddot_des));
    force = vdot(kappa,mvmul(R,e3));
    // Trajectory generation for the attiude controller
    // struct vec b3d = vdiv(kappa,vmag(kappa));
    // struct vec nu  = vcross(b3d,b1r);
    // struct vec b2d = vdiv(nu,vmag(nu));
    // struct vec b1d  = vcross(b2d,b3d);
    struct vec b3d = vnormalize(kappa);
    struct vec nu  = vcross(b3d,b1r);
    struct vec b2d = vnormalize(nu);
    struct vec b1d  = vcross(b2d,b3d);
    Rd = mcolumns(b1d,b2d,b3d);

    // first derivative 
    // struct vec xi_3dot_des = vzero();
    // struct vec kappa_dot = vadd3(vneg(mvmul(K_xi,v_e)),vneg(mvmul(K_v,a_e)),vscl(mass,xi_3dot_des));
    struct vec kappa_dot = vadd(vneg(mvmul(K_xi,v_e)),vneg(mvmul(K_v,a_e)));

    // struct vec b3d_dot = vsub(vdiv(kappa_dot,vmag(kappa)),vdiv(vscl(vdot(kappa,kappa_dot),kappa),powf(vmag(kappa),3.0f)));
    // struct vec nu_dot  = vadd(vcross(b3d_dot,b1r),vcross(b3d,b1r_dot));
    // struct vec b2d_dot = vsub(vdiv(nu_dot,vmag(nu)),vdiv(vscl(vdot(nu,nu_dot),nu),powf(vmag(nu),3.0f)));
    // struct vec b1d_dot  = vadd(vcross(b2d_dot,b3d),vcross(b2d,b3d_dot));
    struct vec b3d_dot = vnormalize1diff(kappa,kappa_dot);
    struct vec nu_dot  = vadd(vcross(b3d_dot,b1r),vcross(b3d,b1r_dot));
    struct vec b2d_dot =  vnormalize1diff(nu,nu_dot);
    struct vec b1d_dot  = vadd(vcross(b2d_dot,b3d),vcross(b2d,b3d_dot));
    struct mat33 Rd_dot = mcolumns(b1d_dot,b2d_dot,b3d_dot);
    

    // second derivative 
    // struct vec xi_4dot_des = vzero();
    // struct vec kappa_ddot = vadd3(vneg(mvmul(K_xi,a_e)),vneg(mvmul(K_v,j_e)),vscl(mass,xi_4dot_des));
    struct vec kappa_ddot = vadd(vneg(mvmul(K_xi,a_e)),vneg(mvmul(K_v,j_e)));
    
    // struct vec b3d_ddot = vadd4(vdiv(kappa_ddot,vmag(kappa)),vdiv(vscl(-2.0f*vdot(kappa,kappa_dot),kappa_dot),powf(vmag(kappa),3.0f)),
    //                             vscl(-1.0f*(powf(vmag(kappa_dot),2.0f)+vdot(kappa,kappa_ddot))/(powf(vmag(kappa),3.0f)),kappa),
    //                             vscl(3.0f*powf(vdot(kappa,kappa_dot),2.0f)/powf(vmag(kappa),5.0f),kappa));
    // struct vec nu_ddot  = vadd3(vcross(b3d_ddot,b1r),vcross(b3d,b1r_ddot),vscl(2.0f,vcross(b3d_dot,b1r_dot)));
    // struct vec b2d_ddot = vadd4(vdiv(nu_ddot,vmag(nu)),vdiv(vscl(-2.0f*vdot(nu,nu_dot),nu_dot),powf(vmag(nu),3.0f)),
    //                             vscl(-1.0f*(powf(vmag(nu_dot),2.0f)+vdot(nu,nu_ddot))/(powf(vmag(nu),3.0f)),nu),
    //                             vscl(3.0f*powf(vdot(nu,nu_dot),2.0f)/powf(vmag(nu),5.0f),nu));
    // struct vec b1d_ddot  = vadd3(vcross(b2d_ddot,b3d),vcross(b2d,b3d_ddot),vscl(2.0f,vcross(b2d_dot,b3d_dot)));
    struct vec b3d_ddot = vnormalize2diff(kappa,kappa_dot,kappa_ddot);
    struct vec nu_ddot  = vadd3(vcross(b3d_ddot,b1r),vcross(b3d,b1r_ddot),vscl(2.0f,vcross(b3d_dot,b1r_dot)));
    struct vec b2d_ddot = vnormalize2diff(nu,nu_dot,nu_ddot);
    struct vec b1d_ddot  = vadd3(vcross(b2d_ddot,b3d),vcross(b2d,b3d_ddot),vscl(2.0f,vcross(b2d_dot,b3d_dot)));

    struct mat33 Rd_ddot = mcolumns(b1d_ddot,b2d_ddot,b3d_ddot);
    omega_d = mvee(mmul(mtranspose(Rd),Rd_dot));
    omega_dot_d = vsub(mvee(mmul(mtranspose(Rd),Rd_ddot)),mkvec(omega_d.y*omega_d.z,omega_d.x*omega_d.z,omega_d.y*omega_d.x));
  }

  
  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = 0.0f;
    control->torqueX =  0.0f;
    control->torqueY =  0.0f;
    control->torqueZ =  0.0f;
  } else {

//  Attitude Control sliding mode control
    // Rd = meye();
    omega_dot_d = vzero();
    omega_d= vzero();
    // force = 0.0f;

    struct quat q_d = mat2quat(Rd);
    q_e = qqmul(qinv(q_d),attitude);

    struct vec R_e = vscl(0.5f,mvee(msub(mmul(mtranspose(Rd),R),mmul(mtranspose(R),Rd))));

    // inte_R_e = vadd(inte_R_e,vscl(ATTITUDE_UPDATE_DT, R_e));
    omega_e = vsub(omega,mvmul(mmul(mtranspose(R),Rd),omega_d));

    struct mat33 K_R      = mmul(mdiag(J.m[0][0],J.m[1][1],J.m[2][2]),mdiag(K_R_x,K_R_y,K_R_z));
    struct mat33 K_Omega  = mmul(mdiag(J.m[0][0],J.m[1][1],J.m[2][2]),mdiag(K_Omega_x,K_Omega_y,K_Omega_z));
    // struct mat33 K_inte_R = mmul(mdiag(J.m[0][0],J.m[1][1],J.m[2][2]),mdiag(K_inte_R_x,K_inte_R_y,K_inte_R_z));
    // computing the Moments
    // Moment = vadd4(vneg(mvmul(K_R,R_e)),vadd(vneg(mvmul(K_Omega,omega_e)),vneg(mvmul(K_inte_R,inte_R_e))),vcross(omega,mvmul(J,omega)),
    //                mvmul(mneg(J),vsub(mvmul(mmul(vhat(omega),mtranspose(R)),mvmul(Rd,omega_d)), mvmul(mmul(mtranspose(R),Rd),omega_dot_d))));

    Moment = vadd4(vneg(mvmul(K_R,R_e)),vneg(mvmul(K_Omega,omega_e)),vcross(omega,mvmul(J,omega)),
                   mvmul(mneg(J),vsub(mvmul(mmul(vhat(omega),mtranspose(R)),mvmul(Rd,omega_d)), mvmul(mmul(mtranspose(R),Rd),omega_dot_d))));
    
    //updating the Force and Moments
    control->thrustSi = force;
    control->torqueX = Moment.x;
    control->torqueY = Moment.y;
    control->torqueZ = Moment.z;
  }
  // Setting up the Control Mode
  control->controlMode = controlModeForceTorque;
}

bool controllerOutOfTreeTest(void) {
  return true;
}

// Parameter setup
PARAM_GROUP_START(ctrlGeo)
PARAM_ADD(PARAM_FLOAT, Kx_x, &K_xi_x)
PARAM_ADD(PARAM_FLOAT, Kx_y, &K_xi_y)
PARAM_ADD(PARAM_FLOAT, Kx_z, &K_xi_z)
PARAM_ADD(PARAM_FLOAT, Kv_x, &K_v_x)
PARAM_ADD(PARAM_FLOAT, Kv_y, &K_v_y)
PARAM_ADD(PARAM_FLOAT, Kv_z, &K_v_z)
PARAM_ADD(PARAM_FLOAT, KR_x, &K_R_x)
PARAM_ADD(PARAM_FLOAT, KR_y, &K_R_y)
PARAM_ADD(PARAM_FLOAT, KR_z, &K_R_z)
PARAM_ADD(PARAM_FLOAT, KOmega_x, &K_Omega_x)
PARAM_ADD(PARAM_FLOAT, KOmega_y, &K_Omega_y)
PARAM_ADD(PARAM_FLOAT, KOmega_z, &K_Omega_z)
// PARAM_ADD(PARAM_FLOAT, KI_R_x, &K_inte_R_x)
// PARAM_ADD(PARAM_FLOAT, KI_R_y, &K_inte_R_y)
// PARAM_ADD(PARAM_FLOAT, KI_R_z, &K_inte_R_z)
PARAM_GROUP_STOP(ctrlGeo)


// Logging info setup
LOG_GROUP_START(ctrlLog)
LOG_ADD(LOG_FLOAT, qe_x, &q_e.x)
LOG_ADD(LOG_FLOAT, qe_y, &q_e.y)
LOG_ADD(LOG_FLOAT, qe_z, &q_e.z)
LOG_ADD(LOG_FLOAT, qe_w, &q_e.w)
// LOG_ADD(LOG_FLOAT, omega_ex, &omega_e.x)
// LOG_ADD(LOG_FLOAT, omega_ey, &omega_e.y)
// LOG_ADD(LOG_FLOAT, omega_ez, &omega_e.z)
LOG_ADD(LOG_FLOAT, force, &force)
LOG_ADD(LOG_FLOAT, moment_x, &Moment.x)
LOG_ADD(LOG_FLOAT, moment_y, &Moment.y)
LOG_ADD(LOG_FLOAT, moment_z, &Moment.z)
LOG_GROUP_STOP(ctrlLog)

