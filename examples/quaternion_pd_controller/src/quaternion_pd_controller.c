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
#include "quaternion_pd_controller.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT (float) 1.0f/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT (float) 1.0f/POS_UPDATE_RATE
#define PI 3.1415f

static const float gravity = 9.81f;  // [m/s^2]
static const float mass = 0.032f;  // [kg]  or 0.037
// static const float PI = 3.1415f;


static float KD_q_x = 100.0f;
static float KD_q_y = 100.0f;
static float KD_q_z = 100.0f;
// static float K_q_x = 0.008f;
// static float K_q_y = 0.008f;
// static float K_q_z = 0.008f;
static float KP_q_x = 900.0f;
static float KP_q_y = 900.0f;
static float KP_q_z = 800.0f;



static float lambda_xi_x = 2.0f;
static float lambda_xi_y = 2.0f;
static float lambda_xi_z = 1.25f;
static float K_xi_x = 4.0f;
static float K_xi_y = 4.0f;
static float K_xi_z = 4.0f;
// static float rho_xi = 1.0f;
static float rho_xi_x = 0.8f;
static float rho_xi_y = 0.8f;
static float rho_xi_z = 0.8f;


static struct vec Moment;
static float force;
static struct quat q_e;
static struct quat q_d;
static float counter;



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

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  while (1)
  {
    vTaskDelay(M2T(2000));

  }
}


void controllerOutOfTreeInit(void) {
  q_e = qeye();
  q_d = qeye();
  Moment = vzero();
  force = 0.0f;
  counter = 0.0f;

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

    struct vec j_e = vzero();
    struct vec e3 = mkvec(0.0f,0.0f,1.0f);


    struct vec b1r = mkvec(cosf(psi_d),sinf(psi_d),0.0f);
    struct vec b1r_dot = mkvec(-sinf(psi_d)*psi_dot_d,cosf(psi_d)*psi_dot_d,0.0f);
    struct vec b1r_ddot =mkvec(-cosf(psi_d)*powf(psi_dot_d,2.0),-sinf(psi_d)*powf(psi_dot_d,2.0f),0.0f);

    // creating gain matrix 
    struct mat33 lambda_xi = mdiag(lambda_xi_x,lambda_xi_y,lambda_xi_z);
    struct mat33 K_xi = mdiag(K_xi_x,K_xi_y,K_xi_z);
    struct mat33 rho_xi = mdiag(rho_xi_x,rho_xi_y,rho_xi_z);

    struct vec xi_ddot_des = mkvec(setpoint->acceleration.x ,setpoint->acceleration.y ,setpoint->acceleration.z );
    struct vec s_xi = vadd(v_e , mvmul(lambda_xi, xi_e));

    struct vec rho_mvmult_s_xi = mvmul(rho_xi,s_xi);
    
    struct vec kappa = vscl(mass,vadd4(xi_ddot_des,vneg(mvmul(lambda_xi, v_e)),vscl(gravity,e3),vneg(mvmul(K_xi,vtanhf(rho_mvmult_s_xi)))));


    force = vdot(kappa,mvmul(R,e3));

// Trajectory generation for the attiude controller
    // struct vec b3d = vscl(1.0f/vmag(kappa),kappa);
    // struct vec b2d = vscl(1.0f/vmag(nu),nu);
    struct vec b3d = vnormalize(kappa);
    struct vec nu  = vcross(b3d,b1r);
    struct vec b2d = vnormalize(nu);
    struct vec b1d  = vcross(b2d,b3d);
    Rd = mcolumns(b1d,b2d,b3d);


    // first derivative 
    // struct vec xi_3dot_des = vzero();
    struct vec sdot_xi = vadd(a_e , mvmul(lambda_xi, v_e));
    // struct vec sdot_xi = vneg(mvmul(K_xi,vtanhf(s_xi)));
    // struct vec kappa_dot = vscl(mass,vadd3(xi_3dot_des,vneg(mvmul(lambda_xi, a_e)),vneg(mvmul(K_xi,veltmul(vpow(vsechf(s_xi),2.0f),sdot_xi)))));
    // struct vec kappa_dot = vscl(mass,vadd(vneg(mvmul(lambda_xi, a_e)),vneg(vscl(rho_xi,mvmul(K_xi,veltmul(vpow(vsechf(vscl(rho_xi,s_xi)),2.0f),sdot_xi))))));

    struct mat33 Kmultrho_xi = mdiag(K_xi_x*rho_xi_x,K_xi_y*rho_xi_y,K_xi_z*rho_xi_z);
    // struct vec kappa_dot = vscl(-mass,vadd(mvmul(lambda_xi, a_e),mvmul(rho_xi,mvmul(K_xi,veltmul(vpow(vsechf(rho_mvmult_s_xi),2.0f),sdot_xi)))));

    struct vec kappa_dot = vscl(-mass,vadd(mvmul(lambda_xi, a_e),mvmul(Kmultrho_xi,veltmul(vsech2f(rho_mvmult_s_xi),sdot_xi))));


    // struct vec b3d_dot = vsub(vdiv(kappa_dot,vmag(kappa)),vdiv(vscl(vdot(kappa,kappa_dot),kappa),powf(vmag(kappa),3.0f)));
    // struct vec b2d_dot = vsub(vdiv(nu_dot,vmag(nu)),vdiv(vscl(vdot(nu,nu_dot),nu),powf(vmag(nu),3.0f)));

    struct vec b3d_dot = vnormalize1diff(kappa,kappa_dot);
    struct vec nu_dot  = vadd(vcross(b3d_dot,b1r),vcross(b3d,b1r_dot));
    struct vec b2d_dot =  vnormalize1diff(nu,nu_dot);
    struct vec b1d_dot  = vadd(vcross(b2d_dot,b3d),vcross(b2d,b3d_dot));
    struct mat33 Rd_dot = mcolumns(b1d_dot,b2d_dot,b3d_dot);

    // second derivative 
    // struct vec xi_4dot_des = vzero();
    struct vec sddot_xi = vadd(j_e , mvmul(lambda_xi, a_e));
    // struct vec sddot_xi = vneg(mvmul(K_xi,veltmul(vpow(vsechf(s_xi),2.0f),sdot_xi)));
    // struct vec kappa_ddot = vscl(mass,vadd4(xi_4dot_des,vneg(mvmul(lambda_xi, j_e)),vneg(mvmul(K_xi,veltmul(vpow(vsechf(s_xi),2.0f),sddot_xi))),
    //                                         vscl(2.0,mvmul(K_xi,veltmul(veltmul(veltmul(vpow(vsechf(s_xi),2.0f),vtanhf(s_xi)),vpow(sdot_xi,2.0f)),sdot_xi)))));

    // struct vec kappa_ddot = vscl(mass,vadd(vneg(mvmul(K_xi,veltmul(vpow(vsechf(s_xi),2.0f),sddot_xi))),
    //                                         vscl(2.0,mvmul(K_xi,veltmul(veltmul(veltmul(vpow(vsechf(s_xi),2.0f),vtanhf(s_xi)),vpow(sdot_xi,2.0f)),sdot_xi)))));

    
    // struct vec kappa_ddot = vscl(mass,vadd(vscl(-rho_xi,mvmul(K_xi,veltmul(vpow(vsechf(vscl(rho_xi,s_xi)),2.0f),sddot_xi))),
    //                                         vscl(2.0f*rho_xi*rho_xi,mvmul(K_xi,veltmul(veltmul(vpow(vsechf(vscl(rho_xi,s_xi)),2.0f),vtanhf(vscl(rho_xi,s_xi))),vpow(sdot_xi,2.0f))))));
    // struct mat33 rho_xi2 = mdiag(rho_xi_x*rho_xi_x,rho_xi_y*rho_xi_y,rho_xi_z*rho_xi_z);
    struct mat33 Kmultrho2_xi_2 = mdiag(2.0f*K_xi_x*rho_xi_x*rho_xi_x,2.0f*K_xi_y*rho_xi_y*rho_xi_y,2.0f*K_xi_z*rho_xi_z*rho_xi_z);
    

    // struct vec kappa_ddot = vscl(mass,vadd(vneg(mvmul(rho_xi,mvmul(K_xi,veltmul(vpow(vsechf(mvmul(rho_xi,s_xi)),2.0f),sddot_xi)))),
    //                                         vscl(2.0f,mvmul(rho_xi2,mvmul(K_xi,veltmul(veltmul(vpow(vsechf(mvmul(rho_xi,s_xi)),2.0f),vtanhf(mvmul(rho_xi,s_xi))),vpow(sdot_xi,2.0f)))))));

    struct vec kappa_ddot = vscl(mass,vsub(mvmul(Kmultrho2_xi_2,veltmul3(vsech2f(rho_mvmult_s_xi),vtanhf(rho_mvmult_s_xi),vpow(sdot_xi,2.0f))),
                                            mvmul(Kmultrho_xi,veltmul(vsech2f(rho_mvmult_s_xi),sddot_xi))));

    // struct vec b3d_ddot = vadd3(vdiv(kappa_ddot,vmag(kappa)),vscl(-2.0f*vdot(kappa,kappa_dot)/(powf(vmag(kappa),3.0f)),kappa_dot),
    //                             vscl(-1.0f*(powf(vmag(kappa_dot),2.0f)+vdot(kappa,kappa_ddot))/(powf(vmag(kappa),3.0f))+3.0f*powf(vdot(kappa,kappa_dot),2.0f)/powf(vmag(kappa),5.0f),kappa));
    // struct vec b2d_ddot = vadd3(vdiv(nu_ddot,vmag(nu)),vscl(-2.0f*vdot(nu,nu_dot)/(powf(vmag(nu),3.0f)),nu_dot),
    //                             vscl(-1.0f*(powf(vmag(nu_dot),2.0f)+vdot(nu,nu_ddot))/(powf(vmag(nu),3.0f))+3.0f*powf(vdot(nu,nu_dot),2.0f)/powf(vmag(nu),5.0f),nu));


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
    counter = 0.0f;
  } else {
    
    if(counter<10){
      Rd = meye();
      counter++;
    }

    omega_dot_d = vzero();
    omega_d= vzero();
    
    q_d = mat2quat(Rd);

    // creating gain matrix 
    struct mat33 KD_q = mdiag(J.m[0][0]*KD_q_x,J.m[1][1]*KD_q_y,J.m[2][2]*KD_q_z);
    struct mat33 KP_q = mdiag(J.m[0][0]*KP_q_x,J.m[1][1]*KP_q_y,J.m[2][2]*KP_q_z);   

    // computing the angular velocity and quaternion errors
    omega_e = vsub(omega, omega_d);
    // struct quat q_e = qqmul(qinv(q_d),attitude);
    q_e = qqmul(qinv(q_d),attitude);

    // computing the quaternion dot error
    // struct quat q_e_dot = qscl(0.5,qqmul(q_e,mkquat(omega_e.x,omega_e.y,omega_e.z,0.0f)));
    // q_e_dot = qscl(0.5,qqmul(q_e,mkquat(omega_e.x,omega_e.y,omega_e.z,0.0f)));
    // computing the sliding surface
    

    Moment = vneg(vadd(mvmul(KD_q, omega_e), mvmul(KP_q, vscl(sgnplus(q_e.w),mkvec(q_e.x,q_e.y,q_e.z)))));
    // Moment = vneg(vadd(mvmul(KD_q, omega_e), mvmul(KP_q, mkvec(q_e.x,q_e.y,q_e.z))));
    // Moment = mkvec(0.0f,0.0f,0.0f);
    
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
PARAM_GROUP_START(ctrlQPD)
PARAM_ADD(PARAM_FLOAT, KD_att_x, &KD_q_x)
PARAM_ADD(PARAM_FLOAT, KD_att_y, &KD_q_y)
PARAM_ADD(PARAM_FLOAT, KD_att_z, &KD_q_z)
PARAM_ADD(PARAM_FLOAT, KP_att_x, &KP_q_x)
PARAM_ADD(PARAM_FLOAT, KP_att_y, &KP_q_y)
PARAM_ADD(PARAM_FLOAT, KP_att_z, &KP_q_z)
PARAM_ADD(PARAM_FLOAT, lambda_pos_x, &lambda_xi_x)
PARAM_ADD(PARAM_FLOAT, lambda_pos_y, &lambda_xi_y)
PARAM_ADD(PARAM_FLOAT, lambda_pos_z, &lambda_xi_z)
PARAM_ADD(PARAM_FLOAT, K_pos_x, &K_xi_x)
PARAM_ADD(PARAM_FLOAT, K_pos_y, &K_xi_y)
PARAM_ADD(PARAM_FLOAT, K_pos_z, &K_xi_z)
PARAM_ADD(PARAM_FLOAT, rho_xi_x, &rho_xi_x)
PARAM_ADD(PARAM_FLOAT, rho_xi_y, &rho_xi_y)
PARAM_ADD(PARAM_FLOAT, rho_xi_z, &rho_xi_z)
PARAM_GROUP_STOP(ctrlQPD)


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
LOG_ADD(LOG_FLOAT, moment_x, &Moment.x)
LOG_ADD(LOG_FLOAT, moment_y, &Moment.y)
LOG_ADD(LOG_FLOAT, moment_z, &Moment.z)
LOG_GROUP_STOP(ctrlLog)

