
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

// Euler based sliding mode control

#include <string.h>
#include <stdint.h>
#include <stdbool.h>


#include "app.h"
#include "controller_brescianini.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"
#define DEBUG_MODULE "MYONTROLLER"
#include "debug.h"
#include "my_controller.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT (float) 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT (float) 1.0/POS_UPDATE_RATE

static const float gravity = 9.81f;  // [m/s^2]
static const float mass = 0.032f;  // [kg]  or 0.037
static const float PI = 3.1415f;
static const float d = 0.046f; //[m]

static float Jr = 6e-7f;
static struct mat33 J =
    {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
      {0.83e-6f, 16.6e-6f, 1.8e-6f},
      {0.72e-6f, 1.8e-6f, 29.3e-6f}}};


static float lambda_q_x = 4.0f;
static float lambda_q_y = 4.0f;
static float lambda_q_z = 4.0f;
static float K_q_x = 8.0f;
static float K_q_y = 8.0f;
static float K_q_z = 8.0f;

static float rho_q_x = 0.5f;
static float rho_q_y = 0.5f;
static float rho_q_z = 0.5f;


// static float Gamma_q_x = 0.001f;
// static float Gamma_q_y = 0.001f;
// static float Gamma_q_z = 0.001f;
// static float Ka_q_x;
// static float Ka_q_y;
// static float Ka_q_z;

static float lambda_xi_x = 2.0f;
static float lambda_xi_y = 2.0f;
static float lambda_xi_z = 1.0f;
static float K_xi_x = 5.0f;
static float K_xi_y = 5.0f;
static float K_xi_z = 8.0f;


static float rho_xi_x = 0.8f;
static float rho_xi_y = 0.8f;
static float rho_xi_z = 0.8f;

// static float Gamma_xi_x = 0.001f;
// static float Gamma_xi_y = 0.001f;
// static float Gamma_xi_z = 0.001f;
// static float Ka_xi_x;
// static float Ka_xi_y;
// static float Ka_xi_z;

// static float psi_old ;
// static float psi_d_old;


static struct vec s_q;
static struct vec s_xi;
static struct vec Moment;

static float force;
static float ux;
static float uy;
struct vec xi_ddot_des;
static struct quat q_e;
struct vec Phi_dot_e;  

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
  //  psi_old = 0.0f;
  //  psi_d_old =0.0f;
  s_q =vzero();
  s_xi=vzero();
  Moment=vzero();
  force=0.0f;
  q_e = qeye();
  ux=0.0f;
  uy=0.0f;
  // Ka_q_x = K_q_x;
  // Ka_q_y = K_q_y;
  // Ka_q_z = K_q_z;
  // Ka_xi_x = K_xi_x;
  // Ka_xi_y = K_xi_y;
  // Ka_xi_z = K_xi_z;
  xi_ddot_des =vzero();
  Phi_dot_e= vzero();

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


   
    // static float psi_d_old;
    static float phi_d;
    static float theta_d;
    // static float Kdot_q_x;
    // static float Kdot_q_y;
    // static float Kdot_q_z;
    // static float Kdot_xi_x;
    // static float Kdot_xi_y;
    // static float Kdot_xi_z;


    float a1 = (J.m[1][1]-J.m[2][2])/J.m[0][0];
    float a3 = (J.m[2][2]-J.m[0][0])/J.m[1][1];
    float a5 = (J.m[0][0]-J.m[1][1])/J.m[2][2];
    float a2 = Jr/J.m[0][0];
    float a4 = Jr/J.m[1][1];
    float b1 = d/J.m[0][0];
    float b2 = d/J.m[1][1];
    float b3 = d/J.m[2][2];


    struct vec Phi_dot= mkvec(
      radians(sensors->gyro.x),
      radians(sensors->gyro.y),
      radians(sensors->gyro.z)
      );
    float phi_dot = Phi_dot.x;
    // float theta_dot = Phi_dot.y;
    float theta_dot = Phi_dot.y;
    float psi_dot = Phi_dot.z; 
    float Omega = 0.0f;

    float psi = radians(state->attitude.yaw);

    // float delta_psi = psi - psi_old;
    // if (delta_psi>PI){
    //     delta_psi = delta_psi-2.0f*PI;
    // }else if (delta_psi<-PI)
    // {
    //   delta_psi = delta_psi+2.0f*PI;
    // }
    // psi = psi_old + delta_psi;
    // psi_old = psi;

      // current attitude
    struct vec Phi= mkvec(radians(state->attitude.roll),
                          -radians(state->attitude.pitch),
                          psi);


    float phi = Phi.x;
    // float theta = Phi.y;
    float theta = Phi.y;
    float psi_d = radians(setpoint->attitude.yaw);
  //  Position Control Sliding Mode Control
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, stabilizerStep)) {

    // computing the position, velocity, acceleration and jerk errors
    struct vec xi_e = mkvec(state->position.x - setpoint->position.x ,
                            state->position.y - setpoint->position.y ,
                            state->position.z - setpoint->position.z);

    struct vec v_e = mkvec(state->velocity.x - setpoint->velocity.x,
                           state->velocity.y - setpoint->velocity.y,
                           state->velocity.z - setpoint->velocity.z);


    // creating gain matrix 
    struct mat33 lambda_xi = mdiag(lambda_xi_x,lambda_xi_y,lambda_xi_z);
    // struct mat33 K_xi = mdiag(K_xi_x,K_xi_y,K_xi_z);   
    xi_ddot_des = mkvec(setpoint->acceleration.x ,setpoint->acceleration.y ,setpoint->acceleration.z );
    s_xi = vadd(v_e , mvmul(lambda_xi, xi_e));
    
    
    force = mass/(cosf(phi)*cosf(theta))*(-K_xi_z*tanhf(rho_xi_z*s_xi.z)-lambda_xi_z*v_e.z + gravity + xi_ddot_des.z);

    ux = mass/force *(-K_xi_x*tanhf(rho_xi_x*s_xi.x)-lambda_xi_x*v_e.x + xi_ddot_des.x);
    uy = mass/force *(-K_xi_y*tanhf(rho_xi_y*s_xi.y)-lambda_xi_y*v_e.y + xi_ddot_des.y);


    // force = mass/(cosf(phi)*cosf(theta))*(-Ka_xi_z*tanhf(s_xi.z)-lambda_xi_z*v_e.z + GRAVITY_MAGNITUDE + xi_ddot_des.z);

    // ux = mass/force *(-Ka_xi_x*tanhf(s_xi.x)-lambda_xi_x*v_e.x + xi_ddot_des.x);
    // uy = mass/force *(-Ka_xi_y*tanhf(s_xi.y)-lambda_xi_y*v_e.y + xi_ddot_des.y);
    
    // Kdot_xi_x = calculateKdot(K_xi_x, Gamma_xi_x, s_xi.x,0.002f);
    // Kdot_xi_y = calculateKdot(K_xi_y, Gamma_xi_y, s_xi.y,0.002f);
    // Kdot_xi_z = calculateKdot(K_xi_z, Gamma_xi_z, s_xi.z,0.002f);
    // Ka_xi_x = Ka_xi_x + (POS_UPDATE_DT)*Kdot_xi_x;
    // Ka_xi_y = Ka_xi_y + (POS_UPDATE_DT)*Kdot_xi_y;
    // Ka_xi_z = Ka_xi_z + (POS_UPDATE_DT)*Kdot_xi_z;


    // float delta_psi_d= psi_d - psi_d_old;

    // if (delta_psi_d>PI){
    //     delta_psi_d = delta_psi_d-2.0f*PI;
    // }else if (delta_psi_d<-PI)
    // {
    //   delta_psi_d = delta_psi_d+2.0f*PI;
    // }
    // psi_d = psi_d_old + delta_psi_d;
    // psi_d_old = psi_d;
    

    phi_d = asinf(minf(maxf(ux*sinf(psi_d)-uy*cosf(psi_d),-1),1));
    theta_d = asinf(minf(maxf((ux*cosf(psi_d)+uy*sinf(psi_d))/cosf(phi_d),-1),1));

  }

    // phi_d =0.0f;
    // theta_d =0.0f;
    // psi_d =0.0f;
    // force = 0.0f;

    struct mat33 lambda_q = mdiag(lambda_q_x,lambda_q_y,lambda_q_z); 

  
    struct vec Phi_d = mkvec(phi_d,theta_d,psi_d);
 
    struct vec Phi_dot_d = vzero();
    struct vec Phi_ddot_d = vzero();
    struct vec Phi_e = vsub(Phi, Phi_d); 
    Phi_dot_e = vsub(Phi_dot, Phi_dot_d); 

    // computing the sliding surface
    s_q = vadd(Phi_dot_e , mvmul(lambda_q,Phi_e));


  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = 0.0f;
    control->torqueX =  0.0f;
    control->torqueY =  0.0f;
    control->torqueZ =  0.0f;
  } else {

//  Attitude Control sliding mode control

    q_e = rpy2quat(Phi_e);

    // // // computing the Moments
    // Moment.x =1.0f/b1*(Phi_ddot_d.x-lambda_q_x*Phi_dot_e.x - Ka_q_x*tanhf(rho_q_x*s_q.x) - theta_dot*psi_dot*a1 + theta_dot*a2*Omega);
    // Moment.y =1.0f/b2*(Phi_ddot_d.y-lambda_q_y*Phi_dot_e.y - Ka_q_y*tanhf(rho_q_y*s_q.y) - phi_dot*psi_dot*a3 - phi_dot*a4*Omega);
    // Moment.z =1.0f/b3*(Phi_ddot_d.z-lambda_q_z*Phi_dot_e.z - Ka_q_z*tanhf(rho_q_z*s_q.z) - theta_dot*phi_dot*a5);

    Moment.x =1.0f/b1*(Phi_ddot_d.x-lambda_q_x*Phi_dot_e.x - K_q_x*tanhf(rho_q_x*s_q.x) - theta_dot*psi_dot*a1 + theta_dot*a2*Omega);
    Moment.y =1.0f/b2*(Phi_ddot_d.y-lambda_q_y*Phi_dot_e.y - K_q_y*tanhf(rho_q_y*s_q.y) - phi_dot*psi_dot*a3 - phi_dot*a4*Omega);
    Moment.z =1.0f/b3*(Phi_ddot_d.z-lambda_q_z*Phi_dot_e.z - K_q_z*tanhf(rho_q_z*s_q.z) - theta_dot*phi_dot*a5);

    // Adaptive gain
    // Moment.x =1.0f/b1*(Phi_ddot_d.x-lambda_q_x*Phi_dot_e.x - Ka_q_x*tanhf(s_q.x) - theta_dot*psi_dot*a1 + theta_dot*a2*Omega);
    // Moment.y =1.0f/b2*(Phi_ddot_d.y-lambda_q_y*Phi_dot_e.y - Ka_q_y*tanhf(s_q.y) - phi_dot*psi_dot*a3 - phi_dot*a4*Omega);
    // Moment.z =1.0f/b3*(Phi_ddot_d.z-lambda_q_z*Phi_dot_e.z - Ka_q_z*tanhf(s_q.z) - theta_dot*phi_dot*a5);
    
    // Kdot_q_x = calculateKdot(Ka_q_x,Gamma_q_x, s_q.x,0.018f);
    // Kdot_q_y = calculateKdot(Ka_q_y,Gamma_q_y, s_q.y,0.01f);
    // Kdot_q_z = calculateKdot(Ka_q_z,Gamma_q_z, s_q.z,0.01f);
    // Ka_q_x = Ka_q_x + (ATTITUDE_UPDATE_DT)*Kdot_q_x;
    // Ka_q_y = Ka_q_y + (ATTITUDE_UPDATE_DT)*Kdot_q_y;
    // Ka_q_z = Ka_q_z + (ATTITUDE_UPDATE_DT)*Kdot_q_z;

    // updating the Force and Moments
    // control->thrustSi = 0; // force to provide control_thrust
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
PARAM_GROUP_START(ctrlSMC)
PARAM_ADD(PARAM_FLOAT, K_att_x, &K_q_x)
PARAM_ADD(PARAM_FLOAT, lambda_att_x, &lambda_q_x)
PARAM_ADD(PARAM_FLOAT, K_att_y, &K_q_y)
PARAM_ADD(PARAM_FLOAT, lambda_att_y, &lambda_q_y)
PARAM_ADD(PARAM_FLOAT, K_att_z, &K_q_z)
PARAM_ADD(PARAM_FLOAT, lambda_att_z, &lambda_q_z)
PARAM_ADD(PARAM_FLOAT, K_pos_x, &K_xi_x)
PARAM_ADD(PARAM_FLOAT, lambda_pos_x, &lambda_xi_x)
PARAM_ADD(PARAM_FLOAT, K_pos_y, &K_xi_y)
PARAM_ADD(PARAM_FLOAT, lambda_pos_y, &lambda_xi_y)
PARAM_ADD(PARAM_FLOAT, K_pos_z, &K_xi_z)
PARAM_ADD(PARAM_FLOAT, lambda_pos_z, &lambda_xi_z)
// PARAM_ADD(PARAM_FLOAT, Gamma_att_x, &Gamma_q_x)
// PARAM_ADD(PARAM_FLOAT, Gamma_att_y, &Gamma_q_y)
// PARAM_ADD(PARAM_FLOAT, Gamma_att_z, &Gamma_q_z)
// PARAM_ADD(PARAM_FLOAT, Gamma_pos_x, &Gamma_xi_x)
// PARAM_ADD(PARAM_FLOAT, Gamma_pos_y, &Gamma_xi_y)
// PARAM_ADD(PARAM_FLOAT, Gamma_pos_z, &Gamma_xi_z)
PARAM_ADD(PARAM_FLOAT, rho_xi_x, &rho_xi_x)
PARAM_ADD(PARAM_FLOAT, rho_xi_y, &rho_xi_y)
PARAM_ADD(PARAM_FLOAT, rho_xi_z, &rho_xi_z)
PARAM_ADD(PARAM_FLOAT, rho_q_x, &rho_q_x)
PARAM_ADD(PARAM_FLOAT, rho_q_y, &rho_q_y)
PARAM_ADD(PARAM_FLOAT, rho_q_z, &rho_q_z)
PARAM_GROUP_STOP(ctrlSMC)


// Logging info setup
LOG_GROUP_START(ctrlLog)
LOG_ADD(LOG_FLOAT, qe_x, &q_e.x)
LOG_ADD(LOG_FLOAT, qe_y, &q_e.y)
LOG_ADD(LOG_FLOAT, qe_z, &q_e.z)
LOG_ADD(LOG_FLOAT, qe_w, &q_e.w)
LOG_ADD(LOG_FLOAT, omega_ex, &Phi_dot_e.x)
LOG_ADD(LOG_FLOAT, omega_ey, &Phi_dot_e.y)
LOG_ADD(LOG_FLOAT, omega_ez, &Phi_dot_e.z)
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
LOG_ADD(LOG_FLOAT, ux, &ux)
LOG_ADD(LOG_FLOAT, uy, &uy)
// LOG_ADD(LOG_FLOAT, Ka_q_x, &Ka_q_x)
// LOG_ADD(LOG_FLOAT, Ka_q_y, &Ka_q_y)
// LOG_ADD(LOG_FLOAT, Ka_q_z, &Ka_q_z)
// LOG_ADD(LOG_FLOAT, Ka_xi_x, &Ka_xi_x)
// LOG_ADD(LOG_FLOAT, Ka_xi_y, &Ka_xi_y)
// LOG_ADD(LOG_FLOAT, Ka_xi_z, &Ka_xi_z)
LOG_ADD(LOG_FLOAT, ax_d, &xi_ddot_des.x)
LOG_ADD(LOG_FLOAT, ay_d, &xi_ddot_des.y)
LOG_ADD(LOG_FLOAT, az_d, &xi_ddot_des.z)
LOG_GROUP_STOP(ctrlLog)



