// /**
//  * Authored by Michael Hamer (http://www.mikehamer.info), November 2016.
//  *
//  * This program is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, in version 3.
//  *
//  * This program is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with this program. If not, see <http://www.gnu.org/licenses/>.
//  *
//  * ============================================================================
//  *
//  * The controller implemented in this file is based on the paper:
//  *
//  * "Quaternion-Based Sliding Mode Control for Six Degrees of Freedom Flight
// Control of Quadrotors"
//  * https://arxiv.org/pdf/2403.10934
//  *
//  * Academic citation would be appreciated.
//  *
//  * BIBTEX ENTRIES:
//       @article{yazdanshenas2024quaternion,
//         title={Quaternion-Based Sliding Mode Control for Six Degrees of Freedom Flight Control of Quadrotors},
//         author={Yazdanshenas, Amin and Faieghi, Reza},
//         journal={arXiv preprint arXiv:2403.10934},
//         year={2024}
//       }
//  *
//  * ============================================================================
//  */
// #include <setjmp.h>
// #include <stdio.h>
// #include <string.h>
// #include <stdint.h>
// #include <stdbool.h>

// #include "app.h"

// #include "FreeRTOS.h"
// #include "task.h"
// #include "controller_brescianini.h"
// #include "log.h"
// #include "param.h"
// #include "num.h"
// #include "math3d.h"
// #include "physicalConstants.h"
// #define DEBUG_MODULE "MYCONTROLLER"
// #include "debug.h"
// #include "quaternion_smc_controller.h"

// #define ATTITUDE_UPDATE_RATE RATE_250_HZ
// #define ATTITUDE_UPDATE_DT (float) 1.0f/ATTITUDE_UPDATE_RATE

// #define POS_UPDATE_RATE RATE_100_HZ
// #define POS_UPDATE_DT (float) 1.0f/POS_UPDATE_RATE
// #define PI 3.1415f

// #define SMOOTHING_ALPHA 0.1f

// static const float gravity = 9.81f;  // [m/s^2]
// static const float mass = 0.032f;  // [kg]  or 0.037
// // static const float PI = 3.1415f;


// static float lambda_q_x = 8.0f;
// static float lambda_q_y = 8.0f;
// static float lambda_q_z = 8.0f;
// // static float K_q_x = 0.008f;
// // static float K_q_y = 0.008f;
// // static float K_q_z = 0.008f;
// static float K_q_x = 400.0f;
// static float K_q_y = 400.0f;
// static float K_q_z = 500.0f;

// // static float rho_q = 1.0f;
// static float rho_q_x = 0.25f;
// static float rho_q_y = 0.25f;
// static float rho_q_z = 0.5f;




// static float lambda_xi_x = 1.25f;
// static float lambda_xi_y = 1.25f;
// static float lambda_xi_z = 1.25f;
// static float K_xi_x = 5.0f;
// static float K_xi_y = 5.0f;
// static float K_xi_z = 5.0f;
// // static float rho_xi = 1.0f;
// static float rho_xi_x = 1.0f;
// static float rho_xi_y = 1.0f;
// static float rho_xi_z = 1.0f;

// static float omega_scale = 1.0f;


// // static struct vec s_q;
// // static struct vec s_xi;
// static struct vec Moment;
// static float force;
// static struct quat q_e;
// struct vec xi_2dot_des ;
// struct vec xi_3dot_des ;
// struct vec xi_4dot_des ;
// struct vec j_e ;
// struct vec a_e ;

// static struct vec prev_a_e;
// struct vec xi_3dot_des_prev ;


// // static struct mat33 Rd;
// // static struct vec omega_dot_d;
// // static struct vec omega_d;
// // static struct vec omega_e;

// // static struct mat33 J=
// //     {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
// //       {0.83e-6f, 16.6e-6f, 1.8e-6f},
// //       {0.72e-6f, 1.8e-6f, 29.3e-6f}}};

// static struct mat33 J=
//     {{{16.6e-6f, 0.0f, 0.0f},
//       {0.0f, 16.6e-6f, 0.0f},
//       {0.0f, 0.0f, 29.3e-6f}}};

// // Struct for logging position information
// static bool isInit = false;
// // static jmp_buf buf;


// void appMain()
// {
//   DEBUG_PRINT("Waiting for activation ...\n");

//   while (1)
//   {
//     vTaskDelay(M2T(2000));

//   }
// }


// void controllerOutOfTreeInit(void) {
//   // s_q = vzero();
//   // s_xi = vzero();
//   q_e = qeye();
//   a_e = vzero();
//   j_e = vzero();
//   Moment = vzero();
//   force = 0.0f;
//   xi_2dot_des = vzero();
//   xi_3dot_des = vzero();
//   prev_a_e = vzero();
//   xi_4dot_des = vzero();
//   xi_3dot_des_prev = vzero();
//   // Rd = meye();
//   // omega_dot_d = vzero();
//   // omega_d= vzero();
//   // omega_e= vzero();
//   if (isInit) {
//     return;
//   }

//   isInit = true;
// }



// void controllerOutOfTree(control_t *control,
//                                  const setpoint_t *setpoint,
//                                  const sensorData_t *sensors,
//                                  const state_t *state,
//                                  const stabilizerStep_t stabilizerStep) {

//   // if (setjmp(buf)) {
//   //   // Handle error
//   //   DEBUG_PRINT("An error occurred. Execution halted.\n");
//   //   return;
//   // }
//     static struct mat33 Rd;
//     static struct vec omega_dot_d;
//     static struct vec omega_d;
//     static struct vec omega_e;

//     struct vec omega = mkvec(
//       radians(sensors->gyro.x),
//       radians(sensors->gyro.y),
//       radians(sensors->gyro.z)
//       );

//       // current attitude
//     struct quat attitude = mkquat(
//       state->attitudeQuaternion.x,
//       state->attitudeQuaternion.y,
//       state->attitudeQuaternion.z,
//       state->attitudeQuaternion.w);
//     attitude = qnormalize(attitude);
//     struct mat33 R = quat2rotmat(attitude);

//     float psi_d = radians(setpoint->attitude.yaw);
//     float psi_dot_d = radians(setpoint->attitudeRate.yaw);

//   //  Position Control Sliding Mode Control
//   // if (RATE_DO_EXECUTE(POS_UPDATE_RATE, stabilizerStep)) {
  
//   if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, stabilizerStep)) {


//     // computing the position, velocity, acceleration and jerk errors
//     struct vec xi_e = mkvec(state->position.x - setpoint->position.x ,
//                             state->position.y - setpoint->position.y ,
//                             state->position.z - setpoint->position.z);

//     struct vec v_e = mkvec(state->velocity.x - setpoint->velocity.x,
//                            state->velocity.y - setpoint->velocity.y,
//                            state->velocity.z - setpoint->velocity.z);

//     a_e = mkvec(state->acc.x - setpoint->acceleration.x,
//                            state->acc.y - setpoint->acceleration.y,
//                            state->acc.z - setpoint->acceleration.z);

//     // struct vec j_e = vzero();
//     j_e = compute_jerk_error(a_e,prev_a_e);
//     prev_a_e = smooth_acceleration(a_e);
//     struct vec e3 = mkvec(0.0f,0.0f,1.0f);


//     struct vec b1r = mkvec(cosf(psi_d),sinf(psi_d),0.0f);
//     struct vec b1r_dot = mkvec(-sinf(psi_d)*psi_dot_d,cosf(psi_d)*psi_dot_d,0.0f);
//     struct vec b1r_ddot =mkvec(-cosf(psi_d)*powf(psi_dot_d,2.0),-sinf(psi_d)*powf(psi_dot_d,2.0f),0.0f);

//     // creating gain matrix 
//     struct mat33 lambda_xi = mdiag(lambda_xi_x,lambda_xi_y,lambda_xi_z);
//     struct mat33 K_xi = mdiag(K_xi_x,K_xi_y,K_xi_z);
//     struct mat33 rho_xi = mdiag(rho_xi_x,rho_xi_y,rho_xi_z);

//     xi_2dot_des = mkvec(setpoint->acceleration.x ,setpoint->acceleration.y ,setpoint->acceleration.z);
//     xi_3dot_des = mkvec(setpoint->jerk.x ,setpoint->jerk.y ,setpoint->jerk.z);
//     xi_4dot_des = compute_derivative(xi_3dot_des,xi_3dot_des_prev);
//     xi_3dot_des_prev = xi_3dot_des;
//     struct vec s_xi = vadd(v_e , mvmul(lambda_xi, xi_e));

//     struct vec rho_mvmult_s_xi = mvmul(rho_xi,s_xi);
    
//     struct vec kappa = vscl(mass,vadd4(xi_2dot_des,vneg(mvmul(lambda_xi, v_e)),vscl(gravity,e3),vneg(mvmul(K_xi,vtanhf(rho_mvmult_s_xi)))));


//     force = vdot(kappa,mvmul(R,e3));

// // Trajectory generation for the attiude controller
//     // struct vec b3d = vscl(1.0f/vmag(kappa),kappa);
//     // struct vec b2d = vscl(1.0f/vmag(nu),nu);
//     struct vec b3d = vnormalize(kappa);
//     struct vec nu  = vcross(b3d,b1r);
//     struct vec b2d = vnormalize(nu);
//     struct vec b1d  = vcross(b2d,b3d);
//     Rd = mcolumns(b1d,b2d,b3d);


//     // first derivative 
//     // struct vec xi_3dot_des = vzero();
//     struct vec sdot_xi = vadd(a_e , mvmul(lambda_xi, v_e));
//     // struct vec sdot_xi = vneg(mvmul(K_xi,vtanhf(s_xi)));
//     // struct vec kappa_dot = vscl(mass,vadd3(xi_3dot_des,vneg(mvmul(lambda_xi, a_e)),vneg(mvmul(K_xi,veltmul(vpow(vsechf(s_xi),2.0f),sdot_xi)))));
//     // struct vec kappa_dot = vscl(mass,vadd(vneg(mvmul(lambda_xi, a_e)),vneg(vscl(rho_xi,mvmul(K_xi,veltmul(vpow(vsechf(vscl(rho_xi,s_xi)),2.0f),sdot_xi))))));

//     struct mat33 Kmultrho_xi = mdiag(K_xi_x*rho_xi_x,K_xi_y*rho_xi_y,K_xi_z*rho_xi_z);
//     // struct mat33 Kmultrho_xi = mdiag(K_xi_x,K_xi_y,K_xi_z);
//     // struct vec kappa_dot = vscl(-mass,vadd(mvmul(lambda_xi, a_e),mvmul(rho_xi,mvmul(K_xi,veltmul(vpow(vsechf(rho_mvmult_s_xi),2.0f),sdot_xi)))));

//     struct vec kappa_dot = vscl(-mass,vadd(mvmul(lambda_xi, a_e),mvmul(Kmultrho_xi,veltmul(vsech2f(rho_mvmult_s_xi),sdot_xi))));

//     kappa_dot = vadd(vscl(mass,xi_3dot_des),kappa_dot);


//     // struct vec b3d_dot = vsub(vdiv(kappa_dot,vmag(kappa)),vdiv(vscl(vdot(kappa,kappa_dot),kappa),powf(vmag(kappa),3.0f)));
//     // struct vec b2d_dot = vsub(vdiv(nu_dot,vmag(nu)),vdiv(vscl(vdot(nu,nu_dot),nu),powf(vmag(nu),3.0f)));

//     struct vec b3d_dot = vnormalize1diff(kappa,kappa_dot);
//     struct vec nu_dot  = vadd(vcross(b3d_dot,b1r),vcross(b3d,b1r_dot));
//     struct vec b2d_dot =  vnormalize1diff(nu,nu_dot);
//     struct vec b1d_dot  = vadd(vcross(b2d_dot,b3d),vcross(b2d,b3d_dot));
//     struct mat33 Rd_dot = mcolumns(b1d_dot,b2d_dot,b3d_dot);

//     // second derivative 
//     // struct vec xi_4dot_des = vzero();
//     struct vec sddot_xi = vadd(j_e , mvmul(lambda_xi, a_e));
//     // struct vec sddot_xi = vneg(mvmul(K_xi,veltmul(vpow(vsechf(s_xi),2.0f),sdot_xi)));
//     // struct vec kappa_ddot = vscl(mass,vadd4(xi_4dot_des,vneg(mvmul(lambda_xi, j_e)),vneg(mvmul(K_xi,veltmul(vpow(vsechf(s_xi),2.0f),sddot_xi))),
//     //                                         vscl(2.0,mvmul(K_xi,veltmul(veltmul(veltmul(vpow(vsechf(s_xi),2.0f),vtanhf(s_xi)),vpow(sdot_xi,2.0f)),sdot_xi)))));

//     // struct vec kappa_ddot = vscl(mass,vadd(vneg(mvmul(K_xi,veltmul(vpow(vsechf(s_xi),2.0f),sddot_xi))),
//     //                                         vscl(2.0,mvmul(K_xi,veltmul(veltmul(veltmul(vpow(vsechf(s_xi),2.0f),vtanhf(s_xi)),vpow(sdot_xi,2.0f)),sdot_xi)))));

    
//     // struct vec kappa_ddot = vscl(mass,vadd(vscl(-rho_xi,mvmul(K_xi,veltmul(vpow(vsechf(vscl(rho_xi,s_xi)),2.0f),sddot_xi))),
//     //                                         vscl(2.0f*rho_xi*rho_xi,mvmul(K_xi,veltmul(veltmul(vpow(vsechf(vscl(rho_xi,s_xi)),2.0f),vtanhf(vscl(rho_xi,s_xi))),vpow(sdot_xi,2.0f))))));
//     // struct mat33 rho_xi2 = mdiag(rho_xi_x*rho_xi_x,rho_xi_y*rho_xi_y,rho_xi_z*rho_xi_z);
//     struct mat33 Kmultrho2_xi_2 = mdiag(2.0f*K_xi_x*rho_xi_x*rho_xi_x,2.0f*K_xi_y*rho_xi_y*rho_xi_y,2.0f*K_xi_z*rho_xi_z*rho_xi_z);
//     // struct mat33 Kmultrho2_xi_2 = mdiag(2.0f*K_xi_x,2.0f*K_xi_y,2.0f*K_xi_z);
    

//     // struct vec kappa_ddot = vscl(mass,vadd(vneg(mvmul(rho_xi,mvmul(K_xi,veltmul(vpow(vsechf(mvmul(rho_xi,s_xi)),2.0f),sddot_xi)))),
//     //                                         vscl(2.0f,mvmul(rho_xi2,mvmul(K_xi,veltmul(veltmul(vpow(vsechf(mvmul(rho_xi,s_xi)),2.0f),vtanhf(mvmul(rho_xi,s_xi))),vpow(sdot_xi,2.0f)))))));

//     struct vec kappa_ddot = vscl(mass,vsub(mvmul(Kmultrho2_xi_2,veltmul3(vsech2f(rho_mvmult_s_xi),vtanhf(rho_mvmult_s_xi),vpow(sdot_xi,2.0f))),
//                                             mvmul(Kmultrho_xi,veltmul(vsech2f(rho_mvmult_s_xi),sddot_xi))));

//     kappa_ddot = vadd(vscl(mass,xi_4dot_des),kappa_ddot);

//     // struct vec b3d_ddot = vadd3(vdiv(kappa_ddot,vmag(kappa)),vscl(-2.0f*vdot(kappa,kappa_dot)/(powf(vmag(kappa),3.0f)),kappa_dot),
//     //                             vscl(-1.0f*(powf(vmag(kappa_dot),2.0f)+vdot(kappa,kappa_ddot))/(powf(vmag(kappa),3.0f))+3.0f*powf(vdot(kappa,kappa_dot),2.0f)/powf(vmag(kappa),5.0f),kappa));
//     // struct vec b2d_ddot = vadd3(vdiv(nu_ddot,vmag(nu)),vscl(-2.0f*vdot(nu,nu_dot)/(powf(vmag(nu),3.0f)),nu_dot),
//     //                             vscl(-1.0f*(powf(vmag(nu_dot),2.0f)+vdot(nu,nu_ddot))/(powf(vmag(nu),3.0f))+3.0f*powf(vdot(nu,nu_dot),2.0f)/powf(vmag(nu),5.0f),nu));


//     struct vec b3d_ddot = vnormalize2diff(kappa,kappa_dot,kappa_ddot);
//     struct vec nu_ddot  = vadd3(vcross(b3d_ddot,b1r),vcross(b3d,b1r_ddot),vscl(2.0f,vcross(b3d_dot,b1r_dot)));
//     struct vec b2d_ddot = vnormalize2diff(nu,nu_dot,nu_ddot);
//     struct vec b1d_ddot  = vadd3(vcross(b2d_ddot,b3d),vcross(b2d,b3d_ddot),vscl(2.0f,vcross(b2d_dot,b3d_dot)));
//     struct mat33 Rd_ddot = mcolumns(b1d_ddot,b2d_ddot,b3d_ddot);
//     // Rd_dot = mdiag(1.0f,1.0f,1.0f);
//     omega_d = mvee(mmul(mtranspose(Rd),Rd_dot));
//     omega_dot_d = vsub(mvee(mmul(mtranspose(Rd),Rd_ddot)),mkvec(omega_d.y*omega_d.z,omega_d.x*omega_d.z,omega_d.y*omega_d.x));
//     // DEBUG_PRINT("omega_dot_d = [%f, %f,%f]\n",(double)omega_dot_d.x,(double)omega_dot_d.y,(double)omega_dot_d.z);
//     // DEBUG_PRINT("Omega_d: [%f, %f, %f]\n", (double)omega_d.x, (double)omega_d.y, (double)omega_d.z);
//     // DEBUG_PRINT("Omega_dot_d: [%f, %f, %f]\n", (double)omega_dot_d.x, (double)omega_dot_d.y, (double)omega_dot_d.z);

//   }

  
//   if (setpoint->mode.z == modeDisable) {
//     control->thrustSi = 0.0f;
//     control->torqueX =  0.0f;
//     control->torqueY =  0.0f;
//     control->torqueZ =  0.0f;
//   } else {

// // //  Attitude Control sliding mode control
//     // Rd = meye();
//     // omega_dot_d = vzero();
//     // omega_d= vzero();
//     // force =0.0f;
//     omega_d= vscl(omega_scale,omega_d);
//     omega_dot_d = vscl(omega_scale,omega_dot_d);
//     struct quat q_d = mat2quat(Rd);

//     // creating gain matrix 
//     struct mat33 lambda_q = mdiag(lambda_q_x,lambda_q_y,lambda_q_z);
//     struct mat33 K_q = mdiag(K_q_x,K_q_y,K_q_z);   
//     struct mat33 rho_q = mdiag(rho_q_x,rho_q_y,rho_q_z); 
//     // computing the angular velocity and quaternion errors
//     omega_e = vsub(omega, omega_d);
//     // struct quat q_e = qqmul(qinv(q_d),attitude);
//     q_e = qqmul(qinv(q_d),attitude);

//     // computing the quaternion dot error
//     struct quat q_e_dot = qscl(0.5,qqmul(q_e,mkquat(omega_e.x,omega_e.y,omega_e.z,0.0f)));
//     // q_e_dot = qscl(0.5,qqmul(q_e,mkquat(omega_e.x,omega_e.y,omega_e.z,0.0f)));
//     // computing the sliding surface
//     struct vec s_q = vadd(omega_e , mvmul(lambda_q, vscl(sgnplus(q_e.w),mkvec(q_e.x,q_e.y,q_e.z))));

//     // computing the Moments
//     // Moment = vadd4(mvmul(J,omega_dot_d) ,
//     //                     vcross(omega,mvmul(J,omega)),
//     //                     vneg(mvmul(mmul(J,lambda_q), vscl(sgnplus(q_e.w),mkvec(q_e_dot.x,q_e_dot.y,q_e_dot.z)))),
//     //                     vneg(mvmul(mmul(J,K_q),vtanhf(vscl(rho_q,s_q)))));


//     Moment = vadd4(mvmul(J,omega_dot_d),
//                    vcross(omega,mvmul(J,omega)),
//                    vneg(mvmul(mmul(J,lambda_q), vscl(sgnplus(q_e.w),mkvec(q_e_dot.x,q_e_dot.y,q_e_dot.z)))),
//                    vneg(mvmul(mmul(J,K_q),vtanhf(mvmul(rho_q,s_q)))));
    
//     //updating the Force and Moments
//     control->thrustSi = force;
//     control->torqueX = Moment.x;
//     control->torqueY = Moment.y;
//     control->torqueZ = Moment.z;
//     // DEBUG_PRINT("Force: %f\n", (double)force);
//     // DEBUG_PRINT("Moment: [%f, %f, %f]\n", (double)Moment.x, (double)Moment.y, (double)Moment.z);
//   }

//   // Setting up the Control Mode
//   control->controlMode = controlModeForceTorque;
// }

// bool controllerOutOfTreeTest(void) {
//   return true;
// }

// // Parameter setup
// PARAM_GROUP_START(ctrlQSMC)
// PARAM_ADD(PARAM_FLOAT, lambda_att_x, &lambda_q_x)
// PARAM_ADD(PARAM_FLOAT, lambda_att_y, &lambda_q_y)
// PARAM_ADD(PARAM_FLOAT, lambda_att_z, &lambda_q_z)
// PARAM_ADD(PARAM_FLOAT, K_att_x, &K_q_x)
// PARAM_ADD(PARAM_FLOAT, K_att_y, &K_q_y)
// PARAM_ADD(PARAM_FLOAT, K_att_z, &K_q_z)
// PARAM_ADD(PARAM_FLOAT, lambda_pos_x, &lambda_xi_x)
// PARAM_ADD(PARAM_FLOAT, lambda_pos_y, &lambda_xi_y)
// PARAM_ADD(PARAM_FLOAT, lambda_pos_z, &lambda_xi_z)
// PARAM_ADD(PARAM_FLOAT, K_pos_x, &K_xi_x)
// PARAM_ADD(PARAM_FLOAT, K_pos_y, &K_xi_y)
// PARAM_ADD(PARAM_FLOAT, K_pos_z, &K_xi_z)
// PARAM_ADD(PARAM_FLOAT, rho_xi_x, &rho_xi_x)
// PARAM_ADD(PARAM_FLOAT, rho_xi_y, &rho_xi_y)
// PARAM_ADD(PARAM_FLOAT, rho_xi_z, &rho_xi_z)
// PARAM_ADD(PARAM_FLOAT, rho_q_x, &rho_q_x)
// PARAM_ADD(PARAM_FLOAT, rho_q_y, &rho_q_y)
// PARAM_ADD(PARAM_FLOAT, rho_q_z, &rho_q_z)
// PARAM_ADD(PARAM_FLOAT, omega_scale, &omega_scale)
// PARAM_GROUP_STOP(ctrlQSMC)


// // Logging info setup
// LOG_GROUP_START(ctrlLog)
// LOG_ADD(LOG_FLOAT, qe_x, &q_e.x)
// LOG_ADD(LOG_FLOAT, qe_y, &q_e.y)
// LOG_ADD(LOG_FLOAT, qe_z, &q_e.z)
// LOG_ADD(LOG_FLOAT, qe_w, &q_e.w)
// LOG_ADD(LOG_FLOAT, a_x, &xi_2dot_des.x)
// LOG_ADD(LOG_FLOAT, a_y, &xi_2dot_des.y)
// LOG_ADD(LOG_FLOAT, a_z, &xi_2dot_des.z)
// LOG_ADD(LOG_FLOAT, ae_x, &a_e.x)
// LOG_ADD(LOG_FLOAT, ae_y, &a_e.y)
// LOG_ADD(LOG_FLOAT, ae_z, &a_e.z)
// LOG_ADD(LOG_FLOAT, j_x, &xi_3dot_des.x)
// LOG_ADD(LOG_FLOAT, j_y, &xi_3dot_des.y)
// LOG_ADD(LOG_FLOAT, j_z, &xi_3dot_des.z)
// LOG_ADD(LOG_FLOAT, je_x, &j_e.x)
// LOG_ADD(LOG_FLOAT, je_y, &j_e.y)
// LOG_ADD(LOG_FLOAT, je_z, &j_e.z)
// // LOG_ADD(LOG_FLOAT, omega_ex, &omega_e.x)
// // LOG_ADD(LOG_FLOAT, omega_ey, &omega_e.y)
// // LOG_ADD(LOG_FLOAT, omega_ez, &omega_e.z)
// // LOG_ADD(LOG_FLOAT, s_att_x, &s_q.x)
// // LOG_ADD(LOG_FLOAT, s_att_y, &s_q.y)
// // LOG_ADD(LOG_FLOAT, s_att_z, &s_q.z)
// // LOG_ADD(LOG_FLOAT, s_pos_x, &s_xi.x)
// // LOG_ADD(LOG_FLOAT, s_pos_y, &s_xi.y)
// // LOG_ADD(LOG_FLOAT, s_pos_z, &s_xi.z)
// LOG_ADD(LOG_FLOAT, moment_x, &Moment.x)
// LOG_ADD(LOG_FLOAT, moment_y, &Moment.y)
// LOG_ADD(LOG_FLOAT, moment_z, &Moment.z)
// LOG_ADD(LOG_FLOAT, force, &force)
// LOG_GROUP_STOP(ctrlLog)


















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
#include "quaternion_smc_controller.h"

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


static float kh = 1.0f;

// static float omega_scale = 1.0f;


// static struct vec s_q;
// static struct vec s_xi;
static struct vec Moment;
static float force;
static struct quat q_e;
static struct quat q_d;
static struct vec snap_des ;
static struct vec jerk_des_old ;
static struct vec jerk_des ;
static struct vec acc_des_old ;
static struct vec acc_des ;
static struct vec acc_old ;

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
  acc_des_old = vzero();
  counter = 0.0f;
  jerk_des_old = vzero();
  acc_old = vzero();
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
    static struct vec j_e ;
    static struct vec e3 ;
    static struct vec jerk ;
    static struct vec b1r ;
    static struct vec b1r_dot ;
    static struct vec b1r_ddot ;
    static struct mat33 lambda_xi;
    static struct mat33 K_xi ;
    static struct mat33 rho_xi ;
    static struct vec s_xi;
    static struct vec rho_mvmult_s_xi ;
    static struct vec kappa ;
    static struct vec b3d ;
    static struct vec nu  ;
    static struct vec b2d ;
    static struct vec b1d ;
    static struct vec sdot_xi ;
    static struct mat33 Kmultrho_xi;
    static struct vec kappa_dot;
    static struct vec b3d_dot;;
    static struct vec nu_dot ;
    static struct vec b2d_dot;
    static struct vec b1d_dot  ;
    static struct mat33 Rd_dot;
    static struct vec sddot_xi ;
    static struct mat33 Kmultrho2_xi_2;
    static struct vec kappa_ddot;
    static struct vec b3d_ddot ;
    static struct vec nu_ddot ;
    static struct vec b2d_ddot ;
    static struct vec b1d_ddot  ;
    static struct mat33 Rd_ddot ;
    static struct mat33 Rd;
    static struct vec omega_dot_d;
    static struct vec omega_d;
    // static struct vec omega_dot_d_b;
    static struct vec omega_d_b;
    static struct vec omega_e;
    static struct mat33 Rt;
    static struct mat33 Rdt;
    static struct mat33 omega_d_b_hat;
    static struct mat33 neg_omega_hat;


    static struct mat33 lambda_q ;
    static struct mat33 K_q ;   
    static struct mat33 rho_q ; 
    static struct quat q_e_dot ;
    static struct vec s_q ;
    static float vh ;


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

    struct vec acc = mkvec(state->acc.x ,
                           state->acc.y ,
                           state->acc.z );

    struct vec velocity = mkvec(state->velocity.x,
                         state->velocity.y,
                         state->velocity.z );
    
    acc_des = mkvec(setpoint->acceleration.x ,setpoint->acceleration.y ,setpoint->acceleration.z);
   // xi_dddot_des = mkvec(setpoint->jerk.x ,setpoint->jerk.y ,setpoint->jerk.z);
    // j_e = vzero();
    jerk_des = compute_derivative(acc_des,acc_des_old);
    snap_des = compute_derivative(jerk_des,jerk_des_old);

    jerk = compute_derivative(acc, acc_old);
    j_e = vsub(jerk, jerk_des);


    e3 = mkvec(0.0f,0.0f,1.0f);


    b1r = mkvec(cosf(psi_d),sinf(psi_d),0.0f);
    b1r_dot = mkvec(-sinf(psi_d)*psi_dot_d,cosf(psi_d)*psi_dot_d,0.0f);
    b1r_ddot =mkvec(-cosf(psi_d)*powf(psi_dot_d,2.0),-sinf(psi_d)*powf(psi_dot_d,2.0f),0.0f);

    // creating gain matrix 
    lambda_xi = mdiag(lambda_xi_x,lambda_xi_y,lambda_xi_z);
    K_xi = mdiag(K_xi_x,K_xi_y,K_xi_z);
    rho_xi = mdiag(rho_xi_x,rho_xi_y,rho_xi_z);


    

    s_xi = vadd(v_e , mvmul(lambda_xi, xi_e));
    rho_mvmult_s_xi = mvmul(rho_xi,s_xi);
    kappa = vscl(mass,vadd4(acc_des,vneg(mvmul(lambda_xi, v_e)),vscl(gravity,e3),vneg(mvmul(K_xi,vtanhf(rho_mvmult_s_xi)))));


    force = vdot(kappa,mvmul(R,e3));
    vh = vdot(velocity,vadd(mcolumn(R,0),mcolumn(R,1)));
    force = force + kh * powf(vh,2.0f);
    force = clamp(force,0.04f,0.6f);

// Trajectory generation for the attiude controller
    b3d = vnormalize(kappa);
    nu  = vcross(b3d,b1r);
    b2d = vnormalize(nu);
    b1d  = vcross(b2d,b3d);
    Rd = mcolumns(b1d,b2d,b3d);


    // first derivative 
    sdot_xi = vadd(a_e , mvmul(lambda_xi, v_e));
    Kmultrho_xi = mdiag(K_xi_x*rho_xi_x,K_xi_y*rho_xi_y,K_xi_z*rho_xi_z);
    kappa_dot = vscl(mass,vadd3(jerk_des,
                                vneg(mvmul(lambda_xi, a_e)),
                                vneg(mvmul(Kmultrho_xi,
                                veltmul(vsech2f(rho_mvmult_s_xi),sdot_xi)))));


    b3d_dot = vnormalize1diff(kappa,kappa_dot);
    nu_dot  = vadd(vcross(b3d_dot,b1r),vcross(b3d,b1r_dot));
    b2d_dot =  vnormalize1diff(nu,nu_dot);
    b1d_dot  = vadd(vcross(b2d_dot,b3d),vcross(b2d,b3d_dot));
    Rd_dot = mcolumns(b1d_dot,b2d_dot,b3d_dot);

    // second derivative 
    sddot_xi = vadd(j_e , mvmul(lambda_xi, a_e));
    Kmultrho2_xi_2 = mdiag(2.0f*K_xi_x*rho_xi_x*rho_xi_x,2.0f*K_xi_y*rho_xi_y*rho_xi_y,2.0f*K_xi_z*rho_xi_z*rho_xi_z);
    kappa_ddot = vscl(mass,vadd4(mvmul(Kmultrho2_xi_2,veltmul3(vsech2f(rho_mvmult_s_xi),vtanhf(rho_mvmult_s_xi),vpow(sdot_xi,2.0f))),
                                 vneg(mvmul(Kmultrho_xi,veltmul(vsech2f(rho_mvmult_s_xi),sddot_xi))),
                                 vneg(mvmul(lambda_xi, j_e)),
                                 snap_des
                                ));  

    b3d_ddot = vnormalize2diff(kappa,kappa_dot,kappa_ddot);
    nu_ddot  = vadd3(vcross(b3d_ddot,b1r),vcross(b3d,b1r_ddot),vscl(2.0f,vcross(b3d_dot,b1r_dot)));
    b2d_ddot = vnormalize2diff(nu,nu_dot,nu_ddot);
    b1d_ddot  = vadd3(vcross(b2d_ddot,b3d),vcross(b2d,b3d_ddot),vscl(2.0f,vcross(b2d_dot,b3d_dot)));
    Rd_ddot = mcolumns(b1d_ddot,b2d_ddot,b3d_ddot);
    // Rd_dot = mdiag(1.0f,1.0f,1.0f);
    Rt = mtranspose(R);
    Rdt = mtranspose(Rd);
    omega_d_b = mvee(mmul(Rdt,Rd_dot));
    omega_d = mvmul(mmul(Rt,Rd),omega_d_b);
    omega_d_b_hat = vhat(omega_d_b);
    neg_omega_hat = vhat(vneg(omega));
    // omega_dot_d = vsub(mvee(mmul(mtranspose(Rd),Rd_ddot)),mkvec(omega_d.y*omega_d.z,omega_d.x*omega_d.z,omega_d.y*omega_d.x));
    omega_dot_d = mvee(msub(mmul(Rdt,Rd_ddot),mmul(omega_d_b_hat,omega_d_b_hat)));
    // omega_dot_d =  mvmul(madd(mmul3(neg_omega_hat,Rt,Rd),mmul(Rt,Rd_dot)),omega_d_b);
    // omega_dot_d = vadd(omega_dot_d, mvmul(mmul(Rt,Rd),omega_dot_d_b));

    acc_des_old = acc_des;
    jerk_des_old = jerk_des;
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
    Rd = meye();
    omega_dot_d = vzero();
    omega_d= vzero();
    force =0.0f;

    q_d = mat2quat(Rd);

    // creating gain matrix 
    lambda_q = mdiag(lambda_q_x,lambda_q_y,lambda_q_z);
    K_q = mdiag(K_q_x,K_q_y,K_q_z);   
    rho_q = mdiag(rho_q_x,rho_q_y,rho_q_z); 
    // computing the angular velocity and quaternion errors
    omega_e = vsub(omega, omega_d);
    q_e = qqmul(qinv(q_d),attitude);

    // computing the quaternion dot error
    q_e_dot = qscl(0.5,qqmul(q_e,mkquat(omega_e.x,omega_e.y,omega_e.z,0.0f)));
    // computing the sliding surface
    s_q = vadd(omega_e , mvmul(lambda_q, vscl(sgnplus(q_e.w),mkvec(q_e.x,q_e.y,q_e.z))));

    // computing the Moments

    Moment = vadd4(mvmul(J,omega_dot_d),
                   vcross(omega,mvmul(J,omega)),
                   vneg(mvmul(mmul(J,lambda_q), vscl(sgnplus(q_e.w),mkvec(q_e_dot.x,q_e_dot.y,q_e_dot.z)))),
                   vneg(mvmul(mmul(J,K_q),vtanhf(mvmul(rho_q,s_q)))));
    
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
PARAM_ADD(PARAM_FLOAT, kh, &kh)
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
LOG_ADD(LOG_FLOAT, ad_x, &acc_des.x)
LOG_ADD(LOG_FLOAT, ad_y, &acc_des.y)
LOG_ADD(LOG_FLOAT, ad_z, &acc_des.z)
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

