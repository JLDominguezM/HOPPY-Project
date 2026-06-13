/*
This is the basic starter code for Hoppy's Leg

PLEASE NOTE! This is an example code that is NOT meant to run on the leg. It is meant to show the use of the program
While several functions and tools are included, many are not used in the simple example, but will probably be useful
when making the leg hop. This code should be used to test the motor functionality and as a starting place for writing
further code. The included run is a PD loop stepping the two   !!   DISCONNECTED   !!    motors back and forth.
Please clamp the motors down before testing the code! BE SAFE!

The code is broken into intuitive function blocks. The control loop runs at 1 kHz
Each function the user needs is listed below and well documented. All units should be NMS and rads
The starting point is the following function. This calls everything important.
If you are just gettings started,   !!!   START HERE   !!! and follow the code path down.

    DoEveryMilliSecond_CPU1                 - Line #368


At each time step the following functions are called:
    calculate_traj                          - Line # 208
    update_states                           - Line # 289    
    calculate_control();                    - Line # 322
    prevent_saturation();                   - Line # 342


Additional support functions include:
    get_avg_vel                             - Line # 241
    calculate_traj                          - Line # 208
    init_values                             - Line # 196
    get_phase                               - Line # 180    
    update_jacobian                         - Line # 272    
    pose_to_joint_space                     - Line # 281
    ADCD_ISR                                - Line # 620
    main                                    - Line # 410

OTHER
    - Update Encoder Positions. Search for this line. It will bring you to the point in the main function where
      the encoders are initilazized and offsets are set. This is important for zeroing the home position
                                            - Line # 507

    - Setup contact input. The code automatically reads the analog and the digital contact switch. 
      Further instructions are there. EDIT THIS FUNCTION BASED ON WHAT CONTACT METHOD YOU USE
                                            - Line # 180

    - General Declariation of Variables
                                            - Line # 107-175


If you have any questions, please contact Kevin Murphy or Joao Ramos

Enjoy Hoppy :)

@AUTHOR KEVIN MURPHY, Dan Block
kfmurph2@illinois.edu
*/

#include <math.h>

#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/c28/Hwi.h>

#include "F28x_Project.h"
#include "F2837xD_device.h"
#include "F2837xD_Ipc_drivers.h"

/***************************** Start Customized functions **************************/
#include "LCD.h"
#include "F28379D_Serial.h"
#include "F28379D_EPwm.h"     // for PWM signal
#include "F28379D_EQep.h"


//#include "driverlib.h"
//#include "device.h"
/*****************************  End Customized functions  **************************/

/*****************************    Start defined Values    **************************/
#define PI       3.1415926535897932384626433832795
#define TWOPI    6.2831853071795864769252867665590
#define HALFPI   1.570796326794897
/*****************************     End defined Values     **************************/

/*****************************  Start Simunlink plotting  **************************/
int adcb0result = 0; // 16-bit
int adcb1result = 0; // 16-bit
long read_1 = 0; // 32-bit
long read_2 = 0; // 32-bit
void simulink_serial_RXD(serial_t *s, char data);
void pose_to_joint_space(void);   // prototype: calculate_traj calls it before its definition
#ifdef _FLASH
// These are defined by the linker (see device linker command file)
extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadSize;
extern unsigned int RamfuncsRunStart;
#endif
extern eqep_t eqep1;
extern eqep_t eqep2;
/*****************************   End Simunlink plotting   **************************/

/****************************************************** OUR ADDITION START **********************************************************/
/******************************************************  User Definitions  **********************************************************/
// int16_t adcd0result = 0;
// int16_t adcd1result = 0;
int16_t analog_in[4] = {0,0,0,0};   // 16 bit analog between 0-3 V.
                                    // should have following index
                                    // [0] = touch sensor
                                    // [1] = optional th_1 encoder
                                    // [2] = optional th_1 encoder
                                    // [3] = open for user input

int16_t analog_limit = 2048;        // this is a tunable value that use used to measure analog linear contact
                                    // the value should be between 0-4096

//Physical dimensions
float LH = .0960;               // hip length
float DK = .0520;               // Knee offset
float LK = .1550;               // length of shank          !this values can be adjusted by the user!
float LKF;                      // shank effective length
float LHF;                      // Distance between hip and foot

//gains TODO TUNABLE VALUES
float Kp_a[2] = { 3000.,  3000.};   // Positional Gain for motor 1 and motor 2 in aerial phase
float Kd_a[2] = {  200.,   200.};   // Derivative Gain for motor 1 and motor 2 in aerial phase

float Kp_s = 60000.;                // Cartesian Positional Gain for motor 1 and motor 2 in stance phase
float Kd_s = 1000.;                 // Derivative Gain for motor 1 and motor 2 in stance phase

//control effort
float u[2]          = {0.0,  0.0};  // control effort for motor 1 and motor 2

//Joint States
float q_now [2]     = {0.0,  0.0};  // Position of the leg at Current time
float q_prev[2]     = {0.0,  0.0};  // Position of the leg at previous time step
float q_ref [2]     = {0.0,  0.0};  // Reference position for aerial
float err_q [2]     = {0.0,  0.0};  // Positional joint Error at current time step

float qdot_now [2]  = {0.0,  0.0};  // Current Joint Velocities
float qdot_prev[2]  = {0.0,  0.0};  // Previous Joint Velocities

//Cartesian States
float pos_des [2] = {0.0,   0.0};   // desired Cartesian position
float pos_now [2] = {0.0,   0.0};   // Current Cartesian position
float err_pos [2] = {0.0,   0.0};   // Positional Error at current time step
float vel_des [2] = {0.0,   0.0};   // desired Cartesian velocity
float vel_err [2] = {0.0,   0.0};   // Cartesian velocity error

// velocity filtering
int horizon = 3;
float vel_avgs[2][3];//YOU MUCH COPY THE HORIZON VALUE TO THE COL(2nd) ARRAY DIMENSION

//Switches
int phase = 1;  // Flag to tell which controller.
                //0 = aerial phase, 1 = stance phase

// Encoders
float enc_offset[2] = {0.0,0.0};    // offset for zeroing the encoders. Initialized in the original read,
                                    //so dont touch HERE. Check out  "Update Encoder Positions" in the Main function

//control Values
float T = 0.001;                    // sampling period in second. This should remain const
float J [2][2];                     //Jacobian

//Misc
long int cnt        = 0;            // time counter

// Temp trajectory vars. These are used in the given example trajectory. You can delete
int posref1dir = 0; // TODO comment out later - just used in initial demo
int posref2dir = 0; // TODO comment out later - just used in initial demo

/****** SLOW LEG TEST - hold it in the air ******/
int   MOTORS_OFF = 1;            // 1 = only checks the IK (q_ref) without moving; 0 = moves the leg
                                 // (starts at 1 for safety: set it to 0 LIVE in Expressions)
// ===== goBILDA MOTOR MODEL (paper Eq.18; used by prevent_saturation) =====
// u[] is REAL TORQUE in N*m and prevent_saturation converts it to voltage:
// V = Rw/(kT*N)*u + kv*N*qdot_f. The old example mapping (10/64.125) treated
// "64 N*m" as full pwm, but physically full pwm = 12 V = ~3.4 N*m at stall
// -> it sent ~5% of the voltage for the real MATLAB torques (the push was not felt).
float Rw = 1.3;                  // armature resistance (ohm)
float kT = 0.0135;               // torque constant (N*m/A)
float kv = 0.0186;               // back-EMF constant (V*s/rad)
float NH = 26.9;                 // hip reduction (rotor -> shaft = units of q_now[0])
float NK = 28.8;                 // EFFECTIVE knee reduction in units of e (the 806.4 scale
                                 // of eqep2 makes the physical 26.9 read as 28.8 = the PDF's N_K)
float VMAX = 12.0;               // driver bus: pwm 10 = 12 V
// Bench PD gains, NOW IN N*m/rad: {20.9, 30.9} = the EXACT equivalents of the
// {400, 600} verified on the bench with the old mapping (same pwm per radian of error).
float Kp_test[2] = {20.9, 30.9};
float Kd_test[2] = { 0.0,  0.0}; // D = 0; with the qdot_f filter it can be raised (0.26 = the old "5")
float u_lim_test = 4.0;          // pseudo-pwm limit (of 10). Raise it live if the knee falls short
float ramp_rate  = 0.15;         // rad/s -> VERY slow (no longer used in flight)
// ===== REAL LEG MORPHOLOGY (clarified, see HANDOFF_FIRMWARE.md) =====
// The leg is BIRD-TYPE (mirrored from the original HOPPY): the "knee" points BACK and
// FLEXING the knee (more negative e) drives the foot FORWARD (+x). Also the tube is
// NEVER aligned with the thigh: at full extension it already sits ~46 deg forward.
// ZERO POSE (reachable and repeatable): THIGH VERTICAL (plumb it with a phone on the plates)
// + knee at its EXTENSION STOP (no spring fighting). There QPOSCNT=0 is written.
// HIP: +q0 = thigh forward. Zero offset = 0 (thigh vertical AT zero).
float beta_off   = 0.0;
// KNEE - FOUR-BAR MAP:  q1_FK = KA*e^2 + KB*e + KC,  e = q_now[1] (encoder).
// q1_FK = effective knee->foot angle relative to the thigh, FORWARD positive. Flexing
// (e<0) INCREASES q1_FK. |KA|,|KB| = shape measured in the 4-point calibration (the effective
// ratio runs ~1.5 extension -> ~0.7 flexion); sign flipped by the mirrored morphology.
// KC = q1_FK at zero (~+0.80 = the ~46 deg of the tube + the foot's DK offset): TUNE with a plumb line.
float KA = -0.454;
float KB = -1.534;
float KC = 0.80;
float q_fk[2] = {0.0, 0.0};   // REAL FK angles (for kinematics/Jacobian); q_now stays in encoder units
// Flight trajectory, tunable LIVE (calculate_traj recomputes pos_des every ms with THESE):
float traj_amp   = 0.00;    // tangential sweep amplitude (m). 0 = foot STILL at traj_x0
                            // (starts at 0 = calibration mode; raise to 0.05 LIVE for the sweep)
float traj_x0    = 0.00;    // sweep center (m). + = forward
float traj_depth = -0.22;   // foot depth below the hip (m)
long  traj_T     = 8000;    // cycle period (ms)
// DIRECT JOINT MODE (calibration): q_ref = q_test as-is (ENCODER units), no IK.
// To measure the knee four-bar effective ratio with the hip held still.
int   JOINT_MODE = 0;
float q_test[2]  = {0.0, 0.0};
float wp_hold    = 2.0;          // s at each pose
#define NWP 2
float wp[NWP][2] = {
  { 0.000,  0.000},   // home (leg extended)
  { 0.524, -1.000},   // crouch: hip +30 deg, knee -57 deg (moves both together)
};
int   wp_i = 0;  float t_wp = 0.0;

/****** STAGE 2 - STANCE CONTROL (bench): Bezier GRF push  u = -J^T*[Fx;Fz] + soft PD ******/
// Faithful port of the paper's Eq.19 (controller.py / Simulator_MATLAB, see mujoco/CONTROL_FORWARD.md).
// The push lasts Tst and the force profile [Fx;Fz] (N, hip frame: x forward, z up) is a
// 4th-order Bezier. The torque comes out in FK space and is mapped to the motor: hip 1:1,
// knee MULTIPLIED by dq1_FK/de = 2*KA*e+KB (four-bar). MOTORS_OFF still has the final say.
int   STANCE_TEST = 0;        // 1 = enables stance mode (the triggers below)
int   st_go     = 0;          // set 1 LIVE = fires ONE push (auto-clears)
int   st_auto   = 0;          // 1 = cyclic push every st_period ms
long  st_period = 3000;       // auto-mode period (ms)
int   st_sensor = 0;          // 1 = fires on STEPPING on the foot sensor (0->1 edge of phase)
float Tst      = 0.35;        // push duration (s). Dry-run: raise to 5.0 and watch F_des/u_st
float fz_scale = 1.0;         // vertical profile scale (real Bezier peak ~42 N at s=0.5)
float fx_scale = 1.0;         // tangential scale. Its SIGN sets the travel direction (Stage 3)
float Fz_bz[5] = {0.0, 20.0, 100.0, 0.0, 0.0};   // control points (= MATLAB/sim)
float Fx_bz[5] = {0.0,  0.0, -25.0, 0.0, 0.0};
float Kp_st = 0.03;           // soft stance PD in FK space (= MATLAB; a regularizer)
float Kd_st = 0.08;
float blend_ms = 10.0;        // flight->stance blend (Eq.20) so no torque step appears
int   st_active = 0;          // (state) 1 while pushing
float st_t = 0.0;             // (state) time within the push (s)
float qd_st_fk[2] = {0.0, 0.0};  // FK pose captured at push start (soft PD reference)
float F_des[2]  = {0.0, 0.0};    // current Bezier [Fx;Fz] (N) - watch in Expressions
float tau_fk[2] = {0.0, 0.0};    // stance torque in FK space (N*m)
float u_air[2]  = {0.0, 0.0};    // flight PD torque (motor space)
float u_st[2]   = {0.0, 0.0};    // stance torque in MOTOR space (N*m) - watch in dry-run
int   phase_prev = 0;            // for the sensor rising edge (st_sensor)
// Velocity filter (= sim Stage 5, lambda=10 rad/s): lets us use Kd without buzz.
float vel_lambda = 10.0;      // bandwidth (rad/s). Raise LIVE (20-30) if the D term is slow
float qdot_f[2]  = {0.0, 0.0};   // filtered qdot (encoder space)
float qfk_dot[2] = {0.0, 0.0};   // velocity in FK space (knee = f'(e)*qdot_f[1])
// PEAKS of the last push (reset on trigger and FROZEN when it ends:
// read them later at leisure, without fighting the Expressions refresh)
float pk_Fz = 0.0;            // max commanded F_des[1] (N)
float pk_pwm[2] = {0.0, 0.0}; // max FINAL |pwm| sent to each motor (post-saturation)
float pk_e_min = 0.0;         // min/max of q_now[1] during the push: how far the knee
float pk_e_max = 0.0;         // TRAVELED (pk_e_max near 0 = it hit its extension stop)

/****** STAGE 3 - JUMP_MODE: continuous hopping by foot sensor (flight<->stance FSM) ******/
// FLIGHT: the PD brings the foot to the landing pose (traj_x0, traj_depth) - FIXED placement
//   (no boom encoder means no Raibert; travel is set by Fx, like the sim with vx_d=0).
// TOUCHDOWN: 0->1 edge of the foot sensor with t_air >= lo_debounce -> GRF push (Stage 2).
// LIFTOFF: sensor released after td_min_stance (the analog of the paper's "GRF < threshold"),
//   or end of Tst -> back to flight and the PD pulls the leg up.
// MOTORS_OFF and u_lim_test still have the final say. Without a counterweight it barely takes
// off ("little bounces"), but the full sensor->push->retract->fall->sensor cycle still validates.
int   JUMP_MODE = 0;          // 1 = continuous hopping (safe defaults: starts off)
float lo_debounce  = 0.04;    // min s in flight before accepting touchdown (debounce)
float td_min_stance = 0.05;   // min s in stance before allowing early liftoff
long  n_hops = 0;             // counter of triggered hops
float t_air = 0.0;            // current time in flight (s)
float t_air_last = 0.0;       // DURATION OF THE LAST FLIGHT (s) - apex height ~ g*t^2/8
float t_st_last  = 0.0;       // real duration of the last stance (s) - to tune Tst
/****************************************************** OUR ADDITION END **********************************************************/


/****************************************************** Start Supporting Functions **********************************************************/
void get_phase(void)
{
    /*This function reads the contact switch to determine phase.
     * if using additional encoders on th_1 and th_2, the user can
     * take the state readings and determine if the ground is being contacted here instead
     * */

    //USE THE FOLLOWING LINE IF YOU ARE USING A BINARY CONTACT SWITCH
    // phase =  GpioDataRegs.GPADAT.bit.GPIO9; // standing
    
    // The example's ADCD ISR is NOT hooked to the RTOS (it does not run), so analog_in never
    // changed. We read the ADC result DIRECTLY (the conversion is triggered every ms at the end
    // of DoEveryMilliSecond). Foot sensor on ADCIN-D0.
    analog_in[0] = AdcdResultRegs.ADCRESULT0;

    //Use the following line if you are using an analog linear potentiometer
    phase = (analog_in[0] >= analog_limit);
    
    //If you are calculating contact based off of TH1 and TH2, do so here 
    // phase = f(th1, th2, th3, th4);
}
void init_values()
{
    /* This function defines any variable that has a function used in its declaration
     * There are some tunable values in here
     *
     * CCS compiler does not allow this to happen globally, so must be done in a function
     * which is big stoopid. But I digress
     * */

    LKF = sqrt(DK*DK + LK*LK);   // = 0.16349  (the example left it UNASSIGNED = 0, a BUG)
}
void calculate_traj(void)
{
    /*This function is used to determine the trajectory. This should be the first thing done in the timed loop
     * as it only depends on the discrete time. This function will eventually determine the phase (aerial or stance)
     * and choose the correct position required for that time in that phase*/

    // DIRECT JOINT MODE: raw joint targets (encoder), to calibrate the four-bar.
    if (JOINT_MODE) {
        q_ref[0] = q_test[0];
        q_ref[1] = q_test[1];
        return;
    }

    // FLIGHT: place the foot at pos_des (Cartesian, hip frame) via IK -> q_ref.
    // We cycle the TANGENTIAL foot position back and forth (slowly), at a fixed depth, to see
    // the foot placement (what flight control does in the air). Tunable LIVE via
    // traj_amp / traj_x0 / traj_depth / traj_T (pos_des is recomputed every ms, do NOT edit it directly).
    // traj_amp = 0 -> foot STILL at (traj_x0, traj_depth): the mode for calibrating beta_off.
    float ang = (cnt % traj_T) * (TWOPI / (float)traj_T);
    pos_des[0] = traj_x0 + traj_amp * sin(ang);   // tangential (forward/back)
    pos_des[1] = traj_depth;                      // depth below the hip

    pose_to_joint_space();             // IK: pos_des -> q_ref (FK convention: q=0 = leg straight down)
    q_ref[0] -= beta_off;              // hip: FK -> encoder (calibrated offset = 0)
    {   // knee: FK -> encoder = inverse of the four-bar map (branch e<=0 with KA,KB<0)
        float disc = KB*KB + 4.0f*KA*(q_ref[1] - KC);
        if (disc < 0.0f) disc = 0.0f;  // target more flexed than the map's reach
        q_ref[1] = (-KB - sqrt(disc))/(2.0f*KA);
    }
}

void get_avg_vel(float raw_vel_h, float raw_vel_k)
{
 /* This function puts the velocity measurements on the queue and finds the average over the past finite horizon.
  * This function should be passed the instantanious raw velocity and it automatically saves the average volocity
  * in the global variable. This is a simple example for data filtering
  */
    float sum_h = 0;
    float sum_k = 0;
    int i = 0;
    for (i = 0; i < horizon; i++)
    {// calculate average
        sum_h += vel_avgs[0][i];//hip
        sum_k += vel_avgs[1][i];//knee
    }
    sum_h += raw_vel_h;//hip
    sum_k += raw_vel_k;//knee

    i = 0;
    for (i = 0; i < horizon-1; i++)
    {   //pop and shift the queue
        vel_avgs[0][i] = vel_avgs[0][i+1];//knee
        vel_avgs[1][i] = vel_avgs[1][i+1];//hip
    }
    //add new value to end of the queue
    vel_avgs[0][horizon-1] = sum_h/(horizon+1);//knee
    vel_avgs[1][horizon-1] = sum_k/(horizon+1);//hip

    //save the joint velocities into global varibles
    qdot_now[0] = sum_h/(horizon+1);
    qdot_now[1] = sum_k/(horizon+1);
}
void update_jacobian(void)
{
    /*This function is given and will be useful when calculating anything in task space. This finds the jacobian*/
     // update jacobian
    // With the REAL FK angles (q_fk), not the encoder ones. For u=-J^T*F the knee torque is
    // also MULTIPLIED by d(q1_FK)/de = 2*KA*e+KB (virtual work: tau_m*de = tau_FK*dq1
    // -> tau_m = tau_FK*f'(e); since f' is negative, the motor sign comes out right on its own).
    J[0][0] = LH*cos(q_fk[0]) + LKF*cos(q_fk[0] + q_fk[1]);
    J[0][1] = LKF*cos(q_fk[0] + q_fk[1]);
    J[1][0] = LH*sin(q_fk[0]) + LKF*sin(q_fk[0] + q_fk[1]);
    J[1][1] = LKF*sin(q_fk[0] + q_fk[1]);
}
void pose_to_joint_space(void)
{
    //kinematics to get desired pose in joint space
    //calc after pos_des is found
    //MIRRORED BRANCH (bird-type leg): the knee pumps FORWARD -> the thigh stays BEHIND the
    //hip-foot line (the acos is SUBTRACTED) and the effective knee is POSITIVE. The original
    //(human knee) added the acos and used -acos on the knee.
    LHF = sqrt(pos_des[0]*pos_des[0] + pos_des[1]*pos_des[1]);      //length hip to foot
    q_ref[0] = (atan2(pos_des[0],-pos_des[1]) - acos((LH*LH + LHF*LHF - LKF*LKF)/(2.*LH*LHF)));
    q_ref[1] = acos((LHF*LHF - LH*LH - LKF*LKF)/(2.*LH*LKF));
}
void update_states(void)
{
    /*This function updates the global state variables, such as position, velocity, and error for the robot
     * No arguments - all values are global. This function should be called after trajectory is determined
     * and before the control effort is calculated*/

    //states for motor 1
    q_prev[0] = q_now[0];                   // Shift time
    q_now[0] = read_Enc1() + enc_offset[0]; // Get actual position
    err_q[0] = q_ref[0] - q_now[0];         // Calculate positional error
    //float raw_vel_h = (q_now[0] - q_prev[0])/T; // Estimate Velocity
    qdot_now[0] = (q_now[0] - q_prev[0])/T;

    //states for motor 2
    q_prev[1] = q_now[1];                   // Shift time
    q_now[1] = read_Enc2() + enc_offset[1]; // Get actual position
    err_q[1] = q_ref[1] - q_now[1];         // Calculate positional error
    //float raw_vel_k = (q_now[1] - q_prev[1])/T; // Estimate Velocity
    qdot_now[1] = (q_now[1] - q_prev[1])/T;

    //OPTIONAL
    //update velocity in the finite horizon vel filter
    //get_avg_vel(raw_vel_h, raw_vel_k);

    // REAL FK angles: hip = encoder + offset; knee = four-bar map
    q_fk[0] = q_now[0] + beta_off;
    q_fk[1] = KA*q_now[1]*q_now[1] + KB*q_now[1] + KC;

    // first-order low-pass velocity filter (= sim Stage 5 with lambda=10):
    // qdot_f is what the control D terms use (the raw derivative buzzes)
    {
        float af = vel_lambda*T/(1.0 + vel_lambda*T);
        qdot_f[0] += af*(qdot_now[0] - qdot_f[0]);
        qdot_f[1] += af*(qdot_now[1] - qdot_f[1]);
    }
    // velocity in FK space (knee via the four-bar chain rule)
    qfk_dot[0] = qdot_f[0];
    qfk_dot[1] = (2.0*KA*q_now[1] + KB)*qdot_f[1];

    // For stance controller
    update_jacobian();

    //get position in cartesian space
    pos_now[0] =  LH*sin(q_fk[0]) + LKF*sin(q_fk[0] + q_fk[1]);//Cartesian position
    pos_now[1] = -LH*cos(q_fk[0]) - LKF*cos(q_fk[0] + q_fk[1]);//from IK solve

}

float bezier4(const float *c, float s)
{
    // 4th-order Bezier (5 control points), s in [0,1] - identical to twin.bezier in the sim
    float m, s2, m2;
    if (s < 0.0) s = 0.0;
    if (s > 1.0) s = 1.0;
    m = 1.0 - s;
    s2 = s*s;  m2 = m*m;
    return c[0]*m2*m2 + 4.0*c[1]*s*m2*m + 6.0*c[2]*s2*m2 + 4.0*c[3]*s2*s*m + c[4]*s2*s2;
}
void calculate_control(void)
{
    /*This function is our main control function. Whatever control algorithm we need should be done here.
     *This function should be called after trajectory/feed-forward is determined and states updated,
     *but before we check for saturation*/

    // FLIGHT (Stage 1): joint PD toward q_ref. The D term now uses the FILTERED velocity (qdot_f):
    // with the filter, Kd_test can be raised live without the raw-derivative buzz.
    u_air[0] = Kp_test[0]*err_q[0] - Kd_test[0]*qdot_f[0];
    u_air[1] = Kp_test[1]*err_q[1] - Kd_test[1]*qdot_f[1];

    // Push trigger: manual st_go / cyclic st_auto / sensor st_sensor (Stage 2),
    // or continuous-hop TOUCHDOWN (JUMP_MODE, Stage 3: sensor edge + debounce)
    if (!STANCE_TEST && !JUMP_MODE) {
        st_active = 0;
    } else if (!st_active) {
        int rising  = (phase && !phase_prev);
        int jump_td = (JUMP_MODE && rising && t_air >= lo_debounce);
        if (st_go || (st_auto && (cnt % st_period) == 0) || (st_sensor && rising) || jump_td) {
            st_go = 0;
            st_active = 1;
            st_t = 0.0;
            qd_st_fk[0] = q_fk[0];   // soft PD reference = the pose at start (the crouch),
            qd_st_fk[1] = q_fk[1];   // like the MATLAB fixed touchdown q_d
            pk_Fz = 0.0;             // reset the peaks from the previous push
            pk_pwm[0] = 0.0;  pk_pwm[1] = 0.0;
            pk_e_min = q_now[1];  pk_e_max = q_now[1];
            if (jump_td) {           // hop telemetry: the flight that just ended
                n_hops++;
                t_air_last = t_air;
            }
        }
    }
    phase_prev = phase;

    if (st_active) {
        // STANCE (Eq.19): u_fk = -J^T*[Fx;Fz] + soft PD, all in FK space
        float s = st_t / Tst;
        float al;
        F_des[0] = fx_scale*bezier4(Fx_bz, s);
        F_des[1] = fz_scale*bezier4(Fz_bz, s);
        tau_fk[0] = -(J[0][0]*F_des[0] + J[1][0]*F_des[1])
                    + Kp_st*(qd_st_fk[0] - q_fk[0]) - Kd_st*qfk_dot[0];
        tau_fk[1] = -(J[0][1]*F_des[0] + J[1][1]*F_des[1])
                    + Kp_st*(qd_st_fk[1] - q_fk[1]) - Kd_st*qfk_dot[1];
        // FK -> motor: hip 1:1 (offset 0); knee x dq1_FK/de (four-bar, see update_jacobian)
        u_st[0] = tau_fk[0];
        u_st[1] = tau_fk[1]*(2.0*KA*q_now[1] + KB);
        // flight->stance blending (Eq.20, ~10 ms)
        al = (blend_ms > 0.0) ? st_t*1000.0/blend_ms : 1.0;
        if (al > 1.0) al = 1.0;
        u[0] = al*u_st[0] + (1.0 - al)*u_air[0];
        u[1] = al*u_st[1] + (1.0 - al)*u_air[1];
        st_t += T;
        {   // end of the push: by time (Tst) or early LIFTOFF in JUMP_MODE (the sensor
            // releases = the foot no longer touches; analog of the paper's "GRF < threshold")
            int early_lo = (JUMP_MODE && st_t >= td_min_stance && !phase);
            if (st_t >= Tst || early_lo) {
                st_active = 0;
                t_st_last = st_t;     // real stance duration (use it to tune Tst)
                t_air = 0.0;          // start the flight timer
                F_des[0] = 0.0;  F_des[1] = 0.0;
            }
        }
    } else {
        u[0] = u_air[0];
        u[1] = u_air[1];
        t_air += T;   // time in flight (touchdown debounce + t_air_last)
    }

    // MOTORS_OFF LAST: everything above stays COMPUTED (dry-run: watch F_des, tau_fk,
    // u_st and u_air in Expressions with the motors still), but nothing reaches the motor.
    if (MOTORS_OFF) { u[0] = 0.0; u[1] = 0.0; }
}
void prevent_saturation(void)
{
    /*This function accesses global control effort variable and makes sure it isnt saturating the motors. No inputs or outputs required
     * This function should be called at the end of processing, directly before the commands are being sent to the motors*/

    // Paper Eq.18 (= controller.py): u[] arrives as REAL TORQUE (N*m, in encoder joint units)
    // and here it is converted to VOLTAGE with back-EMF compensation:
    //     V = Rw/(kT*N)*u + kv*N*qdot_f      pwm = 10*V/VMAX  (pwm 10 = 12 V)
    // Replaces the example mapping (10/64.125 and 10/69.727), which delivered ~5% of the voltage
    // for real torques (see the note at the Rw/kT/kv declarations above).
    // SIGNS verified on the bench (unchanged): hip +, knee +.
    u[0] = (10.0/VMAX) * (Rw/(kT*NH)*u[0] + kv*NH*qdot_f[0]);
    u[1] = (10.0/VMAX) * (Rw/(kT*NK)*u[1] + kv*NK*qdot_f[1]);

    //test SAFETY limit (u_lim_test, lower than 10). In normal operation it would be 10.
    if      (u[0] >  u_lim_test) u[0] =  u_lim_test;
    else if (u[0] < -u_lim_test) u[0] = -u_lim_test;
    if      (u[1] >  u_lim_test) u[1] =  u_lim_test;
    else if (u[1] < -u_lim_test) u[1] = -u_lim_test;
}

/******************************************************  End Supporting Functions  **********************************************************/


/******************************************************Start Every MS = Effective Main**********************************************************/
// For F28379D quadrature encoder counter
// control for speed control
void DoEveryMilliSecond_CPU1(void)
{
    //get Trajectory
    calculate_traj();
    //pose_to_joint_space();//THIS RUNS WHEN NOT USING JOINT SPACE CONTROL. UPDATE THIS IF YOU WANT TO USE IT

    //stance or aerial phase
    get_phase();

    //update states
    update_states();

    //Get the control effort
    calculate_control();

    //double check to make motors are not overcommanded
    prevent_saturation();

    // record the push PEAKS (u here is already the final pwm sent to the motor)
    if (st_active) {
        float a0 = (u[0] < 0.0 ? -u[0] : u[0]);
        float a1 = (u[1] < 0.0 ? -u[1] : u[1]);
        if (F_des[1] > pk_Fz)    pk_Fz = F_des[1];
        if (a0 > pk_pwm[0])      pk_pwm[0] = a0;
        if (a1 > pk_pwm[1])      pk_pwm[1] = a1;
        if (q_now[1] < pk_e_min) pk_e_min = q_now[1];
        if (q_now[1] > pk_e_max) pk_e_max = q_now[1];
    }

    //send commands to motors
    set_EPWM1A_VNH5019(u[0]);
    set_EPWM1B_VNH5019(u[1]);

    //increment time (ms)
    cnt++;

    // manually trigger the analog reading be calling the ADC ISR. You should not need to touch this
    AdcdRegs.ADCSOCPRICTL.bit.RRPOINTER = 0x10;
    AdcdRegs.ADCSOCFRC1.all |= 0x0F; //start conversion for 0,1,2,4
}
/******************************************************End Every MS = Effective Main**********************************************************/

// SYS/BIOS Clock function
void DoEverySecond_CPU1(void){
    GpioDataRegs.GPADAT.bit.GPIO31 ^= 1;

}


 
 
 

int main()
{
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    // Comment this when use CCS for debugging
    //#ifdef _FLASH
    //// Send boot command to allow the CPU2 application to begin execution
    //IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
    //#else
    //// Send boot command to allow the CPU2 application to begin execution
    //IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
    //#endif

#ifdef _FLASH
    InitFlash();
#endif

    // Initialize System Control: PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl(); // F2837xD_SysCtrl.c

    // Initialize GPIO
    InitGpio(); // F2837xD_Gpio.c

    EALLOW;

    // For CPU1
    // Enable an GPIO OUTPUT on GPIO31, set it high
    GpioCtrlRegs.GPAPUD.bit.GPIO31  = 0;  // Enable pullup on GPIO31
    GpioDataRegs.GPASET.bit.GPIO31  = 1;  // Load output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;  // GPIO31 = GPIO31
    GpioCtrlRegs.GPADIR.bit.GPIO31  = 1;  // GPIO31 = output

/************************************************* SETTING UP CONTACT SWITCH **************************************************/
    // Enable an GPIO INPUT on GPIO9
    GpioCtrlRegs.GPAPUD.bit.GPIO9     = 0;    // enable pullup on GPIO9
    GpioCtrlRegs.GPAMUX1.bit.GPIO9    = 0;    // Set pin as standard GPIO as opposed to EPWM, etc
    GpioCtrlRegs.GPACTRL.bit.QUALPRD1 = 0x20; // Divide sampling frequency by 32. t_sample/32
    GpioCtrlRegs.GPADIR.bit.GPIO9     = 0;    // GPIO9 = input

/********************************************** END SETTING UP CONTACT SWITCH *************************************************/
    // For CPU2
    // Enable an GPIO output on GPIO34, set it high
    GpioCtrlRegs.GPBPUD.bit.GPIO34   = 0;   // Enable pullup on GPIO3
    GpioDataRegs.GPBSET.bit.GPIO34   = 1;   // Load output latch
    GpioCtrlRegs.GPBDIR.bit.GPIO34   = 1;   // GPIO34 = output

    GpioCtrlRegs.GPBCSEL1.bit.GPIO34 = 2;   // MUX with CPU2
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34  = 0;

    // Driverlib way for CPU2
    //GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;   // GPIO34 = output
    //GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    //GPIO_SetupPinMux(34, GPIO_MUX_CPU2, 0);

    EDIS;

    /***************************** Init Simulink Serial *****************************/
    // GPIO19 - SCIRXDB, GPIO18 - SCITXDB, Simulink plot through SCI-B
    //init_serial(&SerialB, 115200, simulink_serial_RXD);
    init_serial(&SerialA, 115200, NULL);

    /********************************* Init Text LCD *********************************/
    init_serial(&SerialC, 19200, NULL);

    /********************************** Init EPwm1A **********************************/
    init_EPWM1A_GPIO_VNH5019();     // init GPIO0 as EPWM1A (J4-40)
    init_EPWM1A_VNH5019();          // init EPWM1A with a 20KHz carrier frequency PWM signal.
    set_EPWM1A_VNH5019(u[0]);       // set to 0 (50% duty cycle)at the beginning, update in SYS/BIOS

    init_EPWM1B_GPIO_VNH5019();     // init GPIO1 as EPWM1A (J4-39)
    init_EPWM1B_VNH5019();          // init EPWM1B with a 20KHz carrier frequency PWM signal.
    set_EPWM1B_VNH5019(u[1]);       // set to 0 (50% duty cycle)at the beginning, update in SYS/BIOS

    /****************************** Init EQep1 and EQep2 *****************************/
    // J14 - EQEP1A(GPIO20), EQEP1B(GPIO21), J15 - EQEP2A(GPIO54), EQEP2B(GPIO55)

    // 7: total slits of motor's encoder in one revolution
    // 26.9: gear ratio
    // 753.2 = 7 * 26.9 * 4
    // 1: INVERTED polarity (-1 -> 1): the hip encoder read thigh-forward as NEGATIVE
    // 0: start  0 rad
    init_EQEP(&eqep1, EQEP1, 753.2, 1, 0.0);
    EQep1Regs.QPOSCNT = 0;

    // 7: total slits of motor's encoder in one revolution
    // 19.2: gear ratio
    // 1.5 Belt gear ratio
    // 806.4‬ =  7 * 19.2 * 4 * 1.5
    // 1: polarity of motor direction
    // 0: start  0 rad
    init_EQEP(&eqep2, EQEP2, 806.4, 1, 0.0);
    EQep2Regs.QPOSCNT = 0;


    /*********************************************************** Update Encoder Positions **********************************************************/
    //TODO initilaize the encoders by leaving th3 at zero and hold the shank to a desired position.
    float des_th4_offset = 0; //input the desired joint position in Radians

    enc_offset[0] =  0 - read_Enc1();
    enc_offset[1] =    - read_Enc2() + des_th4_offset; // offset is to that the
    q_now[0] = read_Enc1() + enc_offset[0]; // read encoder1 at the beginning, update in SYS/BIOS func
    q_now[1] = read_Enc2() + enc_offset[1]; // read encoder2 at the beginning, update in SYS/BIOS func

    //init global values with/related to function calls
    init_values();

    /******************************************************  NO CODE below here in Main() ******************************************************/

    EALLOW;
        EPwm5Regs.ETSEL.bit.SOCAEN  = 0;    // Disable SOC on A group
        EPwm5Regs.TBCTL.bit.CTRMODE = 3;    // freeze counter
        EPwm5Regs.ETSEL.bit.SOCASEL = 2;    // Select Event when counter equal to PRD
        EPwm5Regs.ETPS.bit.SOCAPRD  = 1;    // Generate pulse on 1st event (“pulse” is the same as “trigger”)
        EPwm5Regs.TBCTR             = 0x0;  // Clear counter
        EPwm5Regs.TBPHS.bit.TBPHS   = 0x00; // Phase is 0
        EPwm5Regs.TBCTL.bit.PHSEN   = 0;    // Disable phase loading
        EPwm5Regs.TBCTL.bit.CLKDIV  = 0;    // divide by 1  50Mhz Clock
        EPwm5Regs.TBPRD             = 50000;// Set Period to 1ms sample.  Input clock is 50MHz.
        // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
        EPwm5Regs.ETSEL.bit.SOCAEN  = 1;    //enable SOCA
        EPwm5Regs.TBCTL.bit.CTRMODE = 0;    //unfreeze, and enter up count mode
        EDIS;


    EALLOW;
        //write configurations for all ADCs  ADCA, ADCB, ADCC, ADCD
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
        AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
        AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
        AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
        //Set pulse positions to late
        AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        //power up the ADCs
        AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        //delay for 1ms to allow ADC time to power up
        DELAY_US(1000);

        //Select the channels to convert and end of conversion flag
        //Many statements commented out,  To be used when using ADCA or ADCB
        //ADCA
        //AdcaRegs.ADCSOC0CTL.bit.CHSEL = ???;  //SOC0 will convert Channel you choose Does not have to be A0
        //AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
        //AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
        //AdcaRegs.ADCSOC1CTL.bit.CHSEL = ???;  //SOC1 will convert Channel you choose Does not have to be A1
        //AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
        //AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
        //AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = ???; //set to last SOC that is converted and it will set INT1 flag ADCA1
        //AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
        //AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

        //ADCB
        //AdcbRegs.ADCSOC0CTL.bit.CHSEL = ???;  //SOC0 will convert Channel you choose Does not have to be B0
        //AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
        //AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
        //AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???;  //SOC1 will convert Channel you choose Does not have to be B1
        //AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
        //AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
        //AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???;  //SOC2 will convert Channel you choose Does not have to be B2
        //AdcbRegs.ADCSOC2CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
        //AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
        //AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???;  //SOC3 will convert Channel you choose Does not have to be B3
        //AdcbRegs.ADCSOC3CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
        //AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
        //  AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = ???; //set to last SOC that is converted and it will set INT1 flag ADCB1
        //AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
        //AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

        //ADCD
        AdcdRegs.ADCSOC0CTL.bit.CHSEL     = 0;  // set SOC0 to convert pin D0
        AdcdRegs.ADCSOC0CTL.bit.ACQPS     = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
        AdcdRegs.ADCSOC1CTL.bit.CHSEL     = 1;  //set SOC1 to convert pin D1
        AdcdRegs.ADCSOC1CTL.bit.ACQPS     = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
        AdcdRegs.ADCSOC2CTL.bit.CHSEL     = 2;  //set SOC2 to convert pin D2
        AdcdRegs.ADCSOC2CTL.bit.ACQPS     = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
        AdcdRegs.ADCSOC3CTL.bit.CHSEL     = 3;  //set SOC3 to convert pin D3
        AdcdRegs.ADCSOC3CTL.bit.ACQPS     = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
        AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 3;  //set to SOC1, the last converted, and it will set INT1 flag ADCD1
                                                //this should be set to n-1, where n is the number of analog ins being read
        AdcdRegs.ADCINTSEL1N2.bit.INT1E   = 1;  //enable INT1 flag
        AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //make sure INT1 flag is cleared
        EDIS;
        
        IER |= (M_INT1);
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Disable CPU interrupts and clear all CPU interrupt flags
    IFR = 0x0000;

    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP8 | PIEACK_GROUP9);  // ACKnowledge any SCI interrupt requests

    BIOS_start();

    return 0;
}

void ADCD_ISR (void)
{
    /*This function is the interupt service routine for the ADC reading. 
     *You should not need to touch this unless you add more than 4 analog inputs*/

    analog_in[0] = AdcdResultRegs.ADCRESULT0;
    analog_in[1] = AdcdResultRegs.ADCRESULT1;
    analog_in[2] = AdcdResultRegs.ADCRESULT2;
    analog_in[3] = AdcdResultRegs.ADCRESULT3;

    // To covert ADC readings from 12 bit to Volts
    //adcd0Volts = (float)(analog_in[0]*3.0/4095.0);

    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //clear PIE peripheral so processor waits until next interrupt flag
}


// For communication between Simulink & CCS

char SIMU_databyte1 = 0;
char SIMU_TXrawbytes[12];

int SIMU_datacollect = 0;
int SIMU_beginnewdata = 0;
int SIMU_Tranaction_Type = 0;
int SIMU_checkfirstcommandbyte = 0;

int SIMU_Var1_fromSIMU_16bit = 0;
int SIMU_Var2_fromSIMU_16bit = 0;
int SIMU_Var3_fromSIMU_16bit = 0;
int SIMU_Var4_fromSIMU_16bit = 0;
int SIMU_Var5_fromSIMU_16bit = 0;
int SIMU_Var6_fromSIMU_16bit = 0;
int SIMU_Var7_fromSIMU_16bit = 0;

int SIMU_Var1_toSIMU_16bit = 0;   // value to be sent to Simulink
int SIMU_Var2_toSIMU_16bit = 0;   // value to be sent to Simulink
long SIMU_Var1_toSIMU_32bit = 0;  // value to be sent to Simulink
long SIMU_Var2_toSIMU_32bit = 0;  // value to be sent to Simulink

void simulink_serial_RXD(serial_t *s, char data) {

    //  if (savenumbytes < 400) {  // Just for Debug
    //      savebytes[savenumbytes] = data;
    //      savenumbytes++;
    //  }

    // Only true if have not yet begun a message
    if (!SIMU_beginnewdata) {

        if (SIMU_checkfirstcommandbyte == 1) {

            // Check for start 2 bytes command = 32767 because assuming command will stay between -10000 and 10000
            if (0xFF == (unsigned char)data) {
                SIMU_checkfirstcommandbyte = 0;
            }

        } else {

            SIMU_checkfirstcommandbyte = 1;

            // Check for start char
            if (0x7F == (unsigned char)data) {

                SIMU_datacollect = 0;       // amount of data collected in message set to 0

                SIMU_beginnewdata = 1;      // flag to indicate we are collecting a message

                SIMU_Tranaction_Type = 2;

                // Could Start ADC and then ADC interrupt will read ENCs also and then send
                // but that is for Simulink control
                // For Simulink data collection just send most current ADC and ENCs
                // Simulink Sample rate needs to be at least 500HZ but 200Hz probably better

                /*
                 * When Simulink requests data from the DSP these four variables are sent.
                 *
                 * Assign these four variables the values you would like to plot in Simulink
                 *
                 * Notice that two 32-bit integers and two 16-bit integers. To upload a floating
                 * point value you will need to scale it by a factor and then remember to scale
                 * it back down on Simulink¡¯s end.
                 *
                 * 32-bit integer: -2147483648 to 2147483647
                 * 16-bit integer: -32768 to 32767
                 *
                 */

                SIMU_Var1_toSIMU_32bit = read_1;
                SIMU_Var2_toSIMU_32bit = read_2;

                SIMU_Var1_toSIMU_16bit = adcb0result;
                SIMU_Var2_toSIMU_16bit = adcb1result;


                SIMU_TXrawbytes[3] = (char)((SIMU_Var1_toSIMU_32bit >> 24) & 0xFF);
                SIMU_TXrawbytes[2] = (char)((SIMU_Var1_toSIMU_32bit >> 16) & 0xFF);
                SIMU_TXrawbytes[1] = (char)((SIMU_Var1_toSIMU_32bit >> 8) & 0xFF);
                SIMU_TXrawbytes[0] = (char)((SIMU_Var1_toSIMU_32bit) & 0xFF);

                SIMU_TXrawbytes[7] = (char)((SIMU_Var2_toSIMU_32bit >> 24) & 0xFF);
                SIMU_TXrawbytes[6] = (char)((SIMU_Var2_toSIMU_32bit >> 16) & 0xFF);
                SIMU_TXrawbytes[5] = (char)((SIMU_Var2_toSIMU_32bit >> 8) & 0xFF);
                SIMU_TXrawbytes[4] = (char)((SIMU_Var2_toSIMU_32bit) & 0xFF);

                SIMU_TXrawbytes[8] = (char)(SIMU_Var1_toSIMU_16bit & 0xFF);
                SIMU_TXrawbytes[9] = (char)((SIMU_Var1_toSIMU_16bit >> 8) & 0xFF);
                SIMU_TXrawbytes[10] = (char)(SIMU_Var2_toSIMU_16bit & 0xFF);
                SIMU_TXrawbytes[11] = (char)((SIMU_Var2_toSIMU_16bit >> 8) & 0xFF);

                serial_send(&SerialB, SIMU_TXrawbytes, 12);

            }
        }

    } else {  // Filling data

        if (SIMU_Tranaction_Type == 2) {

            if (SIMU_datacollect == 0) {
                SIMU_databyte1 = data;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 1) {

                SIMU_Var1_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 2) {

                SIMU_databyte1 = data;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 3) {

                SIMU_Var2_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 4) {

                SIMU_databyte1 = data;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 5) {

                SIMU_Var3_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 6) {

                SIMU_databyte1 = data;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 7) {

                SIMU_Var4_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 8) {

                SIMU_databyte1 = data;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 9) {

                SIMU_Var5_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 10) {

                SIMU_databyte1 = data;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 11) {

                SIMU_Var6_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 12) {

                SIMU_databyte1 = data;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 13) {

                SIMU_Var7_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_beginnewdata = 0;  // Reset the flag
                SIMU_datacollect = 0;   // Reset the number of chars collected
                SIMU_Tranaction_Type = 0;
            }
        }
    }
}
