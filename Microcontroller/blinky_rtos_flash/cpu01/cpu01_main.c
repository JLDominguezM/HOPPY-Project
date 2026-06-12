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
void pose_to_joint_space(void);   // prototipo: calculate_traj la llama antes de su definicion
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

/****** PRUEBA LENTA DE LA PIERNA - sujetala en el aire ******/
int   MOTORS_OFF = 1;            // 1 = solo verifica la IK (q_ref) sin mover; 0 = mueve la pierna
                                 // (arranca en 1 por seguridad: ponlo en 0 EN VIVO en Expressions)
// ===== MODELO DEL MOTOR goBILDA (Ec.18 del paper; los usa prevent_saturation) =====
// Desde 2026-06-10 (noche) u[] es TORQUE REAL en N*m y prevent_saturation lo convierte a
// voltaje: V = Rw/(kT*N)*u + kv*N*qdot_f. El mapeo viejo del ejemplo (10/64.125) trataba
// "64 N*m" como pwm completo, pero fisicamente pwm completo = 12 V = ~3.4 N*m a rotor parado
// -> mandaba ~5% del voltaje para los torques reales del MATLAB (el empuje no se sentia).
float Rw = 1.3;                  // resistencia de armadura (ohm)
float kT = 0.0135;               // constante de torque (N*m/A)
float kv = 0.0186;               // constante de back-EMF (V*s/rad)
float NH = 26.9;                 // reduccion cadera (rotor -> eje = unidades de q_now[0])
float NK = 28.8;                 // reduccion EFECTIVA rodilla en unidades de e (la escala 806.4
                                 // del eqep2 hace que el 26.9 fisico se vea como 28.8 = el N_K del PDF)
float VMAX = 12.0;               // bus del driver: pwm 10 = 12 V
// Ganancias del PD de banco, AHORA EN N*m/rad: {20.9, 30.9} = equivalentes EXACTOS de los
// {400, 600} verificados en banco con el mapeo viejo (mismo pwm por radian de error).
float Kp_test[2] = {20.9, 30.9};
float Kd_test[2] = { 0.0,  0.0}; // D = 0; con el filtro qdot_f ya se puede subir (0.26 = el "5" viejo)
float u_lim_test = 4.0;          // limite de pseudo-pwm (de 10). Subelo en vivo si la rodilla no alcanza
float ramp_rate  = 0.15;         // rad/s -> MUY lento (ya no se usa en el aereo)
// ===== MORFOLOGIA REAL DE LA PIERNA (aclarada 2026-06-10, ver HANDOFF_FIRMWARE.md) =====
// La pierna es TIPO AVE (espejo del HOPPY original): la "rotula" apunta hacia ATRAS y
// FLEXIONAR la rodilla (e mas negativo) manda el pie hacia ADELANTE (+x). Ademas el tubo
// NUNCA queda alineado con el muslo: en el tope de extension ya va ~46 deg adelante.
// POSE CERO (alcanzable y repetible): MUSLO VERTICAL (aplomar con telefono en las placas)
// + rodilla en su TOPE DE EXTENSION (sin resorte peleando). Ahi se escriben QPOSCNT=0.
// CADERA: +q0 = muslo hacia adelante. Offset del cero = 0 (muslo vertical EN el cero).
float beta_off   = 0.0;
// RODILLA - MAPA DEL 4-BARRAS:  q1_FK = KA*e^2 + KB*e + KC,  e = q_now[1] (encoder).
// q1_FK = angulo efectivo rodilla->pie respecto al muslo, ADELANTE positivo. Flexionar
// (e<0) AUMENTA q1_FK. |KA|,|KB| = forma medida en la calibracion de 4 puntos (la relacion
// efectiva varia ~1.5 extension -> ~0.7 flexion); signo volteado por la morfologia espejo.
// KC = q1_FK en el cero (~+0.80 = los ~46 deg del tubo + offset DK del pie): AFINAR con plomada.
float KA = -0.454;
float KB = -1.534;
float KC = 0.80;
float q_fk[2] = {0.0, 0.0};   // angulos FK REALES (para cinematica/Jacobiano); q_now queda en encoder
// Trayectoria aerea, afinable EN VIVO (calculate_traj recalcula pos_des cada ms con ESTAS):
float traj_amp   = 0.00;    // amplitud tangencial del barrido (m). 0 = pie QUIETO en traj_x0
                            // (arranca en 0 = modo calibracion; sube a 0.05 EN VIVO para el barrido)
float traj_x0    = 0.00;    // centro del barrido (m). + = adelante
float traj_depth = -0.22;   // profundidad del pie bajo la cadera (m)
long  traj_T     = 8000;    // periodo del ciclo (ms)
// MODO JUNTA DIRECTA (calibracion): q_ref = q_test tal cual (unidades del ENCODER), sin IK.
// Para medir la relacion efectiva del 4-barras de la rodilla con la cadera quieta.
int   JOINT_MODE = 0;
float q_test[2]  = {0.0, 0.0};
float wp_hold    = 2.0;          // s en cada pose
#define NWP 2
float wp[NWP][2] = {
  { 0.000,  0.000},   // home (pierna extendida)
  { 0.524, -1.000},   // cuclilla: cadera +30 deg, rodilla -57 deg (mueve los dos juntos)
};
int   wp_i = 0;  float t_wp = 0.0;

/****** ETAPA 2 - CONTROL DE APOYO (banco): empuje GRF Bezier  u = -J^T*[Fx;Fz] + PD suave ******/
// Port fiel de la Ec.19 del paper (controller.py / Simulator_MATLAB, ver mujoco/CONTROL_FORWARD.md).
// El empuje dura Tst y el perfil de fuerza [Fx;Fz] (N, frame de cadera: x adelante, z arriba)
// es un Bezier de 4to orden. El torque sale en espacio FK y se mapea a motor: cadera 1:1,
// rodilla MULTIPLICADA por dq1_FK/de = 2*KA*e+KB (4-barras). MOTORS_OFF sigue mandando.
int   STANCE_TEST = 0;        // 1 = habilita el modo apoyo (los disparos de abajo)
int   st_go     = 0;          // poner 1 EN VIVO = dispara UN empuje (se auto-limpia)
int   st_auto   = 0;          // 1 = empuje ciclico cada st_period ms
long  st_period = 3000;       // periodo del modo auto (ms)
int   st_sensor = 0;          // 1 = dispara al PISAR el sensor de pie (flanco 0->1 de phase)
float Tst      = 0.35;        // duracion del empuje (s). Dry-run: subir a 5.0 y mirar F_des/u_st
float fz_scale = 1.0;         // escala del perfil vertical (pico real del Bezier ~42 N a s=0.5)
float fx_scale = 1.0;         // escala del tangencial. Su SIGNO fija el sentido de avance (Etapa 3)
float Fz_bz[5] = {0.0, 20.0, 100.0, 0.0, 0.0};   // puntos de control (= MATLAB/sim)
float Fx_bz[5] = {0.0,  0.0, -25.0, 0.0, 0.0};
float Kp_st = 0.03;           // PD suave de apoyo en espacio FK (= MATLAB; es un regularizador)
float Kd_st = 0.08;
float blend_ms = 10.0;        // mezcla aereo->apoyo (Ec.20) para no meter un escalon de torque
int   st_active = 0;          // (estado) 1 mientras empuja
float st_t = 0.0;             // (estado) tiempo dentro del empuje (s)
float qd_st_fk[2] = {0.0, 0.0};  // pose FK capturada al iniciar el empuje (ref del PD suave)
float F_des[2]  = {0.0, 0.0};    // [Fx;Fz] actuales del Bezier (N) - mirar en Expressions
float tau_fk[2] = {0.0, 0.0};    // torque de apoyo en espacio FK (N*m)
float u_air[2]  = {0.0, 0.0};    // torque del PD aereo (espacio motor)
float u_st[2]   = {0.0, 0.0};    // torque de apoyo en espacio MOTOR (N*m) - mirar en dry-run
int   phase_prev = 0;            // para el flanco de subida del sensor (st_sensor)
// Filtro de velocidad (= Fase 5 de la sim, lambda=10 rad/s): permite usar Kd sin zumbido.
float vel_lambda = 10.0;      // ancho de banda (rad/s). Subir EN VIVO (20-30) si la D va lenta
float qdot_f[2]  = {0.0, 0.0};   // qdot filtrada (espacio encoder)
float qfk_dot[2] = {0.0, 0.0};   // velocidad en espacio FK (rodilla = f'(e)*qdot_f[1])
// PICOS del ultimo empuje (se resetean al disparar y QUEDAN CONGELADOS al terminar:
// leerlos despues con calma, sin pelear con el refresh de Expressions)
float pk_Fz = 0.0;            // max F_des[1] comandada (N)
float pk_pwm[2] = {0.0, 0.0}; // max |pwm| FINAL enviado a cada motor (post-saturacion)
float pk_e_min = 0.0;         // min/max de q_now[1] durante el empuje: cuanto VIAJO la
float pk_e_max = 0.0;         // rodilla (pk_e_max cerca de 0 = topo en su extension)

/****** ETAPA 3 - JUMP_MODE: salto continuo por sensor de pie (FSM aereo<->apoyo) ******/
// AEREO: el PD lleva el pie a la pose de aterrizaje (traj_x0, traj_depth) - colocacion FIJA
//   (sin encoder de boom no hay Raibert; el avance lo fija Fx, como la sim con vx_d=0).
// TOUCHDOWN: flanco 0->1 del sensor de pie con t_air >= lo_debounce -> empuje GRF (Etapa 2).
// LIFTOFF: sensor suelto tras td_min_stance (el analogo del "GRF < umbral" del paper),
//   o fin de Tst -> regresa al aereo y el PD recoge la pierna.
// MOTORS_OFF y u_lim_test siguen mandando. Sin contrapeso apenas despega ("botecitos"),
// pero el ciclo completo sensor->empuje->recoger->caer->sensor se valida igual.
int   JUMP_MODE = 0;          // 1 = salto continuo (defaults seguros: arranca apagado)
float lo_debounce  = 0.04;    // s minimos en aereo antes de aceptar touchdown (anti-rebote)
float td_min_stance = 0.05;   // s minimos de apoyo antes de permitir liftoff temprano
long  n_hops = 0;             // contador de saltos disparados
float t_air = 0.0;            // tiempo en aereo actual (s)
float t_air_last = 0.0;       // DURACION DEL ULTIMO VUELO (s) - altura apex ~ g*t^2/8
float t_st_last  = 0.0;       // duracion real del ultimo apoyo (s) - para afinar Tst
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
    
    // El ISR ADCD del ejemplo NO esta enganchado al RTOS (no corre), por eso analog_in no
    // cambiaba. Leemos el resultado del ADC DIRECTO (la conversion se dispara cada ms al final
    // de DoEveryMilliSecond). Sensor de pie en ADCIN-D0.
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

    LKF = sqrt(DK*DK + LK*LK);   // = 0.16349  (el ejemplo lo dejaba SIN asignar = 0, BUG)
}
void calculate_traj(void)
{
    /*This function is used to determine the trajectory. This should be the first thing done in the timed loop
     * as it only depends on the discrete time. This function will eventually determine the phase (aerial or stance)
     * and choose the correct position required for that time in that phase*/

    // MODO JUNTA DIRECTA: objetivos de junta crudos (encoder), para calibrar el 4-barras.
    if (JOINT_MODE) {
        q_ref[0] = q_test[0];
        q_ref[1] = q_test[1];
        return;
    }

    // AEREO: coloca el pie en pos_des (Cartesiano, frame de la cadera) via IK -> q_ref.
    // Ciclamos la posicion TANGENCIAL del pie adelante/atras (lento), a profundidad fija, para
    // ver la colocacion de pie (lo que hace el control aereo en vuelo). Afinable EN VIVO via
    // traj_amp / traj_x0 / traj_depth / traj_T (pos_des se recalcula cada ms, NO lo edites directo).
    // traj_amp = 0 -> pie QUIETO en (traj_x0, traj_depth): el modo para calibrar beta_off.
    float ang = (cnt % traj_T) * (TWOPI / (float)traj_T);
    pos_des[0] = traj_x0 + traj_amp * sin(ang);   // tangencial (adelante/atras)
    pos_des[1] = traj_depth;                      // profundidad bajo la cadera

    pose_to_joint_space();             // IK: pos_des -> q_ref (convencion FK: q=0 = pierna recta abajo)
    q_ref[0] -= beta_off;              // cadera: FK -> encoder (offset calibrado = 0)
    {   // rodilla: FK -> encoder = inversa del mapa del 4-barras (rama e<=0 con KA,KB<0)
        float disc = KB*KB + 4.0f*KA*(q_ref[1] - KC);
        if (disc < 0.0f) disc = 0.0f;  // objetivo mas flexionado que el alcance del mapa
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
    // Con los angulos FK REALES (q_fk), no los del encoder. Para u=-J^T*F el torque de rodilla
    // ademas se MULTIPLICA por d(q1_FK)/de = 2*KA*e+KB (trabajo virtual: tau_m*de = tau_FK*dq1
    // -> tau_m = tau_FK*f'(e); como f' es negativo, el signo del motor sale correcto solo).
    J[0][0] = LH*cos(q_fk[0]) + LKF*cos(q_fk[0] + q_fk[1]);
    J[0][1] = LKF*cos(q_fk[0] + q_fk[1]);
    J[1][0] = LH*sin(q_fk[0]) + LKF*sin(q_fk[0] + q_fk[1]);
    J[1][1] = LKF*sin(q_fk[0] + q_fk[1]);
}
void pose_to_joint_space(void)
{
    //kinematics to get desired pose in joint space
    //calc after pos_des is found
    //RAMA ESPEJO (pierna tipo ave): rodilla bombea ADELANTE -> muslo queda ATRAS de la
    //linea cadera-pie (acos se RESTA) y la rodilla efectiva es POSITIVA. El original
    //(rodilla humana) sumaba el acos y usaba -acos en la rodilla.
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

    // angulos FK REALES: cadera = encoder + offset; rodilla = mapa del 4-barras
    q_fk[0] = q_now[0] + beta_off;
    q_fk[1] = KA*q_now[1]*q_now[1] + KB*q_now[1] + KC;

    // filtro pasa-bajas de velocidad (1er orden, = Fase 5 de la sim con lambda=10):
    // qdot_f es la que usan los terminos D del control (la derivada cruda zumba)
    {
        float af = vel_lambda*T/(1.0 + vel_lambda*T);
        qdot_f[0] += af*(qdot_now[0] - qdot_f[0]);
        qdot_f[1] += af*(qdot_now[1] - qdot_f[1]);
    }
    // velocidad en espacio FK (rodilla via regla de la cadena del 4-barras)
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
    // Bezier de 4to orden (5 puntos de control), s en [0,1] - identico a twin.bezier de la sim
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

    // AEREO (Etapa 1): PD de junta hacia q_ref. La D ahora usa la velocidad FILTRADA (qdot_f):
    // con el filtro ya se puede subir Kd_test en vivo sin el zumbido de la derivada cruda.
    u_air[0] = Kp_test[0]*err_q[0] - Kd_test[0]*qdot_f[0];
    u_air[1] = Kp_test[1]*err_q[1] - Kd_test[1]*qdot_f[1];

    // Disparo del empuje: manual st_go / ciclico st_auto / sensor st_sensor (Etapa 2),
    // o TOUCHDOWN del salto continuo (JUMP_MODE, Etapa 3: flanco del sensor + anti-rebote)
    if (!STANCE_TEST && !JUMP_MODE) {
        st_active = 0;
    } else if (!st_active) {
        int rising  = (phase && !phase_prev);
        int jump_td = (JUMP_MODE && rising && t_air >= lo_debounce);
        if (st_go || (st_auto && (cnt % st_period) == 0) || (st_sensor && rising) || jump_td) {
            st_go = 0;
            st_active = 1;
            st_t = 0.0;
            qd_st_fk[0] = q_fk[0];   // ref del PD suave = la pose al iniciar (la cuclilla),
            qd_st_fk[1] = q_fk[1];   // como el q_d fijo de touchdown del MATLAB
            pk_Fz = 0.0;             // resetea los picos del empuje anterior
            pk_pwm[0] = 0.0;  pk_pwm[1] = 0.0;
            pk_e_min = q_now[1];  pk_e_max = q_now[1];
            if (jump_td) {           // telemetria del salto: vuelo que acaba de terminar
                n_hops++;
                t_air_last = t_air;
            }
        }
    }
    phase_prev = phase;

    if (st_active) {
        // APOYO (Ec.19): u_fk = -J^T*[Fx;Fz] + PD suave, todo en espacio FK
        float s = st_t / Tst;
        float al;
        F_des[0] = fx_scale*bezier4(Fx_bz, s);
        F_des[1] = fz_scale*bezier4(Fz_bz, s);
        tau_fk[0] = -(J[0][0]*F_des[0] + J[1][0]*F_des[1])
                    + Kp_st*(qd_st_fk[0] - q_fk[0]) - Kd_st*qfk_dot[0];
        tau_fk[1] = -(J[0][1]*F_des[0] + J[1][1]*F_des[1])
                    + Kp_st*(qd_st_fk[1] - q_fk[1]) - Kd_st*qfk_dot[1];
        // FK -> motor: cadera 1:1 (offset 0); rodilla x dq1_FK/de (4-barras, ver update_jacobian)
        u_st[0] = tau_fk[0];
        u_st[1] = tau_fk[1]*(2.0*KA*q_now[1] + KB);
        // blending aereo->apoyo (Ec.20, ~10 ms)
        al = (blend_ms > 0.0) ? st_t*1000.0/blend_ms : 1.0;
        if (al > 1.0) al = 1.0;
        u[0] = al*u_st[0] + (1.0 - al)*u_air[0];
        u[1] = al*u_st[1] + (1.0 - al)*u_air[1];
        st_t += T;
        {   // fin del empuje: por tiempo (Tst) o LIFTOFF temprano en JUMP_MODE (el sensor
            // se suelta = el pie ya no toca; analogo del "GRF < umbral" del paper)
            int early_lo = (JUMP_MODE && st_t >= td_min_stance && !phase);
            if (st_t >= Tst || early_lo) {
                st_active = 0;
                t_st_last = st_t;     // duracion real del apoyo (afinar Tst con esto)
                t_air = 0.0;          // arranca el cronometro de vuelo
                F_des[0] = 0.0;  F_des[1] = 0.0;
            }
        }
    } else {
        u[0] = u_air[0];
        u[1] = u_air[1];
        t_air += T;   // tiempo en aereo (debounce del touchdown + t_air_last)
    }

    // MOTORS_OFF al FINAL: todo lo de arriba queda CALCULADO (dry-run: mirar F_des, tau_fk,
    // u_st y u_air en Expressions con los motores quietos), pero al motor no le llega nada.
    if (MOTORS_OFF) { u[0] = 0.0; u[1] = 0.0; }
}
void prevent_saturation(void)
{
    /*This function accesses global control effort variable and makes sure it isnt saturating the motors. No inputs or outputs required
     * This function should be called at the end of processing, directly before the commands are being sent to the motors*/

    // Ec.18 del paper (= controller.py): u[] llega en TORQUE REAL (N*m, en unidades de junta
    // del encoder) y aqui se convierte a VOLTAJE con compensacion de back-EMF:
    //     V = Rw/(kT*N)*u + kv*N*qdot_f      pwm = 10*V/VMAX  (pwm 10 = 12 V)
    // Reemplaza el mapeo del ejemplo (10/64.125 y 10/69.727), que entregaba ~5% del voltaje
    // para torques reales (ver nota en la declaracion de Rw/kT/kv arriba).
    // SIGNOS verificados en banco (sin cambio): cadera +, rodilla +.
    u[0] = (10.0/VMAX) * (Rw/(kT*NH)*u[0] + kv*NH*qdot_f[0]);
    u[1] = (10.0/VMAX) * (Rw/(kT*NK)*u[1] + kv*NK*qdot_f[1]);

    //limite de SEGURIDAD de la prueba (u_lim_test, mas bajo que 10). En operacion normal seria 10.
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

    // registra los PICOS del empuje (u aqui ya es el pwm final que se manda al motor)
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
    // 1: polaridad INVERTIDA (-1 -> 1): el encoder de cadera leia muslo-adelante como NEGATIVO
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
