#include <F28379D_EPwm.h>
#include <F28379D_Coecsl.h>
#include <F2837xD_EPwm_defines.h>



/************************** Setup EPWM1A **************************/
void init_EPWM1A_GPIO(void)
{
    EALLOW;
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // Comment out other unwanted lines.
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    // Configure EPWM-1 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM1 functional pins.
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    EDIS;
}


void init_EPWM1A_GPIO_VNH5019(void)
{
    EALLOW;
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // Comment out other unwanted lines.
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    // Configure EPWM-1 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM1 functional pins.
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;
    GpioDataRegs.GPASET.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;

    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;

    EDIS;
}


// This function has already been called for you in the main() function.  
// It sets up PWM1A with a 20KHz carrier frequency PWM signal.  
void init_EPWM1A(void)
{
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;		// Set epwm1 to upcount mode, 0x0
	EPwm1Regs.TBCTL.bit.FREE_SOFT = 0x2;            // Free Run, do not stop
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading, 0x0
	
	EPwm1Regs.TBPRD = 2500;                         // Set epwm1 counter  20KHz, set timer period
	EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;             // Phase is 0
	EPwm1Regs.TBCTR = 0x0000;                       // Clear counter
	
	// For EPWM1A
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;		    // Clear when counter = compareA
	EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;			    // Set when timer is 0
}

// This function sets PWM1A duty cycle given the float value between -10 and 10. Where
// -10 equates to 0% duty cycle
//   0 equates to 50% duty cycle
//  10 equates to 100% duty cycle
//  if you pass -5.0 to this function EPWM1A will be set to 25% duty cycle
//  if you pass 5.0 to this function EPWM1A will be set to 75% duty cycle

//  Example code
//  float myu = 0;
//  float Kpgain = 4.5;
//  float error = 0;
//	myu = Kpgain*error;
//	set_EPWM1A(myu);
void set_EPWM1A(float u)
{
    float pwmCountMax = 2500.0;
    float pwmVal = 0;
	
    if (u >  10) u =  10;
    if (u < -10) u = -10;
	
    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;
	
	// set compareA compare value
    EPwm1Regs.CMPA.bit.CMPA = (int)pwmVal;
}

// This function has already been called for you in the main() function.
// It sets up PWM1A with a 20KHz carrier frequency PWM signal.
void init_EPWM1A_VNH5019(void)
{
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // Set epwm1 to upcount mode, 0x0
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 0x2;            // Free Run, do not stop
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading, 0x0

    EPwm1Regs.TBPRD = 2500;                         // Set epwm1 counter  20KHz, set timer period
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;             // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm1Regs.CMPA.bit.CMPA = 0;

    // For EPWM1A
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Clear when counter = compareA
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;              // Set when timer is 0
}

// This function sets PWM1A duty cycle given the float value between -10 and 10. Where

void set_EPWM1A_VNH5019(float u)
{
    float pwmCountMax = 2500.0;
    float pwmVal = 0;

    if (u >  10) u =  10;
    if (u < -10) u = -10;

    if (u>0) {
        GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
        GpioDataRegs.GPASET.bit.GPIO3 = 1;
    } else {
        GpioDataRegs.GPASET.bit.GPIO2 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
    }
    pwmVal = abs(u) * (pwmCountMax / 10.0);

    // set compareA compare value
    EPwm1Regs.CMPA.bit.CMPA = (int)pwmVal;
}

/************************** Setup EPWM1B **************************/

void init_EPWM1B_GPIO(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    EDIS;
}

void init_EPWM1B_GPIO_VNH5019(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;
    GpioDataRegs.GPASET.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;

    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;

    EDIS;
}




void init_EPWM1B(void)
{
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // Set epwm1 to upcount mode, 0x0
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 0x2;            // Free Run, do not stop
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading, 0x0
    EPwm1Regs.TBPRD = 2500;                         // set epwm1 counter  20KHz
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;             // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                       // Clear counter
    // For EPWM1B
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear when counter = compareA
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;              // Set when timer is 0
}

void init_EPWM1B_VNH5019(void)
{
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // Set epwm1 to upcount mode, 0x0
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 0x2;            // Free Run, do not stop
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading, 0x0
    EPwm1Regs.TBPRD = 2500;                         // set epwm1 counter  20KHz
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;             // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm1Regs.CMPB.bit.CMPB = 0;
    // For EPWM1B
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear when counter = compareA
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;              // Set when timer is 0
}


void set_EPWM1B(float u)
{
    float pwmCountMax = 2500.0;
    float pwmVal = 0;
    if (u >  10) u =  10;
    if (u < -10) u = -10;
    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;
    EPwm1Regs.CMPB.bit.CMPB = (int)pwmVal;
}

void set_EPWM1B_VNH5019(float u)
{
    float pwmCountMax = 2500.0;
    float pwmVal = 0;

    if (u >  10) u =  10;
    if (u < -10) u = -10;

    if (u>0) {
        GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
        GpioDataRegs.GPASET.bit.GPIO5 = 1;
    } else {
        GpioDataRegs.GPASET.bit.GPIO4 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
    }
    pwmVal = abs(u) * (pwmCountMax / 10.0);

    // set compareB compare value
    EPwm1Regs.CMPB.bit.CMPB = (int)pwmVal;
}


/************************** Setup EPWM2A **************************/
void init_EPWM2A_GPIO(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    EDIS;
}

void init_EPWM2A(void)
{
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // Set epwm1 to upcount mode, 0x0
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 0x2;            // Free Run, do not stop
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading, 0x0
    EPwm2Regs.TBPRD = 2500;                         // set epwm1 counter  20KHz
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;             // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                       // Clear counter
    // For EPWM2A
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Clear when counter = compareA
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;              // Set when timer is 0
}

void set_EPWM2A(float u)
{
    float pwmCountMax = 2500.0;
    float pwmVal = 0;
    if (u >  10) u =  10;
    if (u < -10) u = -10;
    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;
    EPwm2Regs.CMPA.bit.CMPA = (int)pwmVal;
}


/************************** Setup EPWM2B **************************/
void init_EPWM2B_GPIO(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
    EDIS;
}


void init_EPWM2B(void)
{
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // Set epwm1 to upcount mode, 0x0
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 0x2;            // Free Run, do not stop
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading, 0x0
    EPwm2Regs.TBPRD = 2500;                         // set epwm1 counter  20KHz
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;             // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                       // Clear counter
    // For EPWM2B
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear when counter = compareA
    EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;              // Set when timer is 0
}

void set_EPWM2B(float u)
{
    float pwmCountMax = 2500.0;
    float pwmVal = 0;
    if (u >  10) u =  10;
    if (u < -10) u = -10;
    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;
    EPwm2Regs.CMPB.bit.CMPB = (int)pwmVal;
}


/************************** Setup EPWM3A **************************/
void init_EPWM3A_GPIO(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    EDIS;
}

void init_EPWM3A(void)
{
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // Set epwm1 to upcount mode, 0x0
    EPwm3Regs.TBCTL.bit.FREE_SOFT = 0x2;            // Free Run, do not stop
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading, 0x0
    EPwm3Regs.TBPRD = 2500;                         // set epwm1 counter  20KHz
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;             // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                       // Clear counter
    // For EPWM3A
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Clear when counter = compareA
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;              // Set when timer is 0
}

void set_EPWM3A(float u)
{
    float pwmCountMax = 2500.0;
    float pwmVal = 0;
    if (u >  10) u =  10;
    if (u < -10) u = -10;
    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;
    EPwm3Regs.CMPA.bit.CMPA = (int)pwmVal;
}


/************************** Setup EPWM3B **************************/
void init_EPWM3B_GPIO(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
    EDIS;
}

void init_EPWM3B(void)
{
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // Set epwm1 to upcount mode, 0x0
    EPwm3Regs.TBCTL.bit.FREE_SOFT = 0x2;            // Free Run, do not stop
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading, 0x0
    EPwm3Regs.TBPRD = 2500;                         // set epwm1 counter  20KHz
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;             // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                       // Clear counter
    // For EPWM3B
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear when counter = compareA
    EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;              // Set when timer is 0
}

void set_EPWM3B(float u)
{
    float pwmCountMax = 2500.0;
    float pwmVal = 0;
    if (u >  10) u =  10;
    if (u < -10) u = -10;
    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;
    EPwm3Regs.CMPB.bit.CMPB = (int)pwmVal;
}



