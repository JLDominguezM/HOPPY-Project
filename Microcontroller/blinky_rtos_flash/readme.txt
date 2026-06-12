/************************************ CPU1 ************************************/

LED D10 blink 1000ms

print time_counter to LCD every 100ms in SYS/BIOS function

LCD RX --- GPIO56 (SCITXD-C, J5-44)

Motor1 --- GPIO0 (EPWM-1A) 
Motor2 --- GPIO1 (EPWM-1B)

Quadrature encoder counter1 --- J14 - EQEP1A(GPIO20), EQEP1B(GPIO21)
Quadrature encoder counter2 --- J15 - EQEP2A(GPIO54), EQEP2B(GPIO55)

PI for Speed Control


/************************************ CPU2 ************************************/

LED D9 blink 500ms