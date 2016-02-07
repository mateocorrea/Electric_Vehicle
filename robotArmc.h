task main() {
    while(true) {
        goal = (vexRT[Btn8U]) ? 2000 : goal; // high
        goal = (vexRT[Btn8U]) ? 1000 : goal; // middle
        goal = (vexRT[Btn8U]) ? 200 : goal; // low
        motor[port3] = 127 * (vexRT[Btn5U] - vexRT[Btn5D]);
        motor[port4] = 127 * (vexRT[Btn5U] - vexRT[Btn5D]);
        motor[port2] = (SensorValue[potentiometer] - goal) / 8;
    }
}