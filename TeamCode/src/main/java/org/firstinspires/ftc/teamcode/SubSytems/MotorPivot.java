package org.firstinspires.ftc.teamcode.SubSytems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

/*
 *  Tune system to find a value of Kp (Proportional Coefficient in PID) and Kf (feed forward term
 *  if necessary) to achieve the target position in the least amount of time. See second link below
 *  for the functions used to set these values.
 *
 * Info for motor control and available motor functions can be found at the links below:
 *  https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback#choosing-a-motor-mode
 *  https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
 */

public class MotorPivot implements TeamConstants {

    DcMotorEx motor;
    int tolerance = 20;
    int minCounts;
    double y = 6;              // Distance from wrist pivot joint to the floor
    double h = 15;             // Distance from arm pivot axis to the floor


    public MotorPivot(DcMotorEx motor) {
        this.motor = motor;
        motor.setTargetPositionTolerance(tolerance);
    }


    public void manualMove(double joystickValue) {
        setPositionCounts((int)(motor.getCurrentPosition() + .2 * joystickValue));
        motor.setVelocity(600);
    }


    public void periodic(int slideLength) {
        minCounts = (int) Math.acos((h-y)/slideLength);
    }


    public void setPosition(double degrees) {
        setPositionCounts(degreesToCounts(degrees));
    }


    public void setPositionCounts(int counts){
        motor.setTargetPosition(clamp(counts));
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setVelocity(3000);
    }


    private int clamp(int position){
        if (position > MAX_POSITION_COUNTS){
            return MAX_POSITION_COUNTS;
        } else if (position < minCounts){
            return minCounts;
        }
        else return position;
    }


    public int getPosition(){
        return(motor.getCurrentPosition());
    }


    public double getVelocity() {
        return(motor.getVelocity());
    }


    public boolean inMotion() {
        return motor.isBusy();
    }


    public int degreesToCounts(double degrees) {
        return (int)(degrees * TeamConstants.DEGREES_TO_COUNTS);
    }


    public double countsToDegrees(double counts) {
        return (counts * 1/TeamConstants.DEGREES_TO_COUNTS);
    }


    // TODO: Add Action enabling Road Runner to have access to setPositionCounts

}
