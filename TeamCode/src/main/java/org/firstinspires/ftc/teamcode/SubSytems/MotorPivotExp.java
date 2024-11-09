package org.firstinspires.ftc.teamcode.SubSytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

public class MotorPivotExp implements TeamConstants {
    DcMotorEx motorL;
    DcMotorEx motorR;
    int tolerance = 20;
    int minCounts;
    double y = 6;              // Distance from wrist pivot joint to the floor
    double h = 15;             // Distance from arm pivot axis to the floor
    int targetDisp;

    public MotorPivotExp(DcMotorEx motorR, DcMotorEx motorL) {

        this.motorR = motorR;
        this.motorL = motorL;
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorR.setTargetPositionTolerance(tolerance);
        motorL.setTargetPositionTolerance(tolerance);

        motorL.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void manualMove(double joystickValue) {

//        double newTarget = (motorR.getCurrentPosition()+motorL.getCurrentPosition())/2.0 + .2 * joystickValue;
        double newTargetR = motorR.getCurrentPosition() + 50 * joystickValue;
        double newTargetL = motorL.getCurrentPosition() + 50 * joystickValue;
//            motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            motorR.setPower(0.750 * joystickValue);
//            motorL.setPower(0.750 * joystickValue);
        targetDisp = (int)newTargetL;
        motorR.setTargetPosition((int)newTargetR);
        motorL.setTargetPosition((int)newTargetL);

        motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorR.setVelocity(2000);
        motorL.setVelocity(2000);
    }


    public void periodic(int slideLength) {
        minCounts = (int) Math.acos((h-y)/slideLength);
    }


    public void setPosition(double degrees) {
        setPositionCounts(degreesToCounts(degrees));
    }


    public void setPositionCounts(int counts){
        motorR.setTargetPosition(clamp(counts));
        motorR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorR.setVelocity(100);
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
        return(motorR.getCurrentPosition());
    }


    public double getVelocity() {
        return(motorR.getVelocity());
    }


    public boolean inMotion() {
        return motorR.isBusy();
    }


    public int degreesToCounts(double degrees) {
        return (int)(degrees * TeamConstants.DEGREES_TO_COUNTS);
    }


    public double countsToDegrees(double counts) {
        return (counts * 1/TeamConstants.DEGREES_TO_COUNTS);
    }

    public int command() {
        return targetDisp;

    }

    public int getRmotorPos() { return motorR.getCurrentPosition(); }
    public int getLmotorPos() { return motorL.getCurrentPosition(); }
    // TODO: Add Action enabling Road Runner to have access to setPositionCounts

}
