package org.firstinspires.ftc.teamcode.SubSytems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
    int minRotationCounts;
    double y = 161.7 / 25.4;        // Distance from wrist pivot joint to the floor
    double h = (336 + 48) / 25.4;   // Distance from arm pivot axis to the floor
    //double lmin = 408 / 25.4;
    //initialize position = 45; degrees;

    public MotorPivotExp(DcMotorEx motorR, DcMotorEx motorL) {

        this.motorR = motorR;
        this.motorL = motorL;
        resetEncoders();

        motorR.setTargetPositionTolerance(tolerance);
        motorL.setTargetPositionTolerance(tolerance);

        motorL.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void triangulateTo(double x, double y) {
        int newTarget = (int)(((Math.toDegrees(-Math.atan2(x, y))+135)/TeamConstants.DEGREES_PER_COUNT));
        setPositionCounts(newTarget, newTarget);
    }

    public void manualMove(double joystickValue) {
        int newTargetL = (int)(motorL.getCurrentPosition() + 20 * joystickValue);
        int newTargetR = (int)(motorR.getCurrentPosition() + 20 * joystickValue);
        setPositionCounts(newTargetL, newTargetR);
    }


    public void periodic(int slideLength) {
        minRotationCounts = degreesToCounts(Math.acos((h-y)/slideLength) * 180 / Math.PI);
    }


//    public void setPositionDegrees(double degrees) {
//        int counts = degreesToCounts(degrees);
//        setPositionCounts(counts);
//    }


    public void setPositionCounts(int countsL, int countsR){
        motorL.setTargetPosition(clamp(countsL));
        motorR.setTargetPosition(clamp(countsR));
        motorL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorL.setVelocity(2000);
        motorR.setVelocity(2000);

    }


    private int clamp(int position){
        if (position > MAX_POSITION_COUNTS){
            return MAX_POSITION_COUNTS;
        } else if (position < minRotationCounts){
            return minRotationCounts;
        }
        else return position;
    }


//    public int getPosition(){
//        return(motorR.getCurrentPosition());
//    }
//
//
//    public double getVelocity() {
//        return(motorR.getVelocity());
//    }
//
//
    public boolean inMotion() {
        return motorR.isBusy()  ||  motorL.isBusy();
    }


    public void resetEncoders() {
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public int degreesToCounts(double degrees) {
        return (int)(degrees / TeamConstants.DEGREES_PER_COUNT);
    }


    public double countsToDegrees(double counts) {
        return (counts * TeamConstants.DEGREES_PER_COUNT);
    }


    public int getRmotorPos() { return motorR.getCurrentPosition(); }
    public int getLmotorPos() { return motorL.getCurrentPosition(); }


    /* ************************* Actions * *************************/
    public class SetPositionCounts implements Action {

        int position;

        public SetPositionCounts(int pos) {
            position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setPositionCounts(position, position);
            return false;
        }

    }


    public Action rotateToPosition(int position) {
        return new SetPositionCounts(position);
    }

}
