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

    DcMotorEx motor;

    int tolerance = 20;
    int minRotationCounts;
    double y = 161.7 / 25.4;        // Distance from wrist pivot joint to the floor
    double h = (336 + 48) / 25.4;   // Distance from arm pivot axis to the floor
    //double lmin = 408 / 25.4;
    //initialize position = 45; degrees;

    public MotorPivotExp(DcMotorEx motor) {

        this.motor = motor;
       resetEncoders();

        motor.setTargetPositionTolerance(tolerance);
        motor.setPositionPIDFCoefficients(8);
//        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void triangulateTo(double x, double y) {
        int newTarget = (int)(((Math.toDegrees(-Math.atan2(x, y))+135)/TeamConstants.DEGREES_PER_COUNT));
        setPositionCounts(newTarget);

    }

    public void manualMove(double joystickValue) {
        int newTarget = (int)(motor.getCurrentPosition() + 60 * joystickValue);
        setPositionCounts(newTarget);
    }


    public void periodic(int slideLength) {
        minRotationCounts = degreesToCounts(Math.acos((h-y)/slideLength) * 180 / Math.PI);
    }


    public void setPositionDegrees(double degrees) {
        int counts = degreesToCounts(degrees);
        setPositionCounts(counts);
    }


    public void setPositionCounts(int counts){
        motor.setTargetPosition(clamp(counts));
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setVelocity(4000);

    }


    private int clamp(int position){
        if (position > MAX_POSITION_COUNTS){
            return MAX_POSITION_COUNTS;
        } else if (position < minRotationCounts){
            return minRotationCounts;
        }
        else return position;
    }


    public int getPosition(){
        return(motor.getCurrentPosition());
    }


    public double getVelocity() {
        return(motor.getVelocity());
    }
//
//
    public boolean inMotion() {
        return motor.isBusy();
    }


    public void resetEncoders() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public int degreesToCounts(double degrees) {
        return (int)(degrees / TeamConstants.DEGREES_PER_COUNT);
    }


    public double countsToDegrees(double counts) {
        return (counts * TeamConstants.DEGREES_PER_COUNT);
    }


    public int getmotorPos() { return motor.getCurrentPosition(); }


    public boolean getBusy(){
        return motor.isBusy();
    }


    /* ************************* Actions * *************************/
    public class SetPositionCounts implements Action {

        int position;

        public SetPositionCounts(int pos) {
            position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setPositionCounts(position);
            return false;
        }

    }


    public class ArmTrig implements Action {

        public double newx;
        public double newy;

        public ArmTrig(double x, double y){newx = x; newy = y;}

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            triangulateTo(newx, newy);
            return false;
        }

    }


    public Action rotateToPosition(int position) {
        return new SetPositionCounts(position);
    }
//    public Action armTrig(double x, double y) {
//        return new armTrig(x,y);
//    }

    public Action armTrig(double x, double y) {
        return new ArmTrig(x,y);
    }

}
