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
    int minRotationCounts;
    double y = 6;              // Distance from wrist pivot joint to the floor
    double h = 15;             // Distance from arm pivot axis to the floor

    public MotorPivotExp(DcMotorEx motorR, DcMotorEx motorL) {

        this.motorR = motorR;
        this.motorL = motorL;
        resetEncoders();

        motorR.setTargetPositionTolerance(tolerance);
        motorL.setTargetPositionTolerance(tolerance);

        motorL.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void manualMove(double joystickValue) {
        int newTarget = (int)(motorR.getCurrentPosition() + 40 * joystickValue);
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
        motorL.setTargetPosition(clamp(counts));
        motorR.setTargetPosition(clamp(counts));
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
//    public boolean inMotion() {
//        return motorR.isBusy();
//    }


    public void resetEncoders() {
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public int degreesToCounts(double degrees) {
        return (int)(degrees / TeamConstants.DEGREES_PER_COUNT);
    }


    public double countsToDegrees(double counts) {
        return (counts * TeamConstants.DEGREES_PER_COUNT);
    }


    public int getRmotorPos() { return motorR.getCurrentPosition(); }
    public int getLmotorPos() { return motorL.getCurrentPosition(); }
    // TODO: Add Action enabling Road Runner to have access to setPositionCounts

}
