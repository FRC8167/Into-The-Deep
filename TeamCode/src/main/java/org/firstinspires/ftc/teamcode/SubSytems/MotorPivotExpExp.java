package org.firstinspires.ftc.teamcode.SubSytems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Cogintilities.PidController;
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

public class MotorPivotExpExp implements TeamConstants {

    DcMotorEx motorMain;
    DcMotorEx motorSecondary;


    int tolerance = 100;
    int minRotationCounts;
    double y = 161.7 / 25.4;        // Distance from wrist pivot joint to the floor
    double h = (336 + 48) / 25.4;   // Distance from arm pivot axis to the floor
    //double lmin = 408 / 25.4;
    //initialize position = 45; degrees;

    PidController MotorController;


    double power = 0;

    public MotorPivotExpExp(DcMotorEx motorMain, DcMotorEx motorSecondary) {

        this.motorMain = motorMain;
        this.motorSecondary = motorSecondary;



        this.motorMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorSecondary.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        this.motorMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.motorSecondary.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double kP = 0.02, kI = 0, kD = 0;

        this.MotorController = new PidController(kP, kI, kD, tolerance);

//       resetEncoders();

//        motorMain.setTargetPositionTolerance(tolerance);
//        motorMain.setPositionPIDFCoefficients(8);
//        motor.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public double getMainPower() {
        return motorMain.getPower();
    }
    public DcMotor.RunMode getMainMode() {
        return motorMain.getMode();
    }

    public DcMotor.RunMode getSecMode() {
        return motorSecondary.getMode();
    }

    public double getSecondaryPower() {
        return motorSecondary.getPower();
    }

    public void setPowers(double power){
        this.motorMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorSecondary.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorMain.setPower(power);
        motorSecondary.setPower(power);
    }

//    public String getMainDirection() {
//        return String.valueOf(motorMain.getDirection());
//    }
//
//    public double getMainVelocity(){
//        return motorMain.getVelocity();
//    }

//    public double getTarget(){
//        return MotorController.getTargetPosition();
//    }

    public void triangulateTo(double x, double y) {
        int newTarget = (int)(((Math.toDegrees(-Math.atan2(x, y))+135)/TeamConstants.DEGREES_PER_COUNT));
        setPositionCounts(newTarget);

    }


    public void manualMove(double joystickValue) {
        int newTarget = (int)(motorMain.getCurrentPosition() + 20 * joystickValue);
        setPositionCounts(newTarget);
    }

    /** *********************** Periodic ********************* **/
    public void periodic() {
        power = MotorController.update(motorMain.getCurrentPosition());
        if (!MotorController.isGood()) {
            setPowers(power);
        }
        else{
            setPowers(0);
        }

//        minRotationCounts = degreesToCounts(Math.acos((h-y)/slideLength) * 180 / Math.PI);

    }


    public void setPositionDegrees(double degrees) {
        int counts = degreesToCounts(degrees);
        setPositionCounts(counts);
    }


    public void setPositionCounts(int counts){
//        motor.setTargetPosition(clamp(counts));
//        motorMain.setTargetPosition(Range.clip(counts, MIN_POSITION_COUNTS, MAX_POSITION_COUNTS));
//        motorMain.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        motorMain.setVelocity(4000);
//        motorSecondary.setPower(motorMain.getPower());
        MotorController.setTarget(counts);
        periodic();
        //while (!motor.isBusy()){}

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
        return(motorMain.getCurrentPosition());
    }


    public double getVelocity() {
        return(motorMain.getVelocity());
    }


    public boolean inMotion() {
        return motorMain.isBusy();
    }


    public void resetEncoders() {
        motorMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public int degreesToCounts(double degrees) {
        return (int)(degrees / TeamConstants.DEGREES_PER_COUNT);
    }



    public double countsToDegrees(double counts) {
        return (counts * TeamConstants.DEGREES_PER_COUNT);
    }

    public double getPosDegrees() {
        return countsToDegrees(motorMain.getCurrentPosition());
    }

    public int getmotorPos() { return motorMain.getCurrentPosition(); }


    public boolean getBusy(){
        return motorMain.isBusy();
    }

    public boolean closeEnough() {
        return Math.abs(motorMain.getCurrentPosition() - motorMain.getTargetPosition()) * TeamConstants.DEGREES_PER_COUNT < TeamConstants.closeEnoughDegTol;
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

                return true;

        }

    }


    public class ArmTrig implements Action {

        public double newx;
        public double newy;

        public ArmTrig(double x, double y){newx = x; newy = y;}

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            triangulateTo(newx, newy);
            if (motorMain.isBusy()){
                return true;
            }
            else return false;
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
