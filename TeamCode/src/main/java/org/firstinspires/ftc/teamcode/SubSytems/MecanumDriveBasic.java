package org.firstinspires.ftc.teamcode.SubSytems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

public class MecanumDriveBasic implements TeamConstants {

    private final DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private double drive, strafe, turn;
    boolean degradedMode;

    /**
     * CONSTRUCTOR Create a mecanum drive object using four motors
     * @param leftFront     Left front motor name from the hardware map
     * @param leftRear      Left rear motor name from the hardware map
     * @param rightFront    Right front motor name from the hardware map
     * @param rightRear     Right rear motor name from the hardware map
     */
    public MecanumDriveBasic(DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightFront, DcMotorEx rightRear) {

        this.leftFront  = leftFront;
        this.leftRear   = leftRear;
        this.rightFront = rightFront;
        this.rightRear  = rightRear;

        /* Assign Motor Directions */
        this.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        this.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        this.leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        this.rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        /* Initialize Motor Power to 0 */
        degradedMode = false;
        setMotorPower(0,0,0,0);
        drive = strafe = turn = 0;
    }


    /**
     * Drive robot using mecanum drive wheels. Calculates the motor power required for the given
     * inputs and reduces power if the degradedMode is true. This is intended to use joystick inputs
     * for the commands and range between 0.0 and 1.0.  There are no checks to ensure the values are in
     * range.
     * @param driveCmd      Drive command, typically gamepad.left_stick_y (negated)
     * @param strafeCmd     Strafe command, typically gamepad.Left_stick_x
     * @param turnCmd       Turn command, typically gamepad.Right_stick_s
     */
    public void mecanumDrive(double driveCmd, double strafeCmd, double turnCmd) {

        drive  = (degradedMode) ? driveCmd  * DEGRADED_DRIVE_LIMIT  : driveCmd;
        strafe = (degradedMode) ? strafeCmd * DEGRADED_STRAFE_LIMIT : strafeCmd;
        turn   = (degradedMode) ? turnCmd   * DEGRADED_TURN_LIMIT   : turnCmd;

        double denominator = Math.max(Math.abs(driveCmd) + Math.abs(strafeCmd) + Math.abs(turnCmd), 1);
        double frontLeftPower  = (drive + strafe + turn) / denominator;
        double backLeftPower   = (drive - strafe + turn) / denominator;
        double frontRightPower = (drive - strafe - turn) / denominator;
        double backRightPower  = (drive + strafe - turn) / denominator;

        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }


    /**
     * Limit applied power to the motors.  Degraded power levels are set in TeamConstants
     * @param snail Recommend setting using digitalInput whilepressed() method.
     */
    public void setDegradedDrive(boolean snail) {
        degradedMode = snail;
    }


    /**
     * Limit applied power to the drive motors while the slide is extneded or arm pivoted beyond
     * set levels. Slide and rotation levels set in TeamConstants
     * @param slidePosition Current slide position
     * @param rotationLimit Current arm rotation position
     */
    public void periodic(double slidePosition, double rotationLimit) {
        degradedMode = (slidePosition > DEGRADED_SLIDE_EXTENDED || rotationLimit > DEGRADED_ARM_ROTATION);
    }


    /**
     * Sets the power level of the four drive motors. Input must range between 0.0 and 1.0
     * @param lfPower Left front motor power
     * @param rfPower right front motor power
     * @param lrPower left rear motor power
     * @param rrPower right rear motor power
     */
    private void setMotorPower(double lfPower,double rfPower, double lrPower, double rrPower) {
        leftFront.setPower(lfPower);
        rightFront.setPower(rfPower);
        leftRear.setPower(lrPower);
        rightRear.setPower(rrPower);
    }


    /** Setters and Getters **/
    public double getDriveCmd() { return drive;  }
    public double getTurnCmd()  { return turn;   }
    public double getStrafe()   { return strafe; }
    public double getLFpos()    { return leftFront.getCurrentPosition(); }
    public double geLRpos()     { return leftRear.getCurrentPosition(); }
    public double geRFpos()     { return rightFront.getCurrentPosition(); }
    public double geRRpos()     { return rightRear.getCurrentPosition(); }
    public double getLFpower()  { return leftFront.getPower(); }
    public double getLRpower()  { return leftRear.getPower(); }
    public double getRFpower()  { return rightFront.getPower(); }
    public double getRRpower()  { return rightRear.getPower(); }
}
