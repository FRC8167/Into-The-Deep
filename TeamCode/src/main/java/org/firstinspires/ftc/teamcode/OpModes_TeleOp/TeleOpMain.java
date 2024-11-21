package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Cogintilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
import org.firstinspires.ftc.teamcode.SubSytems.MotorPivot;
import org.firstinspires.ftc.teamcode.SubSytems.ServoRotate;

//@Disabled
@TeleOp(name="TeleOpMain", group="Competition")
public class TeleOpMain extends RobotConfiguration implements TeamConstants {

    GamepadWrapper driver;
    GamepadWrapper operator;


    @Override
    public void runOpMode() throws InterruptedException {
        double wristX = 288.500/25.4;// ~11.358in
        double wristY = -288.500/25.4;
        // Looking from right side of robot
        // (0,0) at arm pivot
        // Units in inches


        telemetry.addData("Test: ", initializeRobot(new Pose2d(0,0,0)));
        telemetry.update();

        /* For starting directly in TeleOp only */
        armPivot.resetEncoders();
        slide.resetEncoders();
        /* ************************************ */

        driver   = new GamepadWrapper(gamepad1);
        operator = new GamepadWrapper(gamepad2);

        double RotateAcuteAng;
        waitForStart();

        while (opModeIsActive()) {
            double fdrive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            drive.mecanumDrive(fdrive, strafe, turn);
            wristY += (operator.rightTrigger-operator.leftTrigger);
            wristX += (operator.rightStick_Y);
            if (Math.sqrt(wristX*wristX+wristY*wristY) < (408/25.4)){
                wristX = wristX/(Math.sqrt(wristX*wristX+wristY*wristY/(408/25.4)));
                wristY = wristY/(Math.sqrt(wristX*wristX+wristY*wristY/(408/25.4)));
            }
            if (Math.sqrt(wristX*wristX+wristY*wristY) > (TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT)){
                wristX = wristX/(Math.sqrt(wristX*wristX+wristY*wristY/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT)));
                wristY = wristY/(Math.sqrt(wristX*wristX+wristY*wristY/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT)));
            }
//            RotateAcuteAng = Math.abs(Math.toDegrees(Math.atan2(-1*operator.leftStick_Y, operator.leftStick_X)));
//            /* ********* Created for wrist proof of concept ********* */
            if(operator.a.pressed()) gripper.toggleGripper();
            wristPivot.setPosition(operator.rightStick_X+.85);

//            armPivot.triangulateTo(wristX, wristY);
//            slide.triangulateTo(wristX, wristY);

//            //wristRotate.setPosition(-operator.leftStick_X* 0.5 + 0.5);//* 0.5 + 0.5
//            wristPivot.setPosition(-operator.rightStick_Y * 0.5 + 0.5);
//            if (operator.leftStick_Y == 0 && operator.leftStick_X == 0) wristRotate.setPosition(TeamConstants.WRIST_ROTATE_CENTER);
//            else if (operator.leftStick_Y<= 0) wristRotate.setPosition(((((RotateAcuteAng)/(300))+.2)));
            /* ********************************************************/
            RotateAcuteAng = wristRotate.moveTrig(operator.leftStick_X, operator.leftStick_Y);

            /* Output Telemtery Data to Driver Stations */
            telemetry.addData("GripServo: ", gripper.servoPos());
            telemetry.addData("WristRotate: ", wristRotate.servoPos());
            telemetry.addData("WristPivot: ", wristPivot.servoPos());
            telemetry.addData("RotateTest: ", (((((RotateAcuteAng)/(300))+.2))));
            telemetry.addData("Ang: ", RotateAcuteAng);
            telemetry.addData("LX: ", operator.leftStick_X);
            telemetry.addData("LY: ", operator.leftStick_Y);
            telemetry.addData("RY: ", operator.rightStick_Y);
            telemetry.addData("R; ", armPivot.getRmotorPos());
            telemetry.addData("L; ", armPivot.getLmotorPos());

//            drive.mecanumDrive(-driver.leftStick_Y, driver.leftStick_X, driver.rightStick_X);
//
//            /* ********* Created for wrist proof of concept ********* */
//            if(operator.a.pressed()) gripper.toggleGripper();
//            wristRotate.setPosition(-operator.leftStick_Y * 0.5 + 0.5);
//            wristPivot.setPosition(-operator.leftStick_X  * 0.5 + 0.5);
//            /* ********************************************************/

             if(operator.rightStick_Y > 0.1 || operator.rightStick_Y < -0.1) {
                armPivot.manualMove(operator.rightStick_Y);
             }

             if (operator.rightTrigger > 0.0 || operator.leftTrigger > 0.0)  {
                 slide.manualMove(operator.leftTrigger, operator.rightTrigger);
             }

//            /* Output Telemetry Data to Driver Stations */
//            telemetry.addData("Left Motor Pos: ", armPivot.getLmotorPos());
//            telemetry.addData("Right Motor Pos: ", armPivot.getRmotorPos());
//            telemetry.addData("GripServo: ", gripper.servoPos());
//            telemetry.addData("WristRotate: ", wristRotate.servoPos());
//            telemetry.addData("Pose X: ", autoDrive.pose.position.x);
//            telemetry.addData("Pose X: ", autoDrive.pose.position.y);
//            telemetry.addData("Pose Heading: ", autoDrive.pose.heading);

            telemetry.update();
            periodicCalls();
        }
    }


    private void periodicCalls() {
        driver.update();
        operator.update();
//        armPivot.periodic(18);
//        drive.periodic(getSlidePosition(), getPivotPosition());
    }
}

