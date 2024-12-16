package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Cogintilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
import org.firstinspires.ftc.teamcode.SubSytems.MotorPivotExp;

//@Disabled
@TeleOp(name="TeleOpMain", group="Competition")
public class TeleOpMain extends RobotConfiguration implements TeamConstants {

    GamepadWrapper driver;
    GamepadWrapper operator;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot(new Pose2d(0,0,0));  //will need to chang
        double wristX = 288.500/25.4;// ~11.358in
        double wristY = -288.500/25.4;
        double oldWristX;
        double oldWristY;
        boolean bigMove = false;
        boolean retractIsDone = true;
        boolean pivotIsDone = true;
        boolean extendIsDone = true;
        double newWristX;// ~11.358in
        double newWristY;
        boolean wristForward = true;
        double RotateAcuteAng;
        // Looking from right side of robot
        // (0,0) at arm pivot
        // Units in inches

        telemetry.update();

        /* For starting directly in TeleOp only */
        armPivot.resetEncoders();
        slide.resetEncoders();
        /* ************************************ */

        driver   = new GamepadWrapper(gamepad1);
        operator = new GamepadWrapper(gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            if(driver.rightBumper.pressed()) drive.setDegradedDrive(true);
            if(driver.rightBumper.released()) drive.setDegradedDrive(false);
//            drive.setDegradedDrive(driver.rightBumper.pressed());
            drive.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            oldWristX = wristX;
            oldWristY = wristY;
            if (!bigMove) {
                newWristY = wristY + 0.1 * (operator.rightTrigger - operator.leftTrigger);
                newWristX = wristX + 0.1 * (-operator.rightStick_Y);
                wristX = Functions.TestNewX(wristX, wristY, newWristX, newWristY);
                wristY = Functions.TestNewY(wristX, wristY, newWristX, newWristY);
                wristX = Functions.TriClampX(wristX, wristY);
                wristY = Functions.TriClampY(wristX, wristY);
            }
            wristForward = wristPivot.moveByPos(wristX,wristY,wristForward);

            /* ********* Created for wrist proof of concept ********* */
            if(operator.a.pressed()) gripper.toggleGripper();

//            wristPivot.setPosition(operator.rightStick_X+.85);




            if(operator.y.pressed()) {
                wristX = 5;
                wristY = 34;
            }
            if(operator.x.pressed()) {
                wristX = 23.5;
                wristY = 0;
            }
            if(operator.b.pressed()) {
                wristX = 17;
                wristY = 0;
            }


//            //wristRotate.setPosition(-operator.leftStick_X* 0.5 + 0.5);//* 0.5 + 0.5
//            wristPivot.setPosition(-operator.rightStick_Y * 0.5 + 0.5);
//            if (operator.leftStick_Y == 0 && operator.leftStick_X == 0) wristRotate.setPosition(TeamConstants.WRIST_ROTATE_CENTER);
//            else if (operator.leftStick_Y<= 0) wristRotate.setPosition(((((RotateAcuteAng)/(300))+.2)));
            /* ********************************************************/

            wristRotate.moveTrig(operator.leftStick_X, operator.leftStick_Y);

            if (Math.sqrt((wristX-oldWristX)*(wristX-oldWristX)+(wristY-oldWristY)*(wristY-oldWristY)) > TeamConstants.bigMoveTolerance)
            {
                bigMove = true;
                retractIsDone = false;
                pivotIsDone = false;
                extendIsDone = false;
            }

            if (!bigMove) {
                armPivot.triangulateTo(wristX, wristY);
                slide.triangulateTo(wristX, wristY);
            }
            else {
                if (!retractIsDone) {
                    slide.setPosition(0);
//                    if (!slide.getBusy()){sleep(50);}
                    retractIsDone = slide.closeEnough();
                } else if (!pivotIsDone) {
                    armPivot.triangulateTo(wristX, wristY);
//                    if (!armPivot.getBusy()){sleep(50);}
                    pivotIsDone = armPivot.closeEnough();
                } else if (!extendIsDone) {
//                    if (!slide.getBusy()){sleep(50);}
                    extendIsDone = slide.closeEnough();
                } else {
                    bigMove = false;
                }
            }

            /* Output Telemtery Data to Driver Stations */
            telemetry.addData("GripServo: ", gripper.servoPos());
            telemetry.addData("WristRotate: ", wristRotate.servoPos());
            telemetry.addData("WristPivot: ", wristPivot.servoPos());
            telemetry.addData("Pivot Encoder", armPivot.getPosition());
            telemetry.addData("RotateTest: ", wristRotate.getRotateAcuteAng() / 300 + 0.2);
            telemetry.addData("Ang: ", wristRotate.getRotateAcuteAng());
            telemetry.addData("LX: ", operator.leftStick_X);
            telemetry.addData("LY: ", operator.leftStick_Y);
            telemetry.addData("RY: ", operator.rightStick_Y);
            telemetry.addData("Ang: ", (Math.toDegrees(-Math.atan2(wristX, wristY))+135));
            telemetry.addData("TargetX: ", wristX);
            telemetry.addData("TargetY: ", wristY);
            telemetry.addData("Length: ", (Math.sqrt(wristX*wristX+wristY*wristY)));
            telemetry.addData("Test: ", (Math.sqrt(wristX*wristX+wristY*wristY)/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4))));
            telemetry.addData("ClassAngle: ", (wristPivot.getAngle()));
            telemetry.addData("ClassServoPos: ", (wristPivot.getServoPos()));
            telemetry.addData("ClassServoPos: ", (bigMove));
            telemetry.addData("retractDone: ", (retractIsDone));
            telemetry.addData("pivotDone: ", (pivotIsDone));
            telemetry.addData("extendDone: ", (extendIsDone));
            telemetry.addData("bigMove: ", (bigMove));
            telemetry.addData("Dist: ", (Math.sqrt((wristX-oldWristX)*(wristX-oldWristX)+(wristY-oldWristY)*(wristY-oldWristY))));
            telemetry.addData("armBusy: ", (armPivot.getBusy()));
            telemetry.addData("slideBusy: ", (slide.getBusy()));



//            drive.mecanumDrive(-driver.leftStick_Y, driver.leftStick_X, driver.rightStick_X);
//
//            /* ********* Created for wrist proof of concept ********* */
//            if(operator.a.pressed()) gripper.toggleGripper();
//            wristRotate.setPosition(-operator.leftStick_Y * 0.5 + 0.5);
//            wristPivot.setPosition(-operator.leftStick_X  * 0.5 + 0.5);
//            /* ********************************************************/

//             if(operator.rightStick_Y > 0.1 || operator.rightStick_Y < -0.1) {
//                armPivot.manualMove(operator.rightStick_Y);
//             }
//
//             if (operator.rightTrigger > 0.0 || operator.leftTrigger > 0.0)  {
//                 slide.manualMove(operator.leftTrigger, operator.rightTrigger);
//             }

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

