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

        initializeRobot(new Pose2d(0,0,0));

        /* For starting directly in TeleOp only */
        armPivot.resetEncoders();
        /* ************************************ */

        driver   = new GamepadWrapper(gamepad1);
        operator = new GamepadWrapper(gamepad2);
        double RotateAcuteAng;
        waitForStart();

        while (opModeIsActive()) {
//            RotateAcuteAng = Math.abs(Math.toDegrees(Math.atan2(-1*operator.leftStick_Y, operator.leftStick_X)));
//            /* ********* Created for wrist proof of concept ********* */
            if(operator.a.pressed()) gripper.toggleGripper();

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

