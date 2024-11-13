package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Cogintilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
import org.firstinspires.ftc.teamcode.SubSytems.MotorPivot;

//@Disabled
@TeleOp(name="TeleOpMain", group="Competition")
public class TeleOpMain extends RobotConfiguration implements TeamConstants {

    GamepadWrapper driver;
    GamepadWrapper operator;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot(new Pose2d(0,0,0));

        driver   = new GamepadWrapper(gamepad1);
        operator = new GamepadWrapper(gamepad2);

        waitForStart();

        while (opModeIsActive()) {

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

            /* Output Telemetry Data to Driver Stations */
            telemetry.addData("Left Motor Pos: ", armPivot.getLmotorPos());
            telemetry.addData("Right Motor Pos: ", armPivot.getRmotorPos());
            telemetry.addData("GripServo: ", gripper.servoPos());
            telemetry.addData("WristRotate: ", wristRotate.servoPos());
            telemetry.addData("Pose X: ", autoDrive.pose.position.x);
            telemetry.addData("Pose X: ", autoDrive.pose.position.y);
            telemetry.addData("Pose Heading: ", autoDrive.pose.heading);

            telemetry.update();
            periodicCalls();
        }
    }


    private void periodicCalls() {
        driver.update();
        operator.update();
        armPivot.periodic(30);
//        drive.periodic(getSlidePosition(), getPivotPosition());
    }
}

