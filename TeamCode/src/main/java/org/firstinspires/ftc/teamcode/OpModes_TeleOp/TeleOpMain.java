package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Cogintilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

//@Disabled
@TeleOp(name="TeleOpMain", group="Competition")
public class TeleOpMain extends RobotConfiguration implements TeamConstants {

    GamepadWrapper driver;
    GamepadWrapper operator;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();

        driver   = new GamepadWrapper(gamepad1);
        operator = new GamepadWrapper(gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            /* ********* Created for wrist proof of concept ********* */
            if(operator.a.pressed()) gripper.toggleGripper();
            wristRotate.setPosition(-operator.leftStick_Y * 0.5 + 0.5);
            wristPivot.setPosition(-operator.leftStick_X  * 0.5 + 0.5);
            /* ********************************************************/


            /* Ouput Telemtery Data to Driver Stations */
            telemetry.addData("GripServo: ", gripper.servoPos());
            telemetry.addData("WristRotate: ", wristRotate.servoPos());

            telemetry.update();

            periodicCalls();
        }
    }


    private void periodicCalls() {
        driver.update();
        operator.update();
//        drive.periodic(getSlidePosition(), getPivotPosition());
    }
}

