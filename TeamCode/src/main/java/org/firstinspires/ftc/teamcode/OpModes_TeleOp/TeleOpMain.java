package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Cogintilities.GamepadWrapper;
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
        driver = new GamepadWrapper(gamepad1);
        operator = new GamepadWrapper(gamepad2);

        wristRotate.setPosition(0.5);
        waitForStart();

        while (opModeIsActive()) {

            if(operator.a.pressed()) gripper.toggleGripper();
            wristRotate.setPosition(operator.leftStick_X * 0.5 + 0.5);
            wristPivot.setPosition(operator.leftStick_Y  * 0.5 + 0.5);

            telemetry.addData("GripServo: ", gripper.servoPos());
            telemetry.update();

            driver.update();
            operator.update();


        }
    }
}

