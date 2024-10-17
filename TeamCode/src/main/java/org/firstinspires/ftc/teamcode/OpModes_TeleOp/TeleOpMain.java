package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Cogintilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

@Disabled
@TeleOp(name="TeleOpMode", group="Competition")
public class TeleOpMain extends RobotConfiguration implements TeamConstants {

    GamepadWrapper driver;
    GamepadWrapper operator;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();
        driver = new GamepadWrapper(gamepad1);
        operator = new GamepadWrapper(gamepad2);

        waitForStart();

        while (opModeIsActive()) {


        }
    }
}
