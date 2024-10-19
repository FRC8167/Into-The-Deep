package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

@Disabled
@TeleOp(name="TeleOpTemplate", group="Competition")
public class TeleOpTemplate extends RobotConfiguration implements TeamConstants {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
