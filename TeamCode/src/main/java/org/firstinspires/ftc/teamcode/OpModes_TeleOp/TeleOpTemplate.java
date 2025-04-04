package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

@Disabled
@TeleOp(name="TeleOpTemplate", group="Competition")
public class TeleOpTemplate extends RobotConfiguration implements TeamConstants {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot(new Pose2d(0,0,0), false);

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
