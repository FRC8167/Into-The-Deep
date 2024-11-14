package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

@Disabled
@TeleOp(name="PoseEstimator", group="Competition")
public class TeleOpPoseEstimation extends RobotConfiguration implements TeamConstants {

    @Override
    public void runOpMode() throws InterruptedException {

        /* Position robot and the center of the play pen, heading facing 0 degrees */
        initializeRobot(new Pose2d(0,0,0));

        waitForStart();

        while (opModeIsActive()) {

            drive.mecanumDrive(0,0,0);

        }
    }
}
