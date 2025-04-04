package org.firstinspires.ftc.teamcode.OpModes_Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

@Disabled
@Autonomous(name="AutoTeamColorDescriptor", group="Autonomous", preselectTeleOp = "TeleOp")
public class AutonomousOpTemplate extends RobotConfiguration implements TeamConstants {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(0,0,0);

        initializeRobot(startPose, true);
        /* First line of code after initializing the robot should be to set the alliance color.
           This is needed for April Tags, TFOD object files or any other game element that is unique
           to the Red or Blue Alliance. */
        setAlliance(AllianceColor.RED); /* OR */ setAlliance(AllianceColor.BLUE);

        waitForStart();


    }
}
