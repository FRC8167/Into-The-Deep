package org.firstinspires.ftc.teamcode.OpModes_Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

//@Disabled
@Autonomous(name="AutoTeamColorDescriptor", group="Autonomous", preselectTeleOp = "TeleOp")
public class AutoRR extends RobotConfiguration implements TeamConstants {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();
        setAlliance(AllianceColor.RED); /* OR */ setAlliance(AllianceColor.BLUE);

        Pose2d initialPose = new Pose2d(0,0, Math.toRadians(90));

        TrajectoryActionBuilder driveToNoWhere = autoDrive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);

        Action letsDriveToKnowWhere = driveToNoWhere.build();

        waitForStart();

        Actions.runBlocking(letsDriveToKnowWhere);


        Actions.runBlocking(wristPivot.setServoPosition(0.2));


        }

    }

