package org.firstinspires.ftc.teamcode.OpModes_Autonomous;

import android.annotation.SuppressLint;

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
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.RotatedRect;

import java.util.List;

//@Disabled
@Autonomous(name="AutoRR", group="Autonomous", preselectTeleOp = "TeleOp")
public class AutoRR extends RobotConfiguration implements TeamConstants {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();
        setAlliance(AllianceColor.BLUE); /* OR */ //setAlliance(AllianceColor.RED);

        Pose2d initialPose = new Pose2d(10,60, Math.toRadians(-90));

        TrajectoryActionBuilder driveToNoWhere = autoDrive.actionBuilder(initialPose)
                .lineToY(35)
                .waitSeconds(3)
                .strafeTo(new Vector2d(48,38))
                .waitSeconds(2)
                .strafeToSplineHeading(new Vector2d(55,55),Math.toRadians(45))
                .waitSeconds(2)
                .strafeToSplineHeading(new Vector2d(57.5,38),Math.toRadians(-90))
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(55,55),Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToSplineHeading(new Vector2d(55,25),Math.toRadians(0))
                .waitSeconds(1.5)
                .strafeToSplineHeading(new Vector2d(55,55),Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToSplineHeading(new Vector2d(30,10),Math.toRadians(180));

        Action letsDriveToKnowWhere = driveToNoWhere.build();

        waitForStart();

        for(ColorBlobLocatorProcessor.Blob b : vision.blueBlobs())
        {
            RotatedRect boxFit = b.getBoxFit();
            telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
        }
//        for(ColorBlobLocatorProcessor.Blob b : vision.yellowBlobs())
//        {
//            RotatedRect boxFit = b.getBoxFit();
//            telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
//                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
//        }

        Actions.runBlocking(letsDriveToKnowWhere);


        Actions.runBlocking(wristPivot.setServoPosition(0.2));
        telemetry.update();

        }

    }

