package org.firstinspires.ftc.teamcode.OpModes_Autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
import org.firstinspires.ftc.teamcode.SubSytems.ServoPivot;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.RotatedRect;

//@Disabled
@Autonomous(name="AutoBlueFarObs", group="Autonomous", preselectTeleOp = "TeleOp")
public class AutoBlueFarObs extends RobotConfiguration implements TeamConstants {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-12,60, -Math.PI/2);
        initializeRobot(initialPose);
        setAlliance(AllianceColor.BLUE);

       // ************************TRAJECTORIES****************************

        TrajectoryActionBuilder centerX = autoDrive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0,60));
        TrajectoryActionBuilder forward1 = autoDrive.actionBuilder(new Pose2d(0, 60, -Math.PI/2))
                .lineToY(30);
        TrajectoryActionBuilder block1 = autoDrive.actionBuilder(new Pose2d(0, 30, -Math.PI/2))
                 .strafeTo(new Vector2d(50,36));
        TrajectoryActionBuilder basket1 = autoDrive.actionBuilder(new Pose2d(50, 36, -Math.PI/2))
                .strafeToSplineHeading(new Vector2d(55,55),Math.toRadians(45));
        TrajectoryActionBuilder block2 = autoDrive.actionBuilder(new Pose2d(55, 55, Math.toRadians(45)))
                .strafeToSplineHeading(new Vector2d(57.5,38),Math.toRadians(-90));
        TrajectoryActionBuilder basket2 = autoDrive.actionBuilder(new Pose2d(57.5, 38, Math.toRadians(-90)))
                .strafeToSplineHeading(new Vector2d(55,55),Math.toRadians(45));
        TrajectoryActionBuilder block3 = autoDrive.actionBuilder(new Pose2d(55, 55, Math.toRadians(45)))
                .strafeToSplineHeading(new Vector2d(55,25),Math.toRadians(0));
        TrajectoryActionBuilder basket3 = autoDrive.actionBuilder(new Pose2d(55, 25, 0))
                .strafeToSplineHeading(new Vector2d(55,55),Math.toRadians(45));
        TrajectoryActionBuilder park = autoDrive.actionBuilder(new Pose2d(55, 55, Math.toRadians(45)))
                .strafeToSplineHeading(new Vector2d(-62,60),Math.toRadians(90));



        waitForStart();

        if (isStopRequested()) return;

        //**************************TRAJECTORIES -> ACTIONS  *********************

        Action goForward1 = forward1.build();
        Action goCenterX = centerX.build();
        Action goBlock1 = block1.build();
        Action goBasket1 = basket1.build();
        Action goBlock2 = block2.build();
        Action goBasket2 = basket2.build();
        Action goBlock3 = block3.build();
        Action goBasket3 = basket3.build();
        Action goPark = park.build();



        //************************** RUN THE ACTIONS  ****************************
        Actions.runBlocking(
                new SequentialAction(
                        armPivot.armTrig(20,3),
                        slide.slideTrig(20,3),
                        wristPivot.wristTrig(16.5,3, true),
                        goCenterX,
                        goForward1,
                        armPivot.armTrig(30,10),
                        slide.slideTrig(30,10),
                        wristPivot.wristTrig(30,10, true)
//                        goBlock1,
//                        goBasket1,
//                        goBlock2,
//                        goBasket2,
//                        goBlock3,
//                        goBasket3,
//                        goPark
                )
        );



//        Actions.runBlocking(wristPivot.setServoPosition(0.2));

        telemetry.update();

        }

    }

