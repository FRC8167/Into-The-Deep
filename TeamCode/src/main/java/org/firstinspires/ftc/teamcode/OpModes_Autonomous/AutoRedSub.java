package org.firstinspires.ftc.teamcode.OpModes_Autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

//@Disabled
@Autonomous(name="AutoRedSub", group="Autonomous", preselectTeleOp = "TeleOp")
public class AutoRedSub extends RobotConfiguration implements TeamConstants {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-12,-63.5, Math.PI/2);
        initializeRobot(initialPose);
        AutoWristX = 288.500/25.4;
        AutoWristY = -288.500/25.4;
        InitAuto = true;
        setAlliance(AllianceColor.RED);

        armPivot.resetEncoders();
        slide.resetEncoders();

       // ************************TRAJECTORIES****************************
        TrajectoryActionBuilder blank = autoDrive.actionBuilder(initialPose)
                .waitSeconds(0);

        TrajectoryActionBuilder centerX = autoDrive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-8,-(33.5+15)));

        TrajectoryActionBuilder sample1 = centerX.endTrajectory().fresh()
                .setTangent(Math.toRadians(60+180))
                .splineToLinearHeading(new Pose2d(-48,-45, Math.toRadians(-90+180)), Math.toRadians(-50+180));

        TrajectoryActionBuilder drop1 = sample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-59, -60), Math.toRadians(45+180));
        TrajectoryActionBuilder sample2 = drop1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58, -44), Math.toRadians(270+180));
        TrajectoryActionBuilder drop2 = sample2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-59, -60), Math.toRadians(45+180));
        TrajectoryActionBuilder sample3Back = drop2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-38, -29), Math.toRadians(0+180));
        TrajectoryActionBuilder sample3 = sample3Back.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-50, -29), Math.toRadians(0+180));
        TrajectoryActionBuilder drop3 = sample3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-59, -60), Math.toRadians(45+180));
        TrajectoryActionBuilder prepTouch = drop3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-48, -12), Math.toRadians(180+180));
        TrajectoryActionBuilder touch = prepTouch.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-26, -12), Math.toRadians(180+180));



        //**************************TRAJECTORIES -> ACTIONS  *********************

        Action goCenterX = centerX.build();
        Action goSample1 = sample1.build();
        Action goDrop1 = drop1.build();
        Action goSample2 = sample2.build();
        Action goDrop2 = drop2.build();
        Action goSample3 = sample3.build();
        Action goSample3Back = sample3Back.build();
        Action goDrop3 = drop3.build();
        Action goPrepTouch = prepTouch.build();
        Action goTouch = touch.build();




        waitForStart();

        if (isStopRequested()) return;




        //************************** RUN THE ACTIONS  ****************************
        Actions.runBlocking(
                new SequentialAction(
                        armPivot.armTrig(20,11),
                        new ParallelAction(
                                slide.slideTrig(20,11),
                                wristPivot.wristTrig(20,11, true),
                                goCenterX
                        ),
                        new SleepAction(0.5),
                        armPivot.armTrig(20,7),
                        slide.slideTrig(20,7),
                        wristPivot.wristTrig(20,7, true),
                        new SleepAction(0.1),
                        gripper.toggle(),
                        new SleepAction(0.5),
//                        new ParallelAction(
                        goSample1,

                        slide.slideToPosition(0),
                        armPivot.armTrig(28,0),
                        new SleepAction(0.5),
                        wristPivot.wristTrig(22.5,-7.2, true),
                        slide.slideTrig(28,0),
                        slide.slideTrig(22.5,-7.2),
                        armPivot.armTrig(22.5,-7.2),

                        new SleepAction(0.5), // 1
                        gripper.toggle(),
                        new SleepAction(0.5),
                        armPivot.armTrig(14,33),
                        new ParallelAction(
                                slide.slideTrig(14,33),
                                wristPivot.wristTrig(14,33, true),
                                goDrop1
                        ),
                        new SleepAction(0.5),
                        gripper.toggle(),
                        new SleepAction(0.5),
                        goSample2,
                        slide.slideToPosition(0),
                        armPivot.armTrig(28,0),
                        wristPivot.wristTrig(22.5,-7.2, true),
                        new SleepAction(0.5),
                        slide.slideTrig(28,0),
                        slide.slideTrig(22.5,-7.2),
                        armPivot.armTrig(22.5,-7.2),
                        new SleepAction(0.5), // 1
                        gripper.toggle(),
                        new SleepAction(0.5),
                        armPivot.armTrig(14,33),
                        new ParallelAction(
                                slide.slideTrig(14,33),
                                wristPivot.wristTrig(14,33, true),
                                goDrop2
                        ),
                        new SleepAction(0.5),//
                        gripper.toggle(),
                        new SleepAction(0.5),
                        goPrepTouch,
                        new ParallelAction(
                                armPivot.armTrig(16,10),
                                slide.slideTrig(16,10),
                                wristPivot.wristTrig(16,10, true),
                                goTouch
                        ),
                        armPivot.armTrig(16,6.5),
                        slide.slideTrig(16,6.5),
                        wristPivot.wristTrig(16,6.5, true)
                )
        );



//        Actions.runBlocking(wristPivot.setServoPosition(0.2));
        AutoWristX = 16;
        AutoWristY = 7.4;
        EndPos = new Pose2d(new Vector2d(-26, -12), Math.toRadians(360));
        telemetry.update();


    }

    }

