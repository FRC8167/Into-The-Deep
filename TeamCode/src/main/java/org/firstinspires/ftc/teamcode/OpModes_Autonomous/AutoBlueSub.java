package org.firstinspires.ftc.teamcode.OpModes_Autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DisplacementTrajectory;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

//@Disabled
@Autonomous(name="AutoBlueSub", group="Autonomous", preselectTeleOp = "TeleOp")
public class AutoBlueSub extends RobotConfiguration implements TeamConstants {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(12,63.5, -Math.PI/2);
        initializeRobot(initialPose);
        AutoWristX = 288.500/25.4;
        AutoWristY = -288.500/25.4;
        InitAuto = true;
        setAlliance(AllianceColor.BLUE);

        armPivot.resetEncoders();
        slide.resetEncoders();

       // ************************TRAJECTORIES****************************
        TrajectoryActionBuilder blank = autoDrive.actionBuilder(initialPose)
                .waitSeconds(0);

        TrajectoryActionBuilder centerX = autoDrive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(8,33.5+15));

        TrajectoryActionBuilder sample1 = centerX.endTrajectory().fresh()
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(50,46, Math.toRadians(-90)), Math.toRadians(-20));
        TrajectoryActionBuilder drop1 = sample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(58, 61), Math.toRadians(45));
        TrajectoryActionBuilder sample2 = drop1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(58, 46), Math.toRadians(270));
        TrajectoryActionBuilder drop2 = sample2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(58, 61), Math.toRadians(45));
        TrajectoryActionBuilder sample3Back = drop2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(38, 29), Math.toRadians(0));
        TrajectoryActionBuilder sample3 = sample3Back.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(50, 29), Math.toRadians(0));
        TrajectoryActionBuilder drop3 = sample3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(58, 61), Math.toRadians(45));
        TrajectoryActionBuilder prepTouch = drop3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(48, 12), Math.toRadians(180));
        TrajectoryActionBuilder touch = prepTouch.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(24, 12), Math.toRadians(180));



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
                        armPivot.armTrig(20,13),
                        slide.slideTrig(20,13),
                        wristPivot.wristTrig(20,13, true),
                        goCenterX,
                        new SleepAction(0.5),
                        armPivot.armTrig(20,10),
                        slide.slideTrig(20,10),
                        wristPivot.wristTrig(20,10, true),
                        new SleepAction(1),
                        gripper.toggle(),
                        new SleepAction(1),
                        new ParallelAction(
                                goSample1,
                                armPivot.armTrig(16.1,0),
                                slide.slideTrig(16.1,0),
                                wristPivot.wristTrig(16.1,0, true)
//                                wristRotate.rotateTrig(0),
                        ),
                        armPivot.armTrig(24,-6.5),
                        slide.slideTrig(24,-6.5),
                        wristPivot.wristTrig(24,-6.5, true),
                        new SleepAction(1.5),
                        new SequentialAction(
                                gripper.toggle(),
                                new SleepAction(2)
                        ),
                        armPivot.armTrig(13,33),
                        slide.slideTrig(13,33),
                        wristPivot.wristTrig(13,33, true),
                        goDrop1,
                        gripper.toggle(),
                        new SleepAction(1),
                        goSample2,
                        slide.slideTrig(24,-6.5),
                        armPivot.armTrig(24,-6.5),
                        wristPivot.wristTrig(24,-6.5, true),
                        new SleepAction(1),
                        new SequentialAction(
                            gripper.toggle(),
                            new SleepAction(2)
                        ),
                        armPivot.armTrig(13,33),
                        slide.slideTrig(13,33),
                        wristPivot.wristTrig(13,33, true),
                        goDrop2,
                        new SleepAction(1.5),
                        gripper.toggle(),
                        new SleepAction(1),
                        goPrepTouch,
                        new ParallelAction(
                            armPivot.armTrig(16,11.4),
                            slide.slideTrig(16,11.4),
                            wristPivot.wristTrig(16,11.4, true),
                            goTouch
                        ),
                        armPivot.armTrig(16,7),
                        slide.slideTrig(16,7),
                        wristPivot.wristTrig(16,7, true)
                )
        );



//        Actions.runBlocking(wristPivot.setServoPosition(0.2));
        AutoWristX = 16;
        AutoWristY = 7.4;
        telemetry.update();

        }

    }

