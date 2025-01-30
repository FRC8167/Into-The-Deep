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
                .strafeTo(new Vector2d(8,33.5+15-1));

        TrajectoryActionBuilder sample1 = centerX.endTrajectory().fresh()
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(48,46, Math.toRadians(-90)), Math.toRadians(-50));

        TrajectoryActionBuilder drop1 = sample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(58, 61), Math.toRadians(45));
        TrajectoryActionBuilder sample2 = drop1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(58, 46), Math.toRadians(270));
        TrajectoryActionBuilder drop2 = sample2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(58, 61), Math.toRadians(45));

        TrajectoryActionBuilder prepTouch = drop2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(48, 12), Math.toRadians(180));
        TrajectoryActionBuilder touch = prepTouch.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(28, 12), Math.toRadians(180));



        //**************************TRAJECTORIES -> ACTIONS  *********************

        Action goCenterX = centerX.build();
        Action goSample1 = sample1.build();
        Action goDrop1 = drop1.build();
        Action goSample2 = sample2.build();
        Action goDrop2 = drop2.build();
        Action goPrepTouch = prepTouch.build();
        Action goTouch = touch.build();




        waitForStart();

        if (isStopRequested()) return;




        //************************** RUN THE ACTIONS  ****************************
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                        armPivot.armTrig(20,13),//15
                                new SequentialAction(
                                        new SleepAction(1),
                                        new ParallelAction(
                                                slide.slideTrig(20,13),
                                                wristPivot.wristTrig(20,13, true),
                                                goCenterX
                                        )
                                )
                        ),
//                        new SleepAction(0.5),
                        armPivot.armTrig(20,10),
                        slide.slideTrig(20,10),
                        wristPivot.wristTrig(20,10, true),
                        new SleepAction(0.1),
//                        gripper.toggle(),
                        new SleepAction(0.5),
                        new ParallelAction(
                                gripper.spin(),
                                goSample1,
                                new SequentialAction(
                                      new SleepAction(0.5),
                                        gripper.toggle()
                                )
                        ),
//                        gripper.toggle(),
                        gripper.eOpen(),
                        new SleepAction(0.5),
//                        new ParallelAction(

//                                armPivot.armTrig(16.1,0),
//                                slide.slideTrig(16.1,0),
//                                wristPivot.wristTrig(16.1,0, true)
////                                wristRotate.rotateTrig(0),
//                        ),
                        slide.slideToPosition(0),
                        armPivot.armTrig(28,-7.2),
                        wristPivot.wristTrig(28,-7.2, true),
                        new SleepAction(1),
                        slide.slideTrig(28,0),
                        new SleepAction(0.25),
                        new ParallelAction(
                        slide.slideTrig(24,-7.2),
                        armPivot.armTrig(24,-7.2),
                        wristPivot.wristTrig(24,-7.2, true)
                                ),
//                        slide.slideTrig(28,0),
//                        armPivot.armTrig(28,0),
//                        wristPivot.wristTrig(28,-6.8, true),
//                        armPivot.armTrig(24,-6.8),
//                        slide.slideTrig(24,-6.8),
//                        wristPivot.wristTrig(24,-6.8, true),
                        new SleepAction(0.5),//1.5
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
                        gripper.eOpen(),
                        slide.slideToPosition(0),
                        armPivot.armTrig(28,0),
                        new SleepAction(0.5),
                        slide.slideTrig(28,0),
                        new SleepAction(0.25),
                        new ParallelAction(
                        slide.slideTrig(24,-7.2),
                        armPivot.armTrig(24,-7.2),
                        wristPivot.wristTrig(24,-7.2, true)
                                ),
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
        EndPos = new Pose2d(new Vector2d(28, 12), Math.toRadians(180));
        telemetry.update();

        }

    }

