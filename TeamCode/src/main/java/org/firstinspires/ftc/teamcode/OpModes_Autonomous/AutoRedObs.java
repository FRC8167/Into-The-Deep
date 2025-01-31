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
@Autonomous(name="AutoRedObs", group="Autonomous", preselectTeleOp = "TeleOp")
public class AutoRedObs extends RobotConfiguration implements TeamConstants {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(12,-63.5, Math.PI/2);
        initializeRobot(initialPose);
        AutoWristX = 288.500/25.4;
        AutoWristY = -288.500/25.4;
        InitAuto = true;
        setAlliance(AllianceColor.RED);

        armPivot.resetEncoders();
        slide.resetEncoders();

       // ************************TRAJECTORIES****************************


        TrajectoryActionBuilder centerX = autoDrive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(4,-(33.5+15-1)));

        TrajectoryActionBuilder back1 = centerX.endTrajectory().fresh()
                .strafeTo(new Vector2d(10,-55));
        TrajectoryActionBuilder toSample1 = back1.endTrajectory().fresh()
                .setTangent(Math.toRadians(90+180))
                .splineToSplineHeading(new Pose2d(43, -30, Math.toRadians(180+180)), Math.toRadians(-135+180))
                .strafeToSplineHeading(new Vector2d(46, -30), Math.toRadians(170+180))
                .strafeToSplineHeading(new Vector2d(55, -62), Math.toRadians(90+180))
                .strafeToSplineHeading(new Vector2d(56, -30), Math.toRadians(90+180));
        TrajectoryActionBuilder grab = toSample1.endTrajectory().fresh()
                //.strafeToSplineHeading(new Vector2d(-55, 47.5), Math.toRadians(90));
                .strafeToSplineHeading(new Vector2d(56, -55), Math.toRadians(90+180));
        TrajectoryActionBuilder adjust = grab.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(56, -46.5), Math.toRadians(90+180));
        TrajectoryActionBuilder hangEnd = adjust.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90+180))
                .splineToLinearHeading(new Pose2d(8,-(33.5+15-1), Math.toRadians(270+180)), Math.toRadians(-90+180));
        TrajectoryActionBuilder back2 = hangEnd.endTrajectory().fresh()
                .strafeTo(new Vector2d(8,-55));
        TrajectoryActionBuilder park = back2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(55, -60), Math.toRadians(270+180));




        //**************************TRAJECTORIES -> ACTIONS  *********************

        Action goCenterX = centerX.build();
        Action goToSample1 = toSample1.build();
        Action goback1 = back1.build();
        Action goAdjust = adjust.build();
        Action goGrab = grab.build();
        Action goback2 = back2.build();
        Action goendHang = hangEnd.build();
        Action goPark = park.build();



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
                                goToSample1,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        gripper.toggle()
                                ),
                                new SequentialAction(
                                        new SleepAction(1),
                                        armPivot.armTrig(16.1,0),
                                        slide.slideTrig(16.1,0),
                                        wristPivot.wristTrig(16.1,0, true)
                                )

                        ),

                        new SleepAction(2),
                        armPivot.armTrig(16,-4),
                        slide.slideTrig(16,-4),
                        wristPivot.wristTrigFlat(16,-4, true),
                        goGrab,
                        new ParallelAction(
                                wristPivot.wristTrigFlat(19,-5, true),
                                armPivot.armTrig(19,-5),
                                slide.slideTrig(19,-5)
                        ),
                        new SleepAction(1),
                        gripper.toggle(),
                        new SleepAction(0.5),
                        armPivot.armTrig(18,5),
                        slide.slideTrig(18,5),
                        wristPivot.wristTrig(18,5, true),
                        goAdjust,
                        new SequentialAction(
                                armPivot.armTrig(17,-12),
                                slide.slideTrig(17,-12),
                                wristPivot.wristTrigFlat(17,-12, true),
                                new SleepAction(0.2),
                                wristPivot.wristTrig90(17,-12, true),
                                new SleepAction(0.2),
                                wristPivot.wristTrigFlat(17,-12, true),

                                armPivot.armTrig(17,-3),
                                slide.slideTrig(17,-3),
                                new SleepAction(0.5),
                                wristPivot.wristTrig(17,0, true),
                                new SleepAction(0.2),
                                gripper.spin(),
                                new SleepAction(0.3),
                                gripper.toggle(),
                                gripper.toggle()
                        ),
//


                        armPivot.armTrig(20,13),
                        new ParallelAction(
                                goendHang,
                                slide.slideTrig(20,13),
                                wristPivot.wristTrig(20,13, true)
                        ),
//                        new SleepAction(0.5),
//                        armPivot.armTrig(20,7),
//                        slide.slideTrig(20,7),
//                        wristPivot.wristTrig(20,7, true),
//                        new SleepAction(0.1),
                        armPivot.armTrig(20,10),
                        slide.slideTrig(20,10),
                        wristPivot.wristTrig(20,10, true),
                        new SleepAction(0.1),
//                        gripper.toggle(),
                        new SleepAction(0.5),
                        new ParallelAction(
                                gripper.spin(),
                                goback2,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        gripper.toggle()
                                )
                        ),
                        gripper.toggle(),
                        goPark


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
        AutoWristX = 20;
        AutoWristY = 5.5;
        EndPos = new Pose2d(new Vector2d(55, -60), Math.toRadians(270+180));
        telemetry.update();

        }

    }

