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
@Autonomous(name="AutoRedObs", group="Autonomous", preselectTeleOp = "TeleOpMain")
public class AutoRedObs extends RobotConfiguration implements TeamConstants {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(12,-63.5, Math.PI/2);
        initializeRobot(initialPose);
        AutoWristX = 288.500/25.4;
        AutoWristY = -288.500/25.4;
        InitAuto = true;
        InitTele = false;
        GoodPose = false;
        HeadingAprox = Math.toRadians(90);
        setAlliance(AllianceColor.RED);

        armPivot.resetEncoders();
        slide.resetEncoders();

       // ************************TRAJECTORIES****************************


        TrajectoryActionBuilder centerX = autoDrive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-0,-(33.5+15-1)));

//        TrajectoryActionBuilder back1 = centerX.endTrajectory().fresh()
//                .strafeTo(new Vector2d(-10,55));
        TrajectoryActionBuilder toSample1 = centerX.endTrajectory().fresh()
                .setTangent(Math.toRadians(90+180))
                .splineToSplineHeading(new Pose2d(43, -30, Math.toRadians(180+180)), Math.toRadians(-135+180))
                .setTangent(Math.toRadians(120+180))
                .splineToLinearHeading(new Pose2d(56, -61, Math.toRadians(90+180)), Math.toRadians(90+180));
//                .strafeToSplineHeading(new Vector2d(-46, 30), Math.toRadians(170))
//                .strafeToSplineHeading(new Vector2d(-56, 62), Math.toRadians(90));
//                .setTangent(Math.toRadians(-45))
//                .splineToLinearHeading(new Pose2d(-56, 12, Math.toRadians(90)), Math.toRadians(135));
        TrajectoryActionBuilder grab = toSample1.endTrajectory().fresh()
                //.strafeToSplineHeading(new Vector2d(-55, 47.5), Math.toRadians(90));
                .strafeToSplineHeading(new Vector2d(56, -54.8), Math.toRadians(90+180));
        TrajectoryActionBuilder adjust = grab.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(56, -46.5), Math.toRadians(90+180));
        TrajectoryActionBuilder hangEnd = adjust.endTrajectory().fresh()
                .setTangent(Math.toRadians(120+180))
                .splineToLinearHeading(new Pose2d(4,-(33.5+15-1), Math.toRadians(270+180)), Math.toRadians(-90+180));

        TrajectoryActionBuilder grab2 = hangEnd.endTrajectory().fresh()
//                .strafeToSplineHeading(new Vector2d(-55, 60), Math.toRadians(270));
                .setTangent(120+180)
                .splineToLinearHeading(new Pose2d(56, -54.8, Math.toRadians(90+180)), Math.toRadians(90+180));
        TrajectoryActionBuilder adjust2 = grab2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(56, -46.5), Math.toRadians(90+180));
        TrajectoryActionBuilder hangEnd2 = adjust2.endTrajectory().fresh()
                .setTangent(Math.toRadians(120+180))
                .splineToLinearHeading(new Pose2d(8,-(33.5+15-1), Math.toRadians(270+180)), Math.toRadians(-90+180));
        TrajectoryActionBuilder park = hangEnd2.endTrajectory().fresh()
                .setTangent(Math.toRadians(120+180))
                .splineToLinearHeading(new Pose2d(55, -60, Math.toRadians(-90+180)), Math.toRadians(160+180));




        //**************************TRAJECTORIES -> ACTIONS  *********************

        Action goCenterX = update(centerX.build());
        Action goToSample1 = update(toSample1.build());
//        Action goback1 = back1.build();
        Action goAdjust = update(adjust.build());
        Action goGrab = update(grab.build());
        Action goendHang = update(hangEnd.build());
        Action goGrab2 = update(grab2.build());
        Action goAdjust2 = update(adjust2.build());
        Action goEndHang2 = update(hangEnd2.build());
        Action goPark = update(park.build());



        waitForStart();

        if (isStopRequested()) return;




        //************************** RUN THE ACTIONS  ****************************
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                armPivot.armTrig(20,14),//15
                                new SequentialAction(
                                        new SleepAction(1),
                                        new ParallelAction(
                                                slide.slideTrig(20,14),
                                                wristPivot.wristTrig(20,14, true),
                                                goCenterX
                                        )
                                )
                           ),
//                        new SleepAction(0.5),
                        armPivot.armTrig(20,10),
                        slide.slideTrig(20,10),
                        wristPivot.wristTrig(20,10, true),

                        new ParallelAction(
                                goToSample1,
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        gripper.spin(),
                                        new SleepAction(0.3),
                                        gripper.toggle()
                                ),
                                new SequentialAction(
                                        new SleepAction(1),
                                        armPivot.armTrig(16.1,0),
                                        slide.slideTrig(16.1,0),
                                        wristPivot.wristTrig(16.1,0, true)
                                )

                        ),

//                        new SleepAction(2),
                        armPivot.armTrig(16,-4),
                        slide.slideTrig(16,-4),
                        wristPivot.wristTrigFlat(16,-4, true),
                        goGrab,
                        new ParallelAction(
                                wristPivot.wristTrigFlat(19,-4.5, true),
                                armPivot.armTrig(19,-4.5),
                                slide.slideTrig(19,-4.5),
                                new SequentialAction(
                                        new SleepAction(0.25),
                                        gripper.toggle()
                                )
                        ),

                        new SleepAction(0.25),

//                        wristPivot.wristTrig(18,5, true),
                        new ParallelAction(
                                new SequentialAction(
                                        new ParallelAction(
                                armPivot.armTrig(18,5),
                                slide.slideTrig(18,5)
                                        ),
                                        new ParallelAction(
                                                armPivot.armTrig(17,-3),
                                                slide.slideTrig(17,-3),
                                                wristPivot.wristTrig(17,0, true),
                                                new SleepAction(0.4),
                                                        gripper.spin()
                                        )
                                        ),
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        goAdjust
                                )

                        ),



//



                        new ParallelAction(
                                goendHang,
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        wristPivot.wristTrig(15,30, true),
                                        new SleepAction(0.1),
                                        wristPivot.wristTrig(17,0, true),
                                        new SleepAction(0.1),
                                        wristPivot.wristTrig(15,30, true),
                                        new SleepAction(0.1),
                                        wristPivot.wristTrig(17,0, true),

                                        new SleepAction(1),//0.3
                                        gripper.toggle(),
                                        gripper.toggle(),

                                        armPivot.armTrig(20,14),
                                        new ParallelAction(
                                                slide.slideTrig(20,14),
                                                wristPivot.wristTrig(20,14, true)

                                        )

                                )
                        ),
//                        new SleepAction(0.5),
//                        armPivot.armTrig(20,7),
//                        slide.slideTrig(20,7),
//                        wristPivot.wristTrig(20,7, true),
//                        new SleepAction(0.1),
                        armPivot.armTrig(20,10),
                        slide.slideTrig(20,10),
                        wristPivot.wristTrig(20,10, true),
//                        gripper.toggle(),

                        new ParallelAction(
                                goGrab2,
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        gripper.spin(),
                                        new SleepAction(0.3),
                                        gripper.toggle()
                                ),
                                new SequentialAction(
                                    new SleepAction(1),
                                    new ParallelAction(
                                            armPivot.armTrig(16,-4.5),
                                            slide.slideTrig(16,-4.5),
                                            wristPivot.wristTrig(16,-4.5, true)
                                    )
                                ),
                                new SequentialAction(
                                        new SleepAction(2),
                                        wristPivot.wristTrigFlat(16,-4, true)
                                )
                        ),
        new ParallelAction(
                wristPivot.wristTrigFlat(19,-4, true),
                armPivot.armTrig(19,-4),
                slide.slideTrig(19,-4),
                new SequentialAction(
                        new SleepAction(0.25),
                        gripper.toggle()
                )
        ),

                new SleepAction(0.25),

//                        wristPivot.wristTrig(18,5, true),
                        new ParallelAction(
                                new SequentialAction(
                                        new ParallelAction(
                                                armPivot.armTrig(18,5),
                                                slide.slideTrig(18,5)
                                        ),
                                        new ParallelAction(
                                                armPivot.armTrig(17,-3),
                                                slide.slideTrig(17,-3),
                                                wristPivot.wristTrig(17,0, true),
                                                new SleepAction(0.4),
                                                gripper.spin()
                                        )
                                ),
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        goAdjust2
                                )

                        ),



//



                new ParallelAction(
                        goEndHang2,
                        new SequentialAction(
                                new SleepAction(0.1),
                                wristPivot.wristTrig(15,30, true),
                                new SleepAction(0.1),
                                wristPivot.wristTrig(17,0, true),
                                new SleepAction(0.1),
                                wristPivot.wristTrig(15,30, true),
                                new SleepAction(0.1),
                                wristPivot.wristTrig(17,0, true),

                                new SleepAction(1),//0.3
                                gripper.toggle(),
                                gripper.toggle(),

                                armPivot.armTrig(20,14),
                                new ParallelAction(
                                        slide.slideTrig(20,14),
                                        wristPivot.wristTrig(20,14, true)

                                )


                        )
                ),
                        armPivot.armTrig(20,10),
                        slide.slideTrig(20,10),
                        wristPivot.wristTrig(20,10, true),
//                        gripper.toggle(),
                        new SleepAction(0.5),

                        new ParallelAction(
                                goPark,
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        gripper.spin(),
                                        new SleepAction(0.3),
                                        gripper.toggle()
                                ),
                                new SequentialAction(
                                        new SleepAction(1),
                                        new ParallelAction(
                                        armPivot.armTrig(17,0),
                                        slide.slideTrig(17,0),
                                        wristPivot.wristTrig(17,0, true)
                                        )
                                )

                        )


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
//        updateEnd();
        EndPos = new Pose2d(new Vector2d(55, -60), Math.toRadians(270+180));
        GoodPose = true;
        telemetry.update();

        }

    }

