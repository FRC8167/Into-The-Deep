package org.firstinspires.ftc.teamcode.OpModes_Autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
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
        InitTele = false;
        GoodPose = false;
        HeadingAprox = Math.toRadians(-180);
        setAlliance(AllianceColor.BLUE);

        armPivot.resetEncoders();
        slide.resetEncoders();

       // ************************TRAJECTORIES****************************
        TrajectoryActionBuilder centerX = autoDrive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(8,33.5+15-1));

        TrajectoryActionBuilder sample1 = centerX.endTrajectory().fresh()
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(48,46.5, Math.toRadians(-90)), Math.toRadians(-100));

        TrajectoryActionBuilder drop1 = sample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(58, 61), Math.toRadians(45));
        TrajectoryActionBuilder sample2 = drop1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(58, 46.5), Math.toRadians(270));
        TrajectoryActionBuilder drop2 = sample2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(58, 61), Math.toRadians(45));
        TrajectoryActionBuilder sample3 = drop2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(50, 28, Math.toRadians(0)), Math.toRadians(0));
        TrajectoryActionBuilder drop3 = sample3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(58, 61, Math.toRadians(45)), Math.toRadians(45));
        TrajectoryActionBuilder touch = drop3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(24, 12, Math.toRadians(180)), Math.toRadians(180));



        //**************************TRAJECTORIES -> ACTIONS  *********************



        Action goCenterX = update(centerX.build());
        Action goSample1 = update(sample1.build());
        Action goDrop1 = update(drop1.build());
        Action goSample2 = update(sample2.build());
        Action goDrop2 = update(drop2.build());
        Action goSample3 = update(sample3.build());
        Action goDrop3 = update(drop3.build());
        Action goTouch = update(touch.build());






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
                                ),
                                new SequentialAction(
                                        new SleepAction(1),
                                        new ParallelAction(
                                                gripper.eOpen(),
                                                slide.slideToPosition(0),
                                                new SleepAction(0.5),
                                                armPivot.armTrig(28,0)
                                        )
                                )

                        ),
//                        new SleepAction(0.25),
                        slide.slideTrig(28,0),
                        new ParallelAction(
                                slide.slideTrig(24,-7.2),
                                armPivot.armTrig(24,-7.2),
                                wristPivot.wristTrig(24,-7.2, true)
                        ),
                        new SleepAction(0.25), // 1
                        gripper.toggle(),
                        new SleepAction(0.25),
                        new ParallelAction(
                                armPivot.armTrig(14,33),
                                new SequentialAction(
                                        new SleepAction(1.5),
                                        new ParallelAction(
                                                slide.slideTrig(14,33),
                                                wristPivot.wristTrig(14,33, true),
                                                goDrop1
                                        )
                                )
                        ),
                        new SleepAction(0.25),
                        gripper.toggle(),
                        new SleepAction(0.25),
                        new ParallelAction(
                                goSample2,
                                new SequentialAction(
                                        new SleepAction(0.25),
                                        new ParallelAction(
                                                gripper.eOpen(),
                                                slide.slideToPosition(0),
                                                new SleepAction(1)

                                        ),
                                        armPivot.armTrig(28,0)
                                )
                        ),
                        new SleepAction(0.125),
                        slide.slideTrig(28,0),
                        new ParallelAction(
                                slide.slideTrig(24,-7.2),
                                armPivot.armTrig(24,-7.2),
                                wristPivot.wristTrig(24,-7.2, true)
                        ),
                        new SleepAction(0.25), // 1
                        gripper.toggle(),
                        new SleepAction(0.25),
                        new ParallelAction(
                                armPivot.armTrig(14,33),
                                new SequentialAction(
                                        new SleepAction(1.5),
                                        new ParallelAction(
                                                slide.slideTrig(14,33),
                                                wristPivot.wristTrig(14,33, true),
                                                goDrop2
                                        )
                                )
                        ),
                        new SleepAction(0.25),//
                        gripper.toggle(),
                        new SleepAction(0.25),

                        new ParallelAction(
                                goSample3,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        new ParallelAction(
                                                gripper.eOpen(),
                                                slide.slideToPosition(0),
                                                armPivot.armTrig(28,0)
                                        )
                                )
                        ),
                        new SleepAction(0.25),
                        slide.slideTrig(28,0),
                        wristRotate.rotateTrig(0),
//                        new SleepAction(0.25),
                                slide.slideTrig(24,-6.8),
                        new SleepAction(0.25),
                                armPivot.armTrig(24,-6.8),
                                wristPivot.wristTrig(24,-6.8, true),
                        new SleepAction(0.25), // 1
                        gripper.toggle(),
                        new SleepAction(0.25),
                        slide.slideToPosition(0),
                        armPivot.armTrig(14,33),
                        new SleepAction(0.75),
                        new ParallelAction(
                                slide.slideTrig(14,33),
                                wristPivot.wristTrig(14,33, true),
                                wristRotate.rotateTrig(90),
                                goDrop3
                        ),
//                        new SleepAction(0.25),
                        gripper.toggle(),
                        new SleepAction(0.25),
                        new ParallelAction(
                                goTouch,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        new ParallelAction(
                                                armPivot.armTrig(17,0),
                                                slide.slideTrig(17,0),
                                                wristPivot.wristTrig(17,0, true)
                                                //                                goTouch
                                        ),
                                        new SleepAction(1),
                                        new ParallelAction(
                                                armPivot.armTrig(24,0),
                                                slide.slideTrig(24,0),
                                                wristPivot.wristTrig(24,0, true)
                                                //                                goTouch
                                        ),
                                        new SleepAction(0.3),
                                        new ParallelAction(
                                                armPivot.armTrig(26,1),
                                                slide.slideTrig(26,1)
                                                //                                goTouch
                                        )


                                )
                        )
//                        armPivot.armTrig(16,6.5),
//                        slide.slideTrig(16,6.5),
//                        wristPivot.wristTrig(16,6.5, true)
                )
        );



//        Actions.runBlocking(wristPivot.setServoPosition(0.2));
        AutoWristX = 17;
        AutoWristY = -1;
//        EndPos = new Pose2d(new Vector2d(24, 12), Math.toRadians(180));
//        EndPos = autoDrive.pose;
        telemetry.update();

        }

    }

