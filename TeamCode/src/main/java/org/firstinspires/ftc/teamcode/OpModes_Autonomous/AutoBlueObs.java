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
@Autonomous(name="AutoBlueObs", group="Autonomous", preselectTeleOp = "TeleOp")
public class AutoBlueObs extends RobotConfiguration implements TeamConstants {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-12,63.5, -Math.PI/2);
        initializeRobot(initialPose);
        AutoWristX = 288.500/25.4;
        AutoWristY = -288.500/25.4;
        InitAuto = true;
        setAlliance(AllianceColor.BLUE);

        armPivot.resetEncoders();
        slide.resetEncoders();

       // ************************TRAJECTORIES****************************


        TrajectoryActionBuilder centerX = autoDrive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-4,33.5+15-1));

        TrajectoryActionBuilder back1 = centerX.endTrajectory().fresh()
                .strafeTo(new Vector2d(-10,55));
        TrajectoryActionBuilder toSample1 = back1.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-43, 30), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(-46, 30), Math.toRadians(170))
                .strafeToSplineHeading(new Vector2d(-55, 62), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-56, 30), Math.toRadians(90));
        TrajectoryActionBuilder grab = toSample1.endTrajectory().fresh()
                //.strafeToSplineHeading(new Vector2d(-55, 47.5), Math.toRadians(90));
                .strafeToSplineHeading(new Vector2d(-56, 55), Math.toRadians(90));
        TrajectoryActionBuilder adjust = grab.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56, 46.5), Math.toRadians(90));
        TrajectoryActionBuilder hangEnd = adjust.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-8,33.5+15-1, Math.toRadians(270)), Math.toRadians(-90));
        TrajectoryActionBuilder back2 = hangEnd.endTrajectory().fresh()
                .strafeTo(new Vector2d(-8,55));
        TrajectoryActionBuilder park = back2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-55, 60), Math.toRadians(270));




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
                        armPivot.armTrig(20,15),//15
                        new ParallelAction(
                                slide.slideTrig(20,15),
                                wristPivot.wristTrig(20,15, true),
                                goCenterX
                        ),
                        new SleepAction(0.5),
                        armPivot.armTrig(20,12),
                        slide.slideTrig(20,12),
                        wristPivot.wristTrig(20,12, true),
                        new SleepAction(0.1),
//                        gripper.toggle(),
                        new SleepAction(0.5),
                        goback1,
                        gripper.toggle(),
                        new ParallelAction(
                                goToSample1,
                                armPivot.armTrig(16.1,0),
                                slide.slideTrig(16.1,0),
                                wristPivot.wristTrig(16.1,0, true)
//                                wristRotate.rotateTrig(0),
                        ),
                        new SleepAction(2),
                        armPivot.armTrig(16,-4),
                        slide.slideTrig(16,-4),
                        wristPivot.wristTrigFlat(16,-4, true),
                        goGrab,
                        new ParallelAction(
                                wristPivot.wristTrigFlat(19,-3, true),
                                armPivot.armTrig(19,-3),
                                slide.slideTrig(19,-3)
                        ),
                        new SleepAction(1),
                        gripper.toggle(),
                        new SleepAction(1.5),
                        new ParallelAction(
                        slide.slideTrig(16,0),
                        armPivot.armTrig(16,0),
                        wristPivot.wristTrig(16,0, true)
                        ),
                        goAdjust,
                        new SequentialAction(
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


                        armPivot.armTrig(20,15),
                        new ParallelAction(
                                goendHang,
                                slide.slideTrig(20,15),
                                wristPivot.wristTrig(20,15, true)
                        ),
//                        new SleepAction(0.5),
//                        armPivot.armTrig(20,7),
//                        slide.slideTrig(20,7),
//                        wristPivot.wristTrig(20,7, true),
//                        new SleepAction(0.1),
                        new SleepAction(0.5),
                        armPivot.armTrig(20,12),
                        slide.slideTrig(20,12),
                        wristPivot.wristTrig(20,12, true),
                        goback2,
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
        EndPos = new Pose2d(new Vector2d(-55, 60), Math.toRadians(270));
        telemetry.update();

        }

    }

