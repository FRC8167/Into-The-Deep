package org.firstinspires.ftc.teamcode.OpModes_Autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
        TrajectoryActionBuilder waitPlay = autoDrive.actionBuilder(initialPose)
                .waitSeconds(2);
        TrajectoryActionBuilder wait1 = autoDrive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        TrajectoryActionBuilder wait2 = autoDrive.actionBuilder(initialPose)
                .waitSeconds(1);
        TrajectoryActionBuilder wait3 = autoDrive.actionBuilder(initialPose)
                .waitSeconds(1.5);


        TrajectoryActionBuilder centerX = autoDrive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0,-33.5+15));

        TrajectoryActionBuilder sample1 = centerX.endTrajectory().fresh()
                .setTangent(Math.toRadians(80+180))
                .splineToLinearHeading(new Pose2d(-48,-44.5, Math.toRadians(-90+180)), Math.toRadians(-20+180));
        TrajectoryActionBuilder drop1 = sample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58, -61), Math.toRadians(45+180));
        TrajectoryActionBuilder sample2 = drop1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58, -44.5), Math.toRadians(270+180));
        TrajectoryActionBuilder drop2 = sample2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58, -61), Math.toRadians(45+180));
        TrajectoryActionBuilder sample3Back = drop2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-38, -29), Math.toRadians(180));
        TrajectoryActionBuilder sample3 = sample3Back.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-50, -29), Math.toRadians(180));
        TrajectoryActionBuilder drop3 = sample3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58, -61), Math.toRadians(45+180));
        TrajectoryActionBuilder prepTouch = drop3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-48, -12), Math.toRadians(180+180));
        TrajectoryActionBuilder touch = prepTouch.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-24, -12), Math.toRadians(180+180));



        //**************************TRAJECTORIES -> ACTIONS  *********************

        Action goCenterX = centerX.build();
        Action goWaitPlayer = waitPlay.build();
        Action goWait1 = wait1.build();
        Action goWait2 = wait2.build();
        Action goWait3 = wait3.build();
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
                        armPivot.armTrig(20,3.2),
//                        slide.slideTrig(20,3.2),
//                        wristPivot.wristTrig(20,3.2, true),
                        armPivot.armTrig(20,13),
                        slide.slideTrig(20,13),
                        wristPivot.wristTrig(20,13, true),
                        goCenterX,
                        goWait1,
                        armPivot.armTrig(20,10),
                        slide.slideTrig(20,10),
                        wristPivot.wristTrig(20,10, true),
                        goWait2,
                        gripper.toggle(),
                        goWait1,
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
                        goWait3,
                        gripper.toggle(),
                        goWait2,
                        armPivot.armTrig(9.4,33),
                        slide.slideTrig(9.4,33),
                        wristPivot.wristTrig(9.4,33, true),
                        goDrop1,
                        gripper.toggle(),
                        goWait2,
                        goSample2,
                        slide.slideTrig(24,-6.5),
                        armPivot.armTrig(24,-6.5),
                        wristPivot.wristTrig(24,-6.5, true),
                        goWait3,
                        gripper.toggle(),
                        goWait1,
                        armPivot.armTrig(9.4,33),
                        slide.slideTrig(9.4,33),
                        wristPivot.wristTrig(9.4,33, true),
                        goDrop2,
                        goWait3,
                        gripper.toggle(),
                        goWait2,
                        goPrepTouch,
                        armPivot.armTrig(16,11.4),
                        slide.slideTrig(16,11.4),
                        wristPivot.wristTrig(16,11.4, true),
                        goTouch,
                        armPivot.armTrig(16,7.4),
                        slide.slideTrig(16,7.4),
                        wristPivot.wristTrig(16,7.4, true)
//                        goSample3Back,
//                        wristRotate.rotateTrig(0),
//                        slide.slideTrig(24,-4.2),
//                        armPivot.armTrig(24,-4.2),
//                        wristPivot.wristTrig(24,-4.2, true),
//                        goWait2,
//                        goSample3,
//                        wristRotate.rotateTrig(0),
//                        slide.slideTrig(24,-65),
//                        armPivot.armTrig(24,-6.5),
//                        wristPivot.wristTrig(24,-6.5, true),
//                        goWait2,
//                        gripper.toggle(),
//                        goWait2,
//                        goWait2,
//                        armPivot.armTrig(9.4,33),
//                        wristRotate.rotateTrig(90),
//                        slide.slideTrig(9.4,33),
//                        wristPivot.wristTrig(9.4,33, true),
//                        goDrop3,
//                        goWait2,
//                        gripper.toggle(),
//                        goWaitPlayer,
//                        goWaitPlayer

//                        goendHang,
//                        armPivot.armTrig(20,5.5),
//                        slide.slideTrig(20,5.5),
//                        wristPivot.wristTrig(20,5.5, true)
//                        goback2,
//                        goPark


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
        AutoWristX = 16.0;
        AutoWristY = 7.4;
        telemetry.update();

        }

    }

