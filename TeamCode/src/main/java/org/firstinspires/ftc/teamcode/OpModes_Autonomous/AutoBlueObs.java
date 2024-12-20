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
@Autonomous(name="AutoBlueObs", group="Autonomous", preselectTeleOp = "TeleOp")
public class AutoBlueObs extends RobotConfiguration implements TeamConstants {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-12,63.5, -Math.PI/2);
        initializeRobot(initialPose);
        setAlliance(AllianceColor.BLUE);

        armPivot.resetEncoders();
        slide.resetEncoders();

       // ************************TRAJECTORIES****************************
        TrajectoryActionBuilder waitPlay = autoDrive.actionBuilder(initialPose)
                .waitSeconds(2);
        TrajectoryActionBuilder wait1 = autoDrive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        TrajectoryActionBuilder wait2 = autoDrive.actionBuilder(initialPose)
                .waitSeconds(1);


        TrajectoryActionBuilder centerX = autoDrive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0,33.5));

        TrajectoryActionBuilder back1 = centerX.endTrajectory().fresh()
                .strafeTo(new Vector2d(-20,55));
        TrajectoryActionBuilder toSample1 = back1.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-43, 30), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(-46, 30), Math.toRadians(170))
//                .strafeToSplineHeading(new Vector2d(-45, 62), Math.toRadians(90))
//                .strafeToSplineHeading(new Vector2d(-46, 30.5), Math.toRadians(180))
//                .strafeToSplineHeading(new Vector2d(-56, 30.5), Math.toRadians(170))
                .strafeToSplineHeading(new Vector2d(-55, 62), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-55, 30), Math.toRadians(90));
        TrajectoryActionBuilder grab = toSample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-55, 47.5), Math.toRadians(90));
        TrajectoryActionBuilder hangEnd = grab.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(0,50), Math.toRadians(270))
                .strafeTo(new Vector2d(0,33.5));
        TrajectoryActionBuilder back2 = hangEnd.endTrajectory().fresh()
                .strafeTo(new Vector2d(0,55));
        TrajectoryActionBuilder park = back2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-55, 60), Math.toRadians(270));




        //**************************TRAJECTORIES -> ACTIONS  *********************

        Action goCenterX = centerX.build();
        Action goToSample1 = toSample1.build();
        Action goback1 = back1.build();
        Action goWaitPlayer = waitPlay.build();
        Action goWait1 = wait1.build();
        Action goWait2 = wait2.build();
        Action goGrab = grab.build();
        Action goback2 = back2.build();
        Action goendHang = hangEnd.build();
        Action goPark = park.build();



        waitForStart();

        if (isStopRequested()) return;




        //************************** RUN THE ACTIONS  ****************************
        Actions.runBlocking(
                new SequentialAction(
                        armPivot.armTrig(20,3.2),
                        slide.slideTrig(20,3.2),
                        wristPivot.wristTrig(20,3.2, true),
                        goCenterX,
                        armPivot.armTrig(20,5.5),
                        slide.slideTrig(20,5.5),
                        wristPivot.wristTrig(20,5.5, true),
                        goback1,
                        new ParallelAction(
                                goToSample1,
                                armPivot.armTrig(16.1,0),
                                slide.slideTrig(16.1,0),
                                wristPivot.wristTrig(16.1,0, true),
                                gripper.toggle()
                        ),
                        goWaitPlayer,
                        goGrab,
                        armPivot.armTrig(20,-5.5),
                        slide.slideTrig(20,-5.5),
                        wristPivot.wristTrig(20,-5.5, true),
                        goWait1,
                        gripper.toggle(),
                        goWait2,
                        armPivot.armTrig(20,3.2),
                        slide.slideTrig(20,3.2),
                        wristPivot.wristTrig(20,3.2, true),
                        goendHang,
                        armPivot.armTrig(20,5.5),
                        slide.slideTrig(20,5.5),
                        wristPivot.wristTrig(20,5.5, true),
                        goback2,
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

        telemetry.update();

        }

    }

