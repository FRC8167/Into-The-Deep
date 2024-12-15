package org.firstinspires.ftc.teamcode.OpModes_Autonomous;

import static com.sun.tools.javac.code.Type.map;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.HeadingPath;
import com.acmerobotics.roadrunner.HeadingPosePath;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

//@Disabled
@Autonomous(name="PoseTrials", group="Autonomous", preselectTeleOp = "TeleOp")
public class PoseTrials extends RobotConfiguration implements TeamConstants {



    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {





        setAlliance(AllianceColor.RED);
        Pose2d initialPose = new Pose2d(-14,60, -Math.PI/2);

        initializeRobot(initialPose);


        TrajectoryActionBuilder forward1 = autoDrive.actionBuilder(initialPose, p-> new Pose2dDual<>(p.position.x.unaryMinus(), p.position.y.unaryMinus(), p.heading.inverse()))
                .lineToY(36);
        TrajectoryActionBuilder block1 = autoDrive.actionBuilder(new Pose2d(-14, 36, -Math.PI/2), p-> new Pose2dDual<>(p.position.x.unaryMinus(), p.position.y.unaryMinus(), p.heading.inverse()))
                .strafeTo(new Vector2d(50,36));
        TrajectoryActionBuilder basket1 =autoDrive.actionBuilder(new Pose2d(50, 36, -Math.PI/2), p-> new Pose2dDual<>(p.position.x.unaryMinus(), p.position.y.unaryMinus(), p.heading.inverse()))
                .strafeToSplineHeading(new Vector2d(55,55),Math.toRadians(45));


        TrajectoryActionBuilder block2 = autoDrive.actionBuilder(
                        new Pose2d(55, 55, Math.toRadians(45)),
                        p -> new Pose2dDual<>(p.position.x.unaryMinus(), p.position.y.unaryMinus(), p.heading.inverse()))
                        .strafeToSplineHeading(new Vector2d(57.5, 38), Math.toRadians(-90));

        TrajectoryActionBuilder basket2 = autoDrive.actionBuilder(new Pose2d(57.5, 38, Math.toRadians(-90)), p-> new Pose2dDual<>(p.position.x.unaryMinus(), p.position.y.unaryMinus(), p.heading.inverse()))
                .strafeToSplineHeading(new Vector2d(55,55),Math.toRadians(45));

        TrajectoryActionBuilder block3 = autoDrive.actionBuilder(new Pose2d(55, 55, Math.toRadians(45)), p-> new Pose2dDual<>(p.position.x.unaryMinus(), p.position.y.unaryMinus(), p.heading.inverse()))
                .strafeToSplineHeading(new Vector2d(55,25),Math.toRadians(0));

        TrajectoryActionBuilder basket3 = autoDrive.actionBuilder(new Pose2d(55, 25, Math.toRadians(45)), p-> new Pose2dDual<>(p.position.x.unaryMinus(), p.position.y.unaryMinus(), p.heading.inverse()))
                .strafeToSplineHeading(new Vector2d(55, 55), Math.toRadians(45));
        TrajectoryActionBuilder park = autoDrive.actionBuilder(new Pose2d(55, 55, Math.toRadians(45)), p -> new Pose2dDual<>(p.position.x.unaryMinus(), p.position.y.unaryMinus(), p.heading.inverse()))
                .strafeToSplineHeading(new Vector2d(30, 10), Math.toRadians(180));

   

        waitForStart();

        if (isStopRequested()) return;

        Action goForward1 = forward1.build();

        Action goBlock1 = block1.build();
        Action goBasket1 = basket1.build();
        Action goBlock2 = block2.build();
        Action goBasket2 = basket2.build();

        Action goBlock3 = block3.build();
        Action goBasket3 = basket3.build();
        Action goPark = park.build();
        Actions.runBlocking(
                new SequentialAction(
                        goForward1,
                        goBlock1,
                        goBasket1,
                        goBlock2,
                        goBasket2,
                        goBlock3,
                        goBasket3,
                        goPark
                )
        );



//        Actions.runBlocking(wristPivot.setServoPosition(0.2));

        telemetry.update();

        }

    }

