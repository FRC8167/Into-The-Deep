package org.firstinspires.ftc.teamcode.OpModes_Autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.HeadingPath;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

//@Disabled
@Autonomous(name="AutoBlueFarSub", group="Autonomous", preselectTeleOp = "TeleOp")
public class PoseTrials extends RobotConfiguration implements TeamConstants {



    Pose2d poseFlipper(Pose2d pose) {
        return new Pose2d(
                -pose.position.x,  //reflect x-coordinate
                -pose.position.y,       //reflect y-coordinate
                pose.heading.real + Math.PI //  Add pi to heading as I could not implement .inverse()
        );
    }

    Vector2d vectorFlipper(Vector2d vector) {
       return (new Vector2d(
               -vector.x,
               -vector.y));
    }

    double headingFlipper(double heading) {
        return (heading + Math.PI);
    }

    int lineToFlipper(int ordinate) {
        return (-ordinate);
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = poseFlipper(new Pose2d(-14,60, -Math.PI/2));
        initializeRobot(initialPose);
        setAlliance(AllianceColor.RED);


        TrajectoryActionBuilder forward1 = autoDrive.actionBuilder(initialPose)
                .lineToY(36);
        TrajectoryActionBuilder block1 = autoDrive.actionBuilder(poseFlipper(new Pose2d(-14, 36, -Math.PI/2)))
                .strafeTo(new Vector2d(50,36));
        TrajectoryActionBuilder basket1 =autoDrive.actionBuilder(new Pose2d(50, 36, -Math.PI/2))
                .strafeToSplineHeading(new Vector2d(55,55),Math.toRadians(45));
        TrajectoryActionBuilder block2 = autoDrive.actionBuilder(new Pose2d(55, 55, Math.toRadians(45)))
                .strafeToSplineHeading(new Vector2d(57.5,38),Math.toRadians(-90));
        TrajectoryActionBuilder basket2 = autoDrive.actionBuilder(new Pose2d(57.5, 38, Math.toRadians(-90)))
                .strafeToSplineHeading(new Vector2d(55,55),Math.toRadians(45));
        TrajectoryActionBuilder block3 = autoDrive.actionBuilder(new Pose2d(55, 55, Math.toRadians(45)))
                .strafeToSplineHeading(new Vector2d(55,25),Math.toRadians(0));
        TrajectoryActionBuilder basket3 = autoDrive.actionBuilder(new Pose2d(55, 25, Math.toRadians(45)))
                .strafeToSplineHeading(new Vector2d(55,55),Math.toRadians(45));
        TrajectoryActionBuilder park = autoDrive.actionBuilder(new Pose2d(55, 55, Math.toRadians(45)))
                .strafeToSplineHeading(new Vector2d(30,10),Math.toRadians(180));

   

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

