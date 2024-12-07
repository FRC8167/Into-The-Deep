package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Cogintilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

//@Disabled
@TeleOp(name="PoseEstimator", group="Competition")
public class TeleOpPoseEstimation extends RobotConfiguration implements TeamConstants {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        /* Position robot and the center of the play pen, heading facing 0 degrees */
//        initializeRobot(new Pose2d(0,48,-Math.PI/2));
        initializeRobot(new Pose2d(0,0,0));

        GamepadWrapper driver = new GamepadWrapper(gamepad1);
        gamepad1.setLedColor(0, 1, 0, 500);


//
        waitForStart();

        while (opModeIsActive()) {
            //*********************BLUE*************************//
            if (bluSamps.ReadyForPickup() & gamepad1.right_bumper) {
//                wristRotate.setServoPosition(bluSamps.CalcWristAngleDegrees());
                gamepad1.setLedColor(0, 0, 1, 500);
            }

            //*********************RED*************************//
           else if (redSamps.ReadyForPickup() & gamepad1.left_bumper) {
//                wristRotate.setServoPosition(redSamps.CalcWristAngleDegrees());
                gamepad1.setLedColor(1, 0, 0, 500);
            }
//            else if (yelSamps.ReadyForPickup()) {
////                wristRotate.setServoPosition(redSamps.CalcWristAngleDegrees());
//                gamepad1.setLedColor(1, 0, 0, 500);
//            }

            else {gamepad1.setLedColor(0, 1, 0, 500);}

            drive.mecanumDrive(-driver.leftStick_Y *.5 , driver.leftStick_X *.5, driver.rightStick_X*.5);


            aprilTags.scanForAprilTags();
            if(aprilTags.aprilTagsDetected()) {
                for(AprilTagDetection tag : aprilTags.allAprilTagsDetected()) {
                    telemetry.addData("Tag ID: ", tag.id);
                    telemetry.addData("X: ", tag.robotPose.getPosition().x);
                    telemetry.addData("Y: ", tag.robotPose.getPosition().y);
                    telemetry.addData("Heading", tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));

                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            tag.ftcPose.x,
                            tag.ftcPose.y,
                            tag.ftcPose.bearing));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            tag.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            tag.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
                }

            }



            autoDrive.updatePoseEstimate();
            telemetry.addLine("\n_____________________________________\n");
            telemetry.addLine("Road Runner Data");
            telemetry.addData("X: ", autoDrive.pose.position.x);
            telemetry.addData("Y: ", autoDrive.pose.position.y);
            telemetry.addData("Heading: ", Math.toDegrees(autoDrive.pose.heading.toDouble()));

            telemetry.update();
            driver.update();
        }
    }
}
