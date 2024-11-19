package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Cogintilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

//@Disabled
@TeleOp(name="PoseEstimator", group="Competition")
public class TeleOpPoseEstimation extends RobotConfiguration implements TeamConstants {

    @Override
    public void runOpMode() throws InterruptedException {

        /* Position robot and the center of the play pen, heading facing 0 degrees */
        initializeRobot(new Pose2d(0,48,-Math.PI/2));

        GamepadWrapper driver = new GamepadWrapper(gamepad1);

        waitForStart();

        while (opModeIsActive()) {

            drive.mecanumDrive(-driver.leftStick_Y *.5 , driver.leftStick_X *.5, driver.rightStick_X*.5);

            aprilTags.scanForAprilTags();
            if(aprilTags.aprilTagsDetected()) {
                for(AprilTagDetection tag : aprilTags.allAprilTagsDetected()) {
                    telemetry.addData("Tag ID: ", tag.id);
                    telemetry.addData("X: ", tag.robotPose.getPosition().x);
                    telemetry.addData("Y: ", tag.robotPose.getPosition().y);
                    telemetry.addData("Heading", tag.robotPose.getOrientation().getYaw());
                }
            }

            autoDrive.updatePoseEstimate();
            telemetry.addLine("\n_____________________________________\n");
            telemetry.addLine("Road Runner Data");
            telemetry.addData("X: ", autoDrive.pose.position.x);
            telemetry.addData("Y: ", autoDrive.pose.position.y);
            telemetry.addData("Heading: ", autoDrive.pose.heading);

            telemetry.update();
            driver.update();
        }
    }
}
