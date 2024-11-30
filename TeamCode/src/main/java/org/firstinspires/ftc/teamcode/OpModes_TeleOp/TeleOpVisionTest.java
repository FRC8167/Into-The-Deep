package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import static java.util.Collections.swap;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
import org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors.AprilTagProcessorObject;
import org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors.ColorProcessor;
import org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors.VisionPortalObject;
import org.firstinspires.ftc.vision.opencv.ColorRange;

//@Disabled
@TeleOp(name="VisionTest", group="Experimental")
public class TeleOpVisionTest extends RobotConfiguration implements TeamConstants {

        ColorProcessor bluSamps = new ColorProcessor(ColorRange.BLUE);
        ColorProcessor redSamps = new ColorProcessor(ColorRange.RED);
        AprilTagProcessorObject aprilTag = new AprilTagProcessorObject();
        public Boolean blueReadyForPickup = false;
        public Boolean redReadyForPickup = false;


    public TeleOpVisionTest() throws InterruptedException {
    }

    public Boolean ReadyForPickup(double alpha, double width, double height)
    {
        if ((alpha < 45.0 & height > width) || (alpha > 45.0 & width > height))
        {
            return false;
        }
        else return true;
    }

    public double CalcWristAngleDegrees(double alpha, double width, double height)
    {
        if (alpha < 45.0 & height > width)
        {
            return (90 - alpha);
        }
        else if (alpha > 45.0 & width > height) {
            return -alpha;
        }
        else {return 0.0;}
    }


    @Override
    public void runOpMode() throws InterruptedException {

        VisionPortalObject vision = new VisionPortalObject.Builder(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(bluSamps.colorProcessor())
                .addProcessor(redSamps.colorProcessor())
                .addProcessor(aprilTag.getProcessor())
                .build();

        waitForStart();

        while (opModeIsActive()) {


            //*********************BLUE*************************//
            double blueFoundAngle = bluSamps.blobData(ColorProcessor.Filter.NONE).angle;
            double adjBlueAngle = 0;
            double blueFoundWidth = bluSamps.blobData(ColorProcessor.Filter.NONE).size.width;
            double blueFoundHeight = bluSamps.blobData(ColorProcessor.Filter.NONE).size.height;
            adjBlueAngle = 90 - blueFoundAngle;
            blueReadyForPickup = ReadyForPickup(adjBlueAngle, blueFoundWidth, blueFoundHeight);
            if (!blueReadyForPickup) {
                double wristRotDegrees = CalcWristAngleDegrees(adjBlueAngle, blueFoundWidth, blueFoundHeight);
                telemetry.addData("Wrist needs to rotate", wristRotDegrees);
            }
            else {telemetry.addLine("BLUE is ready for pickup");}
            telemetry.addData("blue width", blueFoundWidth);
            telemetry.addData("blue height", blueFoundHeight);
            telemetry.addData("blue angle", blueFoundAngle);
            telemetry.addData("new angle", adjBlueAngle);

            //*********************RED*************************//
            double redFoundAngle = redSamps.blobData(ColorProcessor.Filter.NONE).angle;
            double adjRedAngle = 0;
            double redFoundWidth = redSamps.blobData(ColorProcessor.Filter.NONE).size.width;
            double redFoundHeight = redSamps.blobData(ColorProcessor.Filter.NONE).size.height;
            adjRedAngle = 90 - redFoundAngle;
            redReadyForPickup = ReadyForPickup(adjRedAngle, redFoundWidth, redFoundHeight);
            if (!redReadyForPickup) {
                double wristRotDegrees = CalcWristAngleDegrees(adjRedAngle, redFoundWidth, redFoundHeight);
                telemetry.addData("Wrist needs to rotate", wristRotDegrees);
            }
            else {telemetry.addLine("RED is ready for pickup");}
            telemetry.addData("red width", redFoundWidth);
            telemetry.addData("red height", redFoundHeight);
            telemetry.addData("red angle", redSamps.blobData(ColorProcessor.Filter.NONE).angle);
            telemetry.addData("new angle", adjRedAngle);
            telemetry.update();

            sleep(5000);
        }
    }
}
