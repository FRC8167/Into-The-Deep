package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

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

    public TeleOpVisionTest() throws InterruptedException {
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

            telemetry.addData("blue width", bluSamps.blobData(ColorProcessor.Filter.NONE).size.width);
            telemetry.addData("blue height", bluSamps.blobData(ColorProcessor.Filter.NONE).size.height);
            telemetry.addData("blue angle", bluSamps.blobData(ColorProcessor.Filter.NONE).angle);

            telemetry.addData("red width", redSamps.blobData(ColorProcessor.Filter.ASPECT).size.width);
            telemetry.addData("red height", redSamps.blobData(ColorProcessor.Filter.ASPECT).size.height);
            telemetry.addData("red angle", redSamps.blobData(ColorProcessor.Filter.ASPECT).angle);
            telemetry.update();

            sleep(500);
        }
    }
}