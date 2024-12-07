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

    public TeleOpVisionTest() throws InterruptedException {
    }

//    public Boolean ReadyForPickup(double alpha, double width, double height)
//    {
//        if ((alpha < 45.0 & height > width) || (alpha > 45.0 & width > height))
//        {
//            return false;
//        }
//        else return true;
//    }
//
//    public double CalcWristAngleDegrees(double alpha, double width, double height)
//    {
//        if (alpha < 45.0 & height > width)
//        {
//            return (90 - alpha);
//        }
//        else if (alpha > 45.0 & width > height) {
//            return -alpha;
//        }
//        else {return 0.0;}
//    }


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {


            //*********************BLUE*************************//
            if (!bluSamps.ReadyForPickup()) {
                wristRotate.setServoPosition(bluSamps.CalcWristAngleDegrees());
            }

            //*********************RED*************************//
            if (!redSamps.ReadyForPickup()) {
                wristRotate.setServoPosition(redSamps.CalcWristAngleDegrees());

            }
        }
    }
}
