package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Cogintilities.Func;
import org.firstinspires.ftc.teamcode.SubSytems.MotorPivotExp;
import org.firstinspires.ftc.teamcode.SubSytems.ServoPivot;
import org.firstinspires.ftc.teamcode.SubSytems.ServoRotate;
import org.firstinspires.ftc.teamcode.SubSytems.ServoToggle;
import org.firstinspires.ftc.teamcode.SubSytems.Slide;
import org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors.AprilTagProcessorObject;
import org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors.ColorProcessor;
import org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors.VisionPortalObject;
import org.firstinspires.ftc.teamcode.SubSytems.MecanumDriveBasic;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;
import java.util.Locale;

/**
 * This class should be used to define all the subsystem modules and assign the hardware used in
 * those modules. The keyword 'abstract' indicates that an object of this class cannot be created
 * directly in an opMode. Instead, a class must be created that extends or inherits from this class.
 * In our case, all OpModes will extend RobotConfig. This allows the opMode to use all the
 * variables, objects and methods defined below. It also will create an OpMode that uses the SDK's
 * LinearOpMode framework as this class itself extends the LinearOpMode class.
 */

public abstract class RobotConfiguration extends LinearOpMode {

    /*------------ Public Class Variables - Frowned Upon ------------*/
    public enum AllianceColor { RED, BLUE }
    public int[] myPortalIDs;
    public int aTPortalID;
    public int colorPortalID;

    public static double AutoWristX;
    public static double AutoWristY;
    public static boolean InitAuto = false;
    public static boolean InitTele = false;
    public static boolean GoodPose = false;
    public static Pose2d EndPos = null;
    public static double HeadingAprox;

    public void updateEnd(){
        autoDrive.updatePoseEstimate();
        EndPos = autoDrive.pose;
    }
    public class UpdatePose implements Action {  //Note: slide does not extend



        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            updateEnd();
            return false;
        }
    }
    public Action update(Action traj)
    {
        return new SequentialAction(traj, new UpdatePose());
    };

    /*------------- Private Class Variables - Preferred -------------*/
    static AllianceColor alliance;
    static List<LynxModule> ctrlHubs;


    /*----------- Define all Module Classes (SubSystems) ------------*/
    static protected MecanumDriveBasic  drive;
    static protected MecanumDrive       autoDrive;
    protected VisionPortalObject        atVision;
    protected VisionPortalObject        colorVision;
    static protected ServoRotate        wristRotate;
    static protected ServoPivot         wristPivot;
    static protected ServoToggle        gripper;
    static protected MotorPivotExp      armPivot;
    static protected Slide              slide;
    static protected Func               Functions;


    /*---------------------- Vision Objects -------------------------*/
    protected ColorProcessor bluSamps = new ColorProcessor(ColorRange.BLUE);
    protected ColorProcessor redSamps = new ColorProcessor(ColorRange.RED);
    protected ColorProcessor yelSamps = new ColorProcessor(ColorRange.YELLOW);
    protected AprilTagProcessorObject aprilTags = new AprilTagProcessorObject();


    /**
     * initializeRobot:
     * Initialize robot with a specified start pose (used by Road Runner. This function should be
     * called immediately in the OpMode's runOpMode function. A null value error will result if you
     * try to use any devices connected to the control hub that have not been initialized.  This
     * function creates the Hardware Map and the module objects that use these devices.
     *
     * @throws InterruptedException
     */
    public void initializeRobot(Pose2d startPose) throws InterruptedException {

        /* Find all Control Hubs and Set Sensor Bulk Read Mode to AUTO */
        ctrlHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : ctrlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /* ******************* Define Hardware Map Here ******************** */
        DcMotorEx driveMotorLF = hardwareMap.get(DcMotorEx.class, "par");
        DcMotorEx driveMotorRF = hardwareMap.get(DcMotorEx.class, "perp");
        DcMotorEx driveMotorLR = hardwareMap.get(DcMotorEx.class, "motor1");
        DcMotorEx driveMotorRR = hardwareMap.get(DcMotorEx.class, "motor2");

        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "arm");  //EH3
        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");  //EH2

        Servo wristPivotServo = hardwareMap.get(Servo.class, "servo1");
        Servo wristRotateServo = hardwareMap.get(Servo.class, "servo2");
        Servo gripperServo = hardwareMap.get(Servo.class, "servo0");

        WebcamName webCam1      = hardwareMap.get(WebcamName.class, "Webcam1");
        WebcamName webCam2      = hardwareMap.get(WebcamName.class, "Webcam2");

        /* Create an object of every module/subsystem needed for both autonomous and teleOp modes. */
        drive       = new MecanumDriveBasic(driveMotorLF, driveMotorLR, driveMotorRF, driveMotorRR);
        autoDrive   = new MecanumDrive(hardwareMap, startPose);
        wristRotate = new ServoRotate(wristRotateServo, TeamConstants.WRIST_ROTATE_CENTER, TeamConstants.WRIST_ROTATE_MIN, TeamConstants.WRIST_ROTATE_MAX);
        wristPivot  = new ServoPivot(wristPivotServo, TeamConstants.WRIST_PIVOT_MAX, TeamConstants.WRIST_PIVOT_MIN, TeamConstants.WRIST_PIVOT_MAX);
        gripper     = new ServoToggle(gripperServo, TeamConstants.GRIPPER_CLOSE, TeamConstants.GRIPPER_MIN_POS, TeamConstants.GRIPPER_MAX_POS);
        armPivot    = new MotorPivotExp(armMotor);
        slide       = new Slide(slideMotor);
        Functions   = new Func();

//        int[] myPortalIDs = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
//        aTPortalID = myPortalIDs[1];
//        colorPortalID = myPortalIDs[0];
//
//        atVision    = new VisionPortalObject.Builder(webCam2, aTPortalID)
//                .addProcessor(aprilTags.getProcessor())
//                .build();
//
//        colorVision =  new VisionPortalObject.Builder(webCam1, colorPortalID)
//                .addProcessor(bluSamps.colorProcessor())
//                .addProcessor(redSamps.colorProcessor())
//                .addProcessor((yelSamps.colorProcessor()))
//                .build();
    }


    /**
     * runOpMode must be Overridden in all OpModes
     * This is a requirement from the LinearOpMode class in the SDK
     */
    @Override
    public abstract void runOpMode() throws InterruptedException;


    /* ********* Setters, Getters, Utility and Helper Functions ********** */
    public void setAlliance(AllianceColor color){ alliance = color; }
    public static AllianceColor getAlliance(){ return alliance; }

//    public String hubA() {
//        double currentmA = 0;
//        for (LynxModule hub : ctrlHubs) {
//            currentmA += hub.getCurrent(CurrentUnit.AMPS);
//        }
//        return String.format(Locale.getDefault(), "%.3f mA", currentmA);
//    }

    public static String ctrlHubV() {
        return String.format(Locale.getDefault(), "%.3f V", ctrlHubs.get(0).getInputVoltage(VoltageUnit.VOLTS));
    }


    public static String expHubV() {
        return String.format(Locale.getDefault(), "%.3f V", ctrlHubs.get(1).getInputVoltage(VoltageUnit.VOLTS));
    }


}
