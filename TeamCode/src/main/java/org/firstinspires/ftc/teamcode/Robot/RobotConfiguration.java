package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.SubSytems.Servo1D;
import org.firstinspires.ftc.teamcode.SubSytems.ServoToggle;
import org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors.AprilTagProcessorObject;
import org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors.ColorProcessor;
import org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors.VisionPortalObject;
import org.firstinspires.ftc.teamcode.SubSytems.MecanumDriveBasic;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.openftc.easyopencv.OpenCvCameraFactory;

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

    public int cameraMonitorViewId=0;
    int[] myIDs = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
    List myPortalsList;

    myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
    Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, (int) 0, false)).intValue();
    Portal_2_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, (int) 1, false)).intValue();

    /*------------- Private Class Variables - Preferred -------------*/
    static AllianceColor alliance;
    static List<LynxModule> ctrlHubs;
    static boolean initialized;


    /*----------- Define all Module Classes (SubSystems) ------------*/
    protected MecanumDriveBasic  drive;
    protected MecanumDrive       autoDrive;
    protected VisionPortalObject atVision;
    protected VisionPortalObject colorVision;
    protected Servo1D            wristRotate;
    protected Servo1D            wristPivot;
    protected ServoToggle        gripper;


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

        Servo wristPivotServo  = hardwareMap.get(Servo.class, "servo1");
        Servo wristRotateServo = hardwareMap.get(Servo.class, "servo2");
        Servo gripperServo     = hardwareMap.get(Servo.class, "servo0");





        WebcamName webCam1 = (WebcamName) OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), myIDs[0]);
        WebcamName webCam2 = (WebcamName) OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam2"), myIDs[1]);
//        WebcamName webCam1      = hardwareMap.get(WebcamName.class, "Webcam1");
//        WebcamName webCam2      = hardwareMap.get(WebcamName.class, "Webcam2");


        /** Create an object of every module/subsystem needed for both autonomous and teleOp modes. **/
        drive       = new MecanumDriveBasic(driveMotorLF, driveMotorLR, driveMotorRF, driveMotorRR);
        autoDrive   = new MecanumDrive(hardwareMap, startPose);
        wristRotate = new Servo1D(wristRotateServo, TeamConstants.PIVOT_CENTER, TeamConstants.PIVOT_MIN, TeamConstants.PIVOT_MAX);
        wristPivot  = new Servo1D(wristPivotServo, TeamConstants.ROTATE_CENTER, TeamConstants.ROTATE_MIN, TeamConstants.ROTATE_MAX);
        gripper     = new ServoToggle(gripperServo, TeamConstants.GRIPPER_CLOSE, TeamConstants.GRIPPER_MIN_POS, TeamConstants.GRIPPER_MAX_POS);
        atVision    = new VisionPortalObject.Builder(webCam1)
                                                    .addProcessor(aprilTags.getProcessor())
                                                    .build();
        colorVision =  new VisionPortalObject.Builder(webCam2)
                .addProcessor(bluSamps.colorProcessor())
                .addProcessor(redSamps.colorProcessor())
                .addProcessor((yelSamps.colorProcessor()))
                .build();


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
