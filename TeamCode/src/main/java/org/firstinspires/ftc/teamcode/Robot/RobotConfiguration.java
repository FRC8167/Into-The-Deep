package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.SubSytems.MotorPivotExp;
import org.firstinspires.ftc.teamcode.SubSytems.Servo1D;
import org.firstinspires.ftc.teamcode.SubSytems.ServoRotate;
import org.firstinspires.ftc.teamcode.SubSytems.ServoToggle;
import org.firstinspires.ftc.teamcode.SubSytems.Slide;
import org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors.VisionPortalObject;
import org.firstinspires.ftc.teamcode.SubSytems.MecanumDriveBasic;
import org.firstinspires.ftc.teamcode.MecanumDrive;

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


    /*------------- Private Class Variables - Preferred -------------*/
    static AllianceColor alliance;
    static List<LynxModule> ctrlHubs;


    /*----------- Define all Module Classes (SubSystems) ------------*/
    static protected MecanumDriveBasic  drive;
    static protected MecanumDrive       autoDrive;
    static protected VisionPortalObject vision;
    static protected ServoRotate        wristRotate;
    static protected Servo1D            wristPivot;
    static protected ServoToggle        gripper;
    static protected MotorPivotExp      armPivot;
    static protected Slide              slide;

    /**
     * initializeRobot:
     * Initialize robot with a specified start pose (used by Road Runner. This function should be
     * called immediately in the OpMode's runOpMode function. A null value error will result if you
     * try to use any devices connected to the control hub that have not been initialized.  This
     * function creates the Hardware Map and the module objects that use these devices.
     *
     * @throws InterruptedException
     */
    public double initializeRobot(Pose2d startPose) throws InterruptedException {

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

        DcMotorEx armMotorL = hardwareMap.get(DcMotorEx.class, "armL");
        DcMotorEx armMotorR = hardwareMap.get(DcMotorEx.class, "armR");
        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");

        Servo wristPivotServo = hardwareMap.get(Servo.class, "servo1");
        Servo wristRotateServo = hardwareMap.get(Servo.class, "servo2");
        Servo gripperServo = hardwareMap.get(Servo.class, "servo0");

        WebcamName webCam = hardwareMap.get(WebcamName.class, "Webcam1");

        /** Create an object of every module/subsystem needed for both autonomous and teleOp modes. **/
        drive = new MecanumDriveBasic(driveMotorLF, driveMotorLR, driveMotorRF, driveMotorRR);
        autoDrive = new MecanumDrive(hardwareMap, startPose);
        wristRotate = new ServoRotate(wristRotateServo, TeamConstants.WRIST_ROTATE_CENTER, TeamConstants.WRIST_ROTATE_MIN, TeamConstants.WRIST_ROTATE_MAX);
        wristRotateServo.setPosition(TeamConstants.WRIST_ROTATE_CENTER);
        wristPivot = new Servo1D(wristPivotServo, TeamConstants.WRIST_PIVOT_MAX, TeamConstants.WRIST_PIVOT_MIN, TeamConstants.WRIST_PIVOT_MAX);
        wristPivotServo.setPosition(TeamConstants.WRIST_PIVOT_MAX);
        gripper = new ServoToggle(gripperServo, TeamConstants.GRIPPER_CLOSE, TeamConstants.GRIPPER_MIN_POS, TeamConstants.GRIPPER_MAX_POS);
        gripperServo.setPosition(TeamConstants.GRIPPER_CLOSE);
        vision = new VisionPortalObject(webCam);
        armPivot = new MotorPivotExp(armMotorR,armMotorL);
        slide = new Slide(slideMotor);
        return gripper.servoPos();
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
