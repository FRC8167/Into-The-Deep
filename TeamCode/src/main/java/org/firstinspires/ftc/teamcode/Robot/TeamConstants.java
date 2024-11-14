package org.firstinspires.ftc.teamcode.Robot;

public interface TeamConstants {

//    /*~~~~~~~~~~~~~~~~~~~~~ MecanumDrive Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~*/
//    double DEGRADED_DRIVE_LIMIT    = 0.45;
//    double DEGRADED_STRAFE_LIMIT   = 0.35;
//    double DEGRADED_TURN_LIMIT     = 0.25;
//    double DEGRADED_SLIDE_EXTENDED = 10000; // TODO: update when the slide install is complete
//    double DEGRADED_ARM_ROTATION   = 10000; // TODO: update when the arm install is complete
//
//
//    /*~~~~~~~~~~~~~~~~~~~~~ Wrist Pivot Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~*/
    double PIVOT_MIN = 0.0;
    double PIVOT_MAX = 1.0;
    double PIVOT_CENTER = 0.5;
    // Need preset positions


    /*~~~~~~~~~~~~~~~~~~~~ Wrist Rotate Subsystem Constants ~~~~~~~~~~~~~~~~~~~~*/
    double ROTATE_MIN = 0.5;
    double ROTATE_MAX = 0.0;
    double ROTATE_CENTER = 0.25;  // Needs Verified
//
//
//    /*~~~~~~~~~~~~~~~~~~~~~~~ Gripper Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~~~*/
<<<<<<< HEAD
    double GRIPPER_MAX_POS = 0.515;
    double GRIPPER_MIN_POS = 0.40;
    double GRIPPER_CLOSE   = 0.51;
    double GRIPPER_OPEN    = 0.45;

//    /*~~~~~~~~~~~~~~~~~~~~~ Arm Rotate Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~~*/
=======
//    double GRIPPER_MAX_POS = 0.515;
//    double GRIPPER_MIN_POS = 0.40;
//    double GRIPPER_CLOSE   = 0.51;
//    double GRIPPER_OPEN    = 0.45;
//
    /*~~~~~~~~~~~~~~~~~~~~~ Arm Rotate Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~~*/
>>>>>>> origin/master
    double DEGREES_PER_COUNT = 0.252614; // This for 1425.1ppr, 50.9:1 GBX Ratio, 117 RPM motor
    int MIN_POSITION_COUNTS = 1;    // TODO: update when the arm install is complete
    int MAX_POSITION_COUNTS = 800; // TODO: update when the arm install is complete


    /*~~~~~~~~~~~~~~~~~~~~~ MecanumDrive Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~*/
    double DEGRADED_DRIVE_LIMIT    = 0.45;
    double DEGRADED_STRAFE_LIMIT   = 0.35;
    double DEGRADED_TURN_LIMIT     = 0.25;
    double DEGRADED_SLIDE_EXTENDED = 10000;
    double DEGRADED_ARM_ROTATION   = 10000;


    /*~~~~~~~~~~~~~~~~~~~~~ Wrist Pivot Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~*/
    double WRIST_PIVOT_MIN    = 0.0;
    double WRIST_PIVOT_MAX    = 1.0;
    double WRIST_PIVOT_CENTER = 0.5;
    // Need preset positions


    /*~~~~~~~~~~~~~~~~~~~~ Wrist Rotate Subsystem Constants ~~~~~~~~~~~~~~~~~~~~*/
    double WRIST_ROTATE_MIN    = 0.2; // clockwise
    double WRIST_ROTATE_MAX    = 0.8; //counterclock
    double WRIST_ROTATE_CENTER = 0.5;


    /*~~~~~~~~~~~~~~~~~~~~ Slide Constants ~~~~~~~~~~~~~~~~~~~~~~~~~*/
    int SLIDE_MIN = 0;
    int SLIDE_MAX = 500;
    double INCHES_TO_COUNTS = 0.5;  //TODO:  update when value is known

}