package org.firstinspires.ftc.teamcode.Robot;

public interface TeamConstants {

    /*~~~~~~~~~~~~~~~~~~~~~ MecanumDrive Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~*/
    double DEGRADED_DRIVE_LIMIT    = 0.45;
    double DEGRADED_STRAFE_LIMIT   = 0.35;
    double DEGRADED_TURN_LIMIT     = 0.25;
    double DEGRADED_SLIDE_EXTENDED = 10000; // TODO: update when the slide install is complete
    double DEGRADED_ARM_ROTATION   = 10000; // TODO: update when the arm install is complete


    /*~~~~~~~~~~~~~~~~~~~~~~~ Gripper Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~~~*/
    double GRIPPER_MAX_POS = 0.515;
    double GRIPPER_MIN_POS = 0.40;
    double GRIPPER_CLOSE   = 0.51;
    double GRIPPER_OPEN    = 0.42;


    /*~~~~~~~~~~~~~~~~~~~~~ Arm Rotate Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~~*/

    double DEGREES_PER_COUNT =  360.0/10766;//360/4062.8; //worm gear
            //4062.8 PPR
    double COUNTS_PER_DEGREE = 10766/360.0;
//    double DEGREES_PER_COUNT = 0.252614; // This for 1425.1ppr, 50.9:1 GBX Ratio, 117 RPM motor
    int MIN_POSITION_COUNTS = 0;    // TODO: update when the arm install is complete
    int MAX_POSITION_COUNTS = 9000; // TODO: update when the arm install is complete
    int PIVOT_SCORE_HIGH = 1;


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~ Slide Constants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    int    SLIDE_MIN = 0;
    int    SLIDE_MAX = 2060;  // 460 mm actual calculated is 2186 counts for 488 mm
    double INCHES_PER_COUNT = 0.00879;
    int    SLIDE_SCORE_HIGH = 1;


    /*~~~~~~~~~~~~~~~~~~~~~ Wrist Pivot Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~*/
    double WRIST_PIVOT_MIN    = 0.0;
    double WRIST_PIVOT_MAX    = 1.0;
    double WRIST_PIVOT_CENTER = 0.5;
    double WRISTPIVOT_SCORE_HIGH = 1;
    // Need preset positions


    /*~~~~~~~~~~~~~~~~~~~~ Wrist Rotate Subsystem Constants ~~~~~~~~~~~~~~~~~~~~*/
    double WRIST_ROTATE_MIN    = 0.2; // clockwise
    double WRIST_ROTATE_MAX    = 0.8; // counterclock
    double WRIST_ROTATE_CENTER = 0.5;


    /*~~~~~~~~~~~~~~~~~~~~ Main Movement Constants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    double Wrist_X_MAX = 30; // required for extension constraint rule
    double Wrist_X_MIN = 0; // required for extension constraint rule
    double Wrist_Y_MIN = -8.6; // required for not hit ground TODO: Fine Tune Value


    /*~~~~~~~~~~~~~~~~~~~~ Other ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    double bigMoveTolerance = 1;
    double closeEnoughDegTol = 20;
    double closeEnoughLenTol = 6;

}