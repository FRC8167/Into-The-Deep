package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.Pose2d;

public interface TeamConstants {

    /*~~~~~~~~~~~~~~~~~~~~~ MecanumDrive Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~*/
    double DEGRADED_DRIVE_LIMIT    = 0.45;
    double DEGRADED_STRAFE_LIMIT   = 0.35;
    double DEGRADED_TURN_LIMIT     = 0.25;
    double Accel_Limit             = 0.2;
    double Accel_Stop_Limit        = 0.4;


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Field Positions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    Pose2d sampleBasketScorePosBlue = new Pose2d(57,60, Math.toRadians(-315));
    Pose2d sampleSubPickupPosBlue = new Pose2d(24,9, Math.toRadians(180));

    Pose2d specimenSubPickupPosBlue = new Pose2d(-24,9, Math.toRadians(0));
    Pose2d specimenObsDropGrabPosBlue = new Pose2d(-48,48, Math.toRadians(90));
    Pose2d specimenChambHangPosBlue = new Pose2d(0,48, Math.toRadians(-90));


    Pose2d sampleBasketScorePosRed = new Pose2d(-57,-60, Math.toRadians(-315+180));
    Pose2d sampleSubPickupPosRed = new Pose2d(-24,-9, Math.toRadians(180+180));

    Pose2d specimenSubPickupPosRed = new Pose2d(24,-9, Math.toRadians(0+180));
    Pose2d specimenObsDropGrabPosRed = new Pose2d(48,-48, Math.toRadians(90+180));
    Pose2d specimenChambHangPosRed = new Pose2d(0,-48, Math.toRadians(-90+180));



    /*~~~~~~~~~~~~~~~~~~~~~~~ Gripper Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~~~*/
//    double GRIPPER_Multiplier = 1; // 65.25262 original, 85.25262 extended
    double GRIPPER_Multiplier = 1; // 65.25262 original, 85.25262 extended
    double GRIPPER_LENGTH_OFFSET = 0;


    double GRIPPER_MAX_POS = 0.54;
    double GRIPPER_MIN_POS = 0.40;
    double GRIPPER_CLOSE   = 0.53;
    double GRIPPER_OPEN    = GRIPPER_CLOSE - 0.11/GRIPPER_Multiplier; //0.42

    double GRIPPER_SPIN = GRIPPER_CLOSE-0.025/GRIPPER_Multiplier; //-0.025
    double GRIPPER_EOPEN = GRIPPER_OPEN-0.1/GRIPPER_Multiplier;




    /*~~~~~~~~~~~~~~~~~~~~~ Arm Rotate Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~~*/

    double COUNTS_PER_DEGREE = 32.0833333; //10766/360.0;
    double DEGREES_PER_COUNT =  1/COUNTS_PER_DEGREE; //360.0/10766;//360/4062.8; //worm gear
    //4062.8 PPR

//    double DEGREES_PER_COUNT = 0.252614; // This for 1425.1ppr, 50.9:1 GBX Ratio, 117 RPM motor
    int MIN_POSITION_COUNTS = 0;    // TODO: update when the arm install is complete
    int MAX_POSITION_COUNTS = (int) (150 * COUNTS_PER_DEGREE); // TODO: update when the arm install is complete
    int PIVOT_SCORE_HIGH = 1;
    double m1 = 0.0027839;
    double m2 = 0.173144;
    double l = 14.17;

    double PIVOT_MAX_VEL = 124.675*COUNTS_PER_DEGREE;

    double SecMaxTorque = 133.2; //kg*cm
    double SecMaxSpeed = 60; //RPM
    double SecUpMultiplier = 0.9;
    double SecDownMultiplier = 0;


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~ Slide Constants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    double COUNTS_PER_ROTATION = 384.5; //537.7;
    double INCHES_PER_COUNT = (976.0/25.4)/(2928/360.0 * COUNTS_PER_ROTATION);// (mm/(to in))/(degPerMax/360 * CPR)
    double MILLIMETERS_PER_COUNT = INCHES_PER_COUNT * 25.4;

    int    SLIDE_MIN = 0;
    int    SLIDE_MAX = (int) (460/MILLIMETERS_PER_COUNT);// 2060  // 460 mm actual calculated is 2186 counts for 488 mm
//    double INCHES_PER_COUNT = 0.00879;
    int    SLIDE_SCORE_HIGH = 1;


    /*~~~~~~~~~~~~~~~~~~~~~ Wrist Pivot Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~*/
    double WRIST_PIVOT_MIN    = 0.0;
    double WRIST_PIVOT_MAX    = 0.8;
    double WRIST_PIVOT_CENTER = 0.5;
    double WRISTPIVOT_SCORE_HIGH = 1;
    // Need preset positions


    /*~~~~~~~~~~~~~~~~~~~~ Wrist Rotate Subsystem Constants ~~~~~~~~~~~~~~~~~~~~*/
    double WRIST_ROTATE_MIN    = 0.2; // clockwise
    double WRIST_ROTATE_MAX    = 0.8; // counterclock
    double WRIST_ROTATE_CENTER = 0.5;


    /*~~~~~~~~~~~~~~~~~~~~ Main Movement Constants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    double Wrist_X_MAX = 30; // required for extension constraint rule
    double Wrist_X_MIN = 3; // required for extension constraint rule
    double Wrist_Y_MIN = -8.6; // required for not hit ground TODO: Fine Tune Value

    /*~~~~~~~~~~~~~~~~~~~~ Vision ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    int FRAME_WIDTH = 640;
    int FRAME_HEIGHT = 480;
    double DRIVE_KP = 0.1;

    /*~~~~~~~~~~~~~~~~~~~~ Other ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    double bigMoveTolerance = 1;
    double closeEnoughDegTol = 20;
    double closeEnoughLenTol = 6;

}