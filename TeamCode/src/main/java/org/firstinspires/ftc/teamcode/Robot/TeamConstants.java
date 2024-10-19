package org.firstinspires.ftc.teamcode.Robot;

public interface TeamConstants {

    /*~~~~~~~~~~~~~~~~~~~~~ MecanumDrive Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~*/
    double DEGRADED_DRIVE_LIMIT    = 0.45;
    double DEGRADED_STRAFE_LIMIT   = 0.35;
    double DEGRADED_TURN_LIMIT     = 0.25;
    double DEGRADED_SLIDE_EXTENDED = 10000;
    double DEGRADED_ARM_ROTATION   = 10000;


    /*~~~~~~~~~~~~~~~~~~~~~ Wrist Pivot Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~*/
    double WRIST_PIVOT_MIN = 0.0;
    double WRIST_PIVOT_MAX = 1.0;
    double WRIST_PIVOT_CENTER = 0.5;


    /*~~~~~~~~~~~~~~~~~~~~ Wrist Rotate Subsystem Constants ~~~~~~~~~~~~~~~~~~~~*/
    double WRIST_ROTATE_MIN = 0.5;
    double WRIST_ROTATE_MAX = 0.0;
    double WRIST_ROTATE_CENTER = 0.25;  // Needs Verified


    /*~~~~~~~~~~~~~~~~~~~~~~~ Gripper Subsystem Constants ~~~~~~~~~~~~~~~~~~~~~~~*/
    double GRIPPER_CLOSE = 0.515;
    double GRIPPER_OPEN  = 0.45;

}