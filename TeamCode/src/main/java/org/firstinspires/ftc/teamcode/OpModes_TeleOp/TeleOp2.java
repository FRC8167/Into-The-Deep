package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Cogintilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
import org.firstinspires.ftc.teamcode.SubSytems.MotorPivotExp;

//@Disabled
@TeleOp(name="TeleOp2", group="Competition")
public class TeleOp2 extends RobotConfiguration implements TeamConstants {

    GamepadWrapper driver;
    GamepadWrapper operator;
    boolean autoDriveInProgress;
    boolean armMotionInProgress;

    @Override
    public void runOpMode() throws InterruptedException {

        driver = new GamepadWrapper(gamepad1);
        operator = new GamepadWrapper(gamepad2);
        autoDriveInProgress = false;
        armMotionInProgress = false;

        initializeRobot(new Pose2d(0,0,0));

        waitForStart();

        while (opModeIsActive()) {
            if (!autoDriveInProgress) {

                if (driver.leftBumper.pressed())  drive.setDegradedDrive(true);
                if (driver.leftBumper.released()) drive.setDegradedDrive(false);
                drive.mecanumDrive(-driver.leftStick_Y, driver.leftStick_X, driver.rightStick_X);

                armPivot.manualMove(operator.leftStick_Y);
            }
            if (operator.y.pressed()) scoreHigh();


            /*
            manual move
            - arm pivot (left joystick y)
            - wrist (right joystick y pivot, right joystick x rotate)
            - slide (right bumper out, left bumper in)
            move to position
            - pick up sample
            - drop (low and high baskets)
            hang
            - low and high
            nest
             */

            periodic();
        }
    }


    public void periodic(){
        driver.update();
        operator.update();
        if (armPivot.inMotion()) {

        }

    }


    public void scoreHigh(){
        armPivot.setPositionCounts(PIVOT_SCORE_HIGH);
        slide.setPositionCounts(TeamConstants.SLIDE_SCORE_HIGH);
        wristPivot.setPosition(WRISTPIVOT_SCORE_HIGH);
        wristRotate.setPosition(TeamConstants.WRIST_ROTATE_CENTER);
        armMotionInProgress = true;
    }
}
