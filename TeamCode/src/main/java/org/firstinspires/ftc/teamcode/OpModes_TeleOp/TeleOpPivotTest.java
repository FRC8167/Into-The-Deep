package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

@Disabled
@TeleOp(name="PivotTest", group="Competition")
public class TeleOpPivotTest extends RobotConfiguration implements TeamConstants {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot(new Pose2d(0,0,0));
        armPivot.resetEncoders();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad2.x) {  // 33.536842 counts/deg
                armPivot.setPositionCounts(armPivot.degreesToCounts(45));
            }

            telemetry.addData("EncoderCounts", armPivot.getPosition());
            telemetry.update();
        }
    }
}
