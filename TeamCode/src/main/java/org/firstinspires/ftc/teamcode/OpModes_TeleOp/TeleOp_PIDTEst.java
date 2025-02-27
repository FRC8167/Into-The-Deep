package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Cogintilities.PidController;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
import org.firstinspires.ftc.teamcode.SubSytems.MotorPivotExp;
import org.firstinspires.ftc.teamcode.SubSytems.TestAction;

//@Disabled
@TeleOp(name="TeleOp PID Test")
public class TeleOp_PIDTEst extends LinearOpMode {

    double target;
    double error;
    double lastError;
    long currentTime;
    long previousTime;
    PidController MotorController;


    @Override
    public void runOpMode() {

        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        DcMotorEx armMotor2 = hardwareMap.get(DcMotorEx.class, "arm2");
        MotorPivotExp pivot = new MotorPivotExp(armMotor, armMotor2);

        target = TeamConstants.COUNTS_PER_DEGREE * 45;
        previousTime = System.currentTimeMillis();
        currentTime = System.currentTimeMillis();

        MotorController = new PidController(1,0,0,20,-1,1);

        waitForStart();

        while (opModeIsActive()) {
            previousTime = currentTime;
            currentTime = System.currentTimeMillis();

            error = target - pivot.getPosition();
            MotorController.setTargetPosition(target);
            pivot.setPowers(MotorController.update(pivot.getPosition()));





        }
    }

}
