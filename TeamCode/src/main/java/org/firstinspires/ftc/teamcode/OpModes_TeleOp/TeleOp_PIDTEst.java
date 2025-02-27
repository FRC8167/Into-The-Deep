package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Cogintilities.PidController;
import org.firstinspires.ftc.teamcode.Cogintilities.Time;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
import org.firstinspires.ftc.teamcode.SubSytems.MotorPivotExp;
import org.firstinspires.ftc.teamcode.SubSytems.TestAction;

//@Disabled
@TeleOp(name="TeleOp PID Test")
public class TeleOp_PIDTEst extends LinearOpMode {

    double target;
    double error;
    double lastError;
    double power;
    long currentTime;
    long previousTime;
    double kP;
    double kD;
    Time time;
    PidController pid;


    @Override
    public void runOpMode() {

        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        DcMotorEx armMotor2 = hardwareMap.get(DcMotorEx.class, "arm2");
        MotorPivotExp pivot = new MotorPivotExp(armMotor, armMotor2);
        time = new Time();
        pid = new PidController(0.03,0,0,20);

        target = TeamConstants.COUNTS_PER_DEGREE * 90;
        previousTime = System.currentTimeMillis();
        currentTime = System.currentTimeMillis();

        error = 0;
        lastError = 0;

        kP = 0.03;
        kD = 0;

//        MotorController = new PidController(0.05,0,0,1.5,-1,1);

        waitForStart();

        while (opModeIsActive()) {
//            lastError = error;
//            previousTime = currentTime;
//            currentTime = System.currentTimeMillis();
//
//            error = target - pivot.getPosition();
//
////            MotorController.setTargetPosition(target);
//
////            power = MotorController.update(pivot.getPosition());
//            power = error * kP + kD * (error - lastError) / time.seconds();
//            pivot.setPowers(power);
//            if (Math.abs(error) <= 20){
//                target = TeamConstants.COUNTS_PER_DEGREE * 45;
//            }
            pid.setTarget(target);
            power = pid.update(pivot.getPosition());
            pivot.setPowers(power);

            telemetry.addData("Power: ", power);
            telemetry.addData("Pos: ", pivot.getPosition()*TeamConstants.DEGREES_PER_COUNT);
            telemetry.update();






        }
    }

}
