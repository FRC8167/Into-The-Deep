package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSytems.TestAction;

//@Disabled
@TeleOp(name="TeleOp Counter")
public class TeleOp_Counter extends LinearOpMode {

    TestAction counter = new TestAction();
    boolean timesUp = false;

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive() && !timesUp) {
            Actions.runBlocking(
                    new SequentialAction(
                            counter.countTo(10)
                    )
            );
            timesUp = true;
        }
    }

}
