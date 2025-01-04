package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Cogintilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

import java.util.Locale;

//@Disabled
@TeleOp(name="TeleOpMain", group="Competition")
public class TeleOpMain extends RobotConfiguration implements TeamConstants {

    GamepadWrapper driver;
    GamepadWrapper operator;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot(new Pose2d(0,0,0));  //will need to chang
        slide.setDirection();
        double wristX;
        double wristY;
        if (InitAuto) {
            wristX = AutoWristX; //288.500/25.4;// ~11.358in
            wristY = AutoWristY; //-288.500/25.4;
        }
        else{
            wristX = 288.500/25.4;// ~11.358in
            wristY = -288.500/25.4;
            armPivot.resetEncoders();
            slide.resetEncoders();
        }
        double oldWristX;
        double oldWristY;
        boolean bigMove = false;
        boolean retractIsDone = true;
        boolean pivotIsDone = true;
        boolean extendIsDone = true;
        double newWristX;// ~11.358in
        double newWristY;
        boolean wristForward = true;
        // Looking from right side of robot
        // (0,0) at arm pivot
        // Units in inches

        telemetry.update();

        driver   = new GamepadWrapper(gamepad1);
        operator = new GamepadWrapper(gamepad2);

        TrajectoryActionBuilder driveToBaskets = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.real))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(basketScorePos, Math.toRadians(45));

        TrajectoryActionBuilder driveToSub = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.real))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(subPickupPos, Math.toRadians(180));  //-90??

        Action toTheBaskets = driveToBaskets.build();
        Action toTheSub = driveToSub.build();

        waitForStart();

        while (opModeIsActive()) {

            double fdrive = -driver.leftStick_Y;
            double strafe = driver.leftStick_X + 0.25 * operator.rightStick_X;
            double turn = driver.rightStick_X ;

//            drive.setDegradedDrive(driver.rightBumper.whilePressed());
            if (driver.rightBumper.whilePressed()) { // || wristY > 25
                drive.setDegradedDrive(true, 0.45);
            } else {
                drive.setDegradedDrive(false, 0.8);
            }
            drive.mecanumDrive(fdrive, strafe, turn);

            //Robot Pose to Score Baskets x = 58, y = 61, theta = 45
            if(driver.x.pressed()) {
                Actions.runBlocking(toTheBaskets);
            }

            //Robot Pose to Submersible x = 24, y = 12, theta = 180
            if (driver.b.pressed()) {
                Actions.runBlocking(toTheSub);
            }

            oldWristX = wristX;
            oldWristY = wristY;
            if (!bigMove) {
                newWristY = wristY + 0.1 * (operator.rightTrigger - operator.leftTrigger);
                newWristX = wristX + 0.1 * (-operator.rightStick_Y);
                wristX = newWristX;
                wristY = newWristY;
                wristX = Functions.TriClampX(wristX, wristY);
                wristY = Functions.TriClampY(wristX, wristY);
            }
            wristForward = wristPivot.moveByPos(wristX,wristY,wristForward);

            if(operator.a.pressed()) gripper.toggleGripper();


            telemetry.addData("Sample Angle Detected", bluSamps.CalcWristAngleDegrees());
            telemetry.addData("Sample Angle Raw", bluSamps.getAlpha());
            telemetry.addData("Width", bluSamps.getWidth());
            telemetry.addData("Height", bluSamps.getHeight());
            for (int i = 0; i <4; i++)
            {
                telemetry.addLine(String.format(Locale.ROOT,"%d, (%d, %d)", i, (int) bluSamps.getPoint()[i].x, (int) bluSamps.getPoint()[i].y));
            }


            if(operator.rightBumper.pressed()) {
                wristX = 5;
                wristY = 34;
            }
            if(operator.leftBumper.pressed()) {
                wristX = 23.5;
                wristY = 0;
            }
            if(operator.b.pressed()) {
                wristX = 17;
                wristY = 0;
            }

            /* ********************************************************/
            if(operator.x.whilePressed()) {
                wristRotate.moveAng(bluSamps.CalcWristAngleDegrees());
            }
            else {
                wristRotate.moveTrig(operator.leftStick_X, operator.leftStick_Y);
            }
            if (Math.sqrt((wristX-oldWristX)*(wristX-oldWristX)+(wristY-oldWristY)*(wristY-oldWristY)) > TeamConstants.bigMoveTolerance)
                {
                bigMove = true;
                retractIsDone = false;
                pivotIsDone = false;
                extendIsDone = false;

            }

            if (!bigMove) {
                armPivot.triangulateTo(wristX, wristY);
                slide.triangulateTo(wristX, wristY);
            }
            else {
                if (!retractIsDone) {
                    slide.setPosition(0);
                    retractIsDone = slide.closeEnough();
                } else if (!pivotIsDone) {
                    armPivot.triangulateTo(wristX, wristY);
                    pivotIsDone = armPivot.closeEnough();
                } else if (!extendIsDone) {
                    extendIsDone = slide.closeEnough();
                } else {
                    bigMove = false;
                }
            }

            /* Output Telemtery Data to Driver Stations */
            telemetry.addData("GripServo: ", gripper.servoPos());
            telemetry.addData("WristRotate: ", wristRotate.servoPos());
            telemetry.addData("WristPivot: ", wristPivot.servoPos());
            telemetry.addData("Pivot Encoder", armPivot.getPosition());
            telemetry.addData("LX: ", operator.leftStick_X);
            telemetry.addData("LY: ", operator.leftStick_Y);
            telemetry.addData("RY: ", operator.rightStick_Y);
            telemetry.addData("Ang: ", (Math.toDegrees(-Math.atan2(wristX, wristY))+135));
            telemetry.addData("TargetX: ", wristX);
            telemetry.addData("TargetY: ", wristY);
            telemetry.addData("Length: ", (Math.sqrt(wristX*wristX+wristY*wristY)));
            telemetry.addData("Test: ", (Math.sqrt(wristX*wristX+wristY*wristY)/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4))));
            telemetry.addData("ClassAngle: ", (wristPivot.getAngle()));
            telemetry.addData("ClassServoPos: ", (wristPivot.getServoPos()));
            telemetry.addData("ClassServoPos: ", (bigMove));
            telemetry.addData("retractDone: ", (retractIsDone));
            telemetry.addData("pivotDone: ", (pivotIsDone));
            telemetry.addData("extendDone: ", (extendIsDone));
            telemetry.addData("bigMove: ", (bigMove));
            telemetry.addData("Dist: ", (Math.sqrt((wristX-oldWristX)*(wristX-oldWristX)+(wristY-oldWristY)*(wristY-oldWristY))));
            telemetry.addData("armBusy: ", (armPivot.getBusy()));
            telemetry.addData("slideBusy: ", (slide.getBusy()));
            telemetry.addData("rotateAng: ", (wristRotate.getRotateAcuteAng()));
            telemetry.addData("PIvotEncoders: ", (armPivot.getPosition()));
            telemetry.addData("SLideEncoders: ", (slide.getPosition()));



//             if(operator.rightStick_Y > 0.1 || operator.rightStick_Y < -0.1) {
//                armPivot.manualMove(operator.rightStick_Y);
//             }
//
//             if (operator.rightTrigger > 0.0 || operator.leftTrigger > 0.0)  {
//                 slide.manualMove(operator.leftTrigger, operator.rightTrigger);
//             }

            telemetry.update();
            periodicCalls();
        }
    }


    private void periodicCalls() {
        driver.update();
        operator.update();
        autoDrive.updatePoseEstimate();
//        armPivot.periodic(18);
//        drive.periodic(getSlidePosition(), getPivotPosition());
    }
}

