package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Cogintilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.Cogintilities.Time;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.Robot.TeamConstants;

import java.util.ArrayList;
import java.util.List;

//@Disabled
@TeleOp(name="TeleOpMain", group="Competition")
public class TeleOpMain extends RobotConfiguration implements TeamConstants {

    GamepadWrapper driver;
    GamepadWrapper operator;


    @Override
    public void runOpMode() throws InterruptedException {
          //will need to chang
        double wristX;
        double wristY;
        if (InitAuto && !InitTele) {
            if (EndPos != null) {
                initializeRobot(EndPos);
            }
            else {
                initializeRobot(new Pose2d(0,0, HeadingAprox));
            }
//            wristX = AutoWristX; //288.500/25.4;// ~11.358in
//            wristY = AutoWristY; //-288.500/25.4;
            wristX = Functions.reverseTrigX(armPivot.getPosDegrees(), slide.getInches());
            wristY = Functions.reverseTrigY(armPivot.getPosDegrees(), slide.getInches());
            InitTele = true;
        } else if (!InitTele) {
            initializeRobot(new Pose2d(24,12,Math.toRadians(180)));
            armPivot.resetEncoders();
            slide.resetEncoders();
            InitTele = true;
            wristX = Functions.reverseTrigX(armPivot.getPosDegrees(), slide.getInches());
            wristY = Functions.reverseTrigY(armPivot.getPosDegrees(), slide.getInches());
            setAlliance(AllianceColor.BLUE);
            GoodPose = false;
        } else{
            initializeRobot(new Pose2d(24,12,Math.toRadians(180)));
            wristX = Functions.reverseTrigX(armPivot.getPosDegrees(), slide.getInches());
            wristY = Functions.reverseTrigY(armPivot.getPosDegrees(), slide.getInches());
            GoodPose = false;
        }
        slide.setDirection();
        double oldWristX;
        double oldWristY;
        boolean bigMove = false;
        boolean bigMSkip = false;
        boolean retractIsDone = true;
        boolean pivotIsDone = true;
        boolean extendIsDone = true;
        double newWristX = 288.500/25.4;// ~11.358in
        double newWristY = -288.500/25.4;
        boolean wristForward = true;
        boolean wrist0 = false;
        double trigMoveMultiplier = 1;
        boolean debugMode = false; //runs default commands for debugging purposes

        boolean sampleTrajectorys = true;
        boolean specimenTrajectorys = false;
        // true: back+a false: back+b
        // Looking from right side of robot
        // (0,0) at arm pivot
        // Units in inches

        driver   = new GamepadWrapper(gamepad1);
        operator = new GamepadWrapper(gamepad2);

        double fdrive = -driver.leftStick_Y;
        double strafe = driver.leftStick_X + (trigMoveMultiplier * 0.5 * operator.rightStick_X);
        double turn = driver.rightStick_X;
        double oldFdrive;
        double oldStrafe;
        double oldTurn;

        Time time = new Time();

        telemetry.update();



        FtcDashboard dash = FtcDashboard.getInstance();
        List<Action> runningActions = new ArrayList<>();



        waitForStart();

        while (opModeIsActive()) {
            bigMSkip = false;

            TelemetryPacket packet = new TelemetryPacket();
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions; // associates new action reference with runningActions as well allowing the original reference to be changed and still have access
            dash.sendTelemetryPacket(packet);

                    switch (getAlliance()) {
                        case BLUE:
                            if (sampleTrajectorys) {
                                if (driver.x.pressed() && GoodPose) {
                                    TrajectoryActionBuilder driveToBasketsBlue = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.toDouble()))
                                        .setTangent(Math.toRadians(-30))
                                        .splineToLinearHeading(sampleBasketScorePosBlue, Math.toRadians(45));
                                    Action toTheBasketsBlue = driveToBasketsBlue.build();
                                    newActions.add(toTheBasketsBlue);
                                }

                                if (driver.b.pressed() && GoodPose) {
                                    TrajectoryActionBuilder driveToSubBlue = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.toDouble()))
                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(sampleSubPickupPosBlue, Math.toRadians(160));  //-90??
                                    Action toTheSubBlue = driveToSubBlue.build();
                                    newActions.add(toTheSubBlue);

                                }
                            }

                            else if (specimenTrajectorys) {
                                if (driver.x.pressed() && GoodPose) {
                                    TrajectoryActionBuilder driveToSubBlue = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.toDouble()))
                                            .setTangent(Math.toRadians(180))
                                            .splineToSplineHeading(new Pose2d(-40,22, Math.toRadians(-90)), Math.toRadians(-90))
                                            .setTangent(Math.toRadians(-90))
                                            .splineToLinearHeading(specimenSubPickupPosBlue, Math.toRadians(20));
                                    Action toTheSubBlue = driveToSubBlue.build();
                                    newActions.add(toTheSubBlue);
                                }

                                if (driver.b.pressed() && GoodPose) {
                                    TrajectoryActionBuilder driveToObsBlue = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.toDouble()))
                                            .setTangent(Math.toRadians(160))
                                            .splineToLinearHeading(specimenObsDropGrabPosBlue, Math.toRadians(90));  //-90??
                                    Action toTheObsBlue = driveToObsBlue.build();
                                    newActions.add(toTheObsBlue);

                                }
                                if (driver.a.pressed() && GoodPose) {
                                    TrajectoryActionBuilder driveToChambBlue = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.toDouble()))
                                            .setTangent(Math.toRadians(0))
                                            .splineToLinearHeading(specimenChambHangPosBlue, Math.toRadians(0));  //-90??
                                    Action toTheChambBlue = driveToChambBlue.build();
                                    newActions.add(toTheChambBlue);

                                }
                            }
                            if (autoDrive.pose.position.x > 24) {
                            sampleTrajectorys = true;
                            specimenTrajectorys = false;

                            }
                            else if (autoDrive.pose.position.x < -24) {
                                sampleTrajectorys = false;
                                specimenTrajectorys = true;

                            }
                            break;
                        case RED:
                            if (sampleTrajectorys) {
                                if (driver.x.pressed() && GoodPose) {
                                    TrajectoryActionBuilder driveToBasketsRed = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.toDouble()))
                                            .setTangent(Math.toRadians(-30+180))
                                            .splineToLinearHeading(sampleBasketScorePosRed, Math.toRadians(45+180));
                                    Action toTheBasketsRed = driveToBasketsRed.build();
                                    newActions.add(toTheBasketsRed);
                                }

                                if (driver.b.pressed() && GoodPose) {
                                    TrajectoryActionBuilder driveToSubRed = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.toDouble()))
                                            .setTangent(Math.toRadians(-90+180))
                                            .splineToLinearHeading(sampleSubPickupPosRed, Math.toRadians(180+180));  //-90??
                                    Action toTheSubRed = driveToSubRed.build();
                                    newActions.add(toTheSubRed);

                                }
                            }

                            else if (specimenTrajectorys) {
                                if (driver.x.pressed() && GoodPose) {
                                    TrajectoryActionBuilder driveToSubRed = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.toDouble()))
                                            .setTangent(Math.toRadians(180+180))
                                            .splineToSplineHeading(new Pose2d(40,-22, Math.toRadians(-90+180)), Math.toRadians(-90+180))
                                            .splineToLinearHeading(specimenSubPickupPosRed, Math.toRadians(20));
                                    Action toTheSubRed = driveToSubRed.build();
                                    newActions.add(toTheSubRed);
                                }

                                if (driver.b.pressed() && GoodPose) {
                                    TrajectoryActionBuilder driveToObsRed = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.toDouble()))
                                            .setTangent(Math.toRadians(160+180))
                                            .splineToLinearHeading(specimenObsDropGrabPosRed, Math.toRadians(90+180));  //-90??
                                    Action toTheObsBlue = driveToObsRed.build();
                                    newActions.add(toTheObsBlue);

                                }
                                if (driver.a.pressed() && GoodPose) {
                                    TrajectoryActionBuilder driveToChambRed = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.toDouble()))
                                            .setTangent(Math.toRadians(0+180))
                                            .splineToLinearHeading(specimenChambHangPosRed, Math.toRadians(0+180));  //-90??
                                    Action toTheChambRed = driveToChambRed.build();
                                    newActions.add(toTheChambRed);

                                }
                            }
                            if (autoDrive.pose.position.x < -24) {
                                sampleTrajectorys = true;
                                specimenTrajectorys = false;

                            }
                            else if (autoDrive.pose.position.x > 24) {
                                sampleTrajectorys = false;
                                specimenTrajectorys = true;

                            }
                            break;
                    }

            if (driver.leftBumper.pressed()) {
                runningActions.clear();
                autoDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

            }




            if (driver.dpadRight.pressed()) {
                sampleTrajectorys = false;
                specimenTrajectorys = true;

            }
            oldFdrive = fdrive;
            oldStrafe = strafe;
            oldTurn = turn;
            if (driver.leftStick_Y != 0) {
                fdrive = Functions.clamp(oldFdrive - TeamConstants.Accel_Limit, -driver.leftStick_Y, oldFdrive + TeamConstants.Accel_Limit);
            }
            else{
                fdrive = Functions.clamp(oldFdrive - TeamConstants.Accel_Stop_Limit, -driver.leftStick_Y, oldFdrive + TeamConstants.Accel_Stop_Limit);
            }
            if (driver.leftStick_X != 0 || operator.rightStick_X != 0) {
                strafe = Functions.clamp(oldStrafe - TeamConstants.Accel_Limit, (driver.leftStick_X + (trigMoveMultiplier * 0.5 * operator.rightStick_X)), oldStrafe + TeamConstants.Accel_Limit);
            }
            else {
                strafe = Functions.clamp(oldStrafe - TeamConstants.Accel_Stop_Limit, (driver.leftStick_X + (trigMoveMultiplier * 0.5 * operator.rightStick_X)), oldStrafe + TeamConstants.Accel_Stop_Limit);
            }
            if (driver.rightStick_X != 0) {
                turn = Functions.clamp(oldTurn - TeamConstants.Accel_Limit, driver.rightStick_X, oldTurn + TeamConstants.Accel_Limit);
            }
            else {
                turn = Functions.clamp(oldTurn - TeamConstants.Accel_Stop_Limit, driver.rightStick_X, oldTurn + TeamConstants.Accel_Stop_Limit);
            }


            if (driver.y.pressed()) {
                if (GoodPose && operator.leftTrigger == 0) {
                    if (Math.toDegrees(autoDrive.pose.heading.toDouble()) < -90 || Math.toDegrees(autoDrive.pose.heading.toDouble()) > 90) {
                        autoDrive.pose = new Pose2d(23.80512, autoDrive.pose.position.y, Math.toRadians(180));
                    } else if (Math.toDegrees(autoDrive.pose.heading.toDouble()) > -90 || Math.toDegrees(autoDrive.pose.heading.toDouble()) < 90) {
                        autoDrive.pose = new Pose2d(-23.80512, autoDrive.pose.position.y, Math.toRadians(0));
                    }
                }
                else{
                    if (Math.toDegrees(autoDrive.pose.heading.toDouble()) < -90 || Math.toDegrees(autoDrive.pose.heading.toDouble()) > 90) {
                        EndPos = new Pose2d(23.80512, 0, Math.toRadians(180));
                        autoDrive.pose = new Pose2d(23.80512, 0, Math.toRadians(180));
                        GoodPose = true;
                    } else if (Math.toDegrees(autoDrive.pose.heading.toDouble()) > -90 || Math.toDegrees(autoDrive.pose.heading.toDouble()) < 90) {
                        EndPos = new Pose2d(-23.80512, 0, Math.toRadians(180));
                        autoDrive.pose = new Pose2d(-23.80512, 0, Math.toRadians(0));
                        GoodPose = true;
                    }
                }
            }

//            drive.setDegradedDrive(driver.rightBumper.whilePressed());
            if (driver.rightBumper.whilePressed()) { // || wristY > 25
                drive.setDegradedDrive(true, 0.45);
            } else if (wristY > 25) {
                drive.setDegradedDrive(true, 0.6);
            } else {
                drive.setDegradedDrive(false, 0.8);
            }
            if (runningActions.isEmpty()) {
                drive.mecanumDrive(fdrive, strafe, turn);
            }
            //Robot Pose to Score Baskets x = 58, y = 61, theta = 45
//            if (driver.x.pressed()) {
//                TrajectoryActionBuilder driveToBaskets = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.real))
//                        .setTangent(Math.toRadians(0))
//                        .splineToSplineHeading(basketScorePos, Math.toRadians(45));
//
//                Action toTheBaskets = driveToBaskets.build();
//
//                Actions.runBlocking(toTheBaskets);
//            }


            //Robot Pose to Submersible x = 24, y = 12, theta = 180
//            if (driver.b.pressed()) {
//                TrajectoryActionBuilder driveToSub = autoDrive.actionBuilder(new Pose2d(autoDrive.pose.position.x, autoDrive.pose.position.y, autoDrive.pose.heading.real))
//                        .setTangent(Math.toRadians(-90))
//                        .splineToSplineHeading(subPickupPos, Math.toRadians(180));  //-90??
//                Action toTheSub = driveToSub.build();
//                Actions.runBlocking(toTheSub);
//            }

            oldWristX = wristX;
            oldWristY = wristY;
            if (!bigMove) {
                newWristY = wristY + 0.4 * trigMoveMultiplier * (operator.rightTrigger - operator.leftTrigger);
                newWristX = wristX + 0.4 * trigMoveMultiplier * (-operator.rightStick_Y);
                wristX = newWristX;
                wristY = newWristY;
                wristX = Functions.TriClampX(wristX, wristY);
                wristY = Functions.TriClampY(wristX, wristY);
            }
            if (wristPivot.isEnabled())  {
                if (operator.dpadLeft.whilePressed()) wristForward = wristPivot.moveByPosFlat(wristX, wristY, wristForward);
                else if (wrist0)  wristPivot.setPosition(0);
                else wristForward = wristPivot.moveByPos(wristX, wristY, wristForward);
            }
            if (debugMode){
                wristPivot.setPosition(TeamConstants.WRIST_PIVOT_MIN);
                wristX = 17;
                wristY = 0;
            }
            if (operator.a.pressed()) gripper.toggleGripper(); //open/close gripper
            if ((operator.back.whilePressed()&& operator.a.pressed()) || (driver.back.whilePressed()&& driver.a.pressed())){ //debug mode on
                debugMode = true;
            }

            else if ((operator.back.whilePressed()&& operator.b.pressed()) || (driver.back.whilePressed()&& driver.b.pressed())){ //debug mode off
                debugMode = false;
            }

            if (operator.y.whilePressed()) { //yellow sample camera
                wristRotate.moveAng(yelSamps.CalcWristAngleDegrees());
            }

//            else if(operator.x.whilePressed()) {
//                switch (getAlliance()) {
//                    case RED:
//                        wristRotate.moveAng(redSamps.CalcWristAngleDegrees());
//                        break;
//                    case BLUE:
//                        wristRotate.moveAng(bluSamps.CalcWristAngleDegrees());
//                        break;
//                }
//            }
            else {
                    wristRotate.moveTrig(operator.leftStick_X, operator.leftStick_Y);
                }



//            telemetry.addData("Sample Angle Detected", bluSamps.CalcWristAngleDegrees());
//            telemetry.addData("Sample Angle Raw", bluSamps.getAlpha());
//            telemetry.addData("Width", bluSamps.getWidth());
//            telemetry.addData("Height", bluSamps.getHeight());
//            for (int i = 0; i <4; i++)
//            {
//                telemetry.addLine(String.format(Locale.ROOT,"%d, (%d, %d)", i, (int) bluSamps.getPoint()[i].x, (int) bluSamps.getPoint()[i].y));
//            }

            if(driver.a.whilePressed() && aprilTags.AprilTagUpdatePose()!=null)  {
//                for (AprilTagDetection detection : aprilTags.allAprilTagsDetected()) {
//                    if (detection.metadata != null) {
//                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                                detection.robotPose.getPosition().x,
//                                detection.robotPose.getPosition().y,
//                                detection.robotPose.getPosition().z));
//                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                                detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                                detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                                detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//                    } else {
//                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//                    }
//                }
//                double atX = aprilTags.AprilTagUpdatePose()[0];
//                double atY = aprilTags.AprilTagUpdatePose()[1];
//                double atH = aprilTags.AprilTagUpdatePose()[2];
//                Pose2d atPose = new Pose2d(atX, atY, atH);
//                Pose2d initialPose = atPose;
//                telemetry.addData("X: ", atX);
//                telemetry.addData("Y: ", atY);
//                telemetry.addData("H: ", atH);
            }

            if(operator.rightBumper.pressed()) { //score high basket
                wristX = 13;
                wristY = 31;
            }
            if(operator.leftBumper.pressed()) { //score in submersible
                wristX = 20;
                wristY = -3.5;
            }

            if(armPivot.closeEnough() && slide.closeEnough() && !wristPivot.isEnabled()){
                wristPivot.enable();
            }

            if(operator.b.pressed()) {
                if (wristY >= 29) wristPivot.disable();
                wristX = 17;
                wristY = 0;
            }
            if (operator.dpadRight.whilePressed()) {
                trigMoveMultiplier = 2;
            }
            else {
                trigMoveMultiplier = 1;
            }

            if (operator.dpadUp.pressed()){
                wristX = 20;
                wristY = 15;
            }

            if (operator.dpadDown.pressed()){
                wristX = Math.cos(Math.toRadians(armPivot.getPosDegrees()-45))*slide.getInches();
                wristY = Math.sin(Math.toRadians(armPivot.getPosDegrees()-45))*slide.getInches();
                bigMove = false;
                bigMSkip = true;

            }

            if (operator.x.whilePressed()) {
                wristX =17;
                wristY = -3;
                wrist0 = true;
                gripper.spinSpecimen();

//               gripper.setPosition(TeamConstants.GRIPPER_CLOSE-0.025);

            }

            else {
                if (gripper.servoPos() != TeamConstants.GRIPPER_CLOSE || gripper.servoPos() != TeamConstants.GRIPPER_OPEN) {
                    gripper.setToToggle();
                }

                if (wrist0) {
                    wristX = Functions.reverseTrigX(armPivot.getPosDegrees(),slide.getInches());
                    wristY = Functions.reverseTrigY(armPivot.getPosDegrees(),slide.getInches());
                }
                wrist0 = false;
            }

//            if (operator.dpadDown.pressed()) {
//
//                Actions.runBlocking(
//                        new SequentialAction(
//                                armPivot.armTrig(wristX-6,wristY),
//                                slide.slideTrig(wristX-6,wristY),
//                                wristPivot.wristTrig(wristX-6, wristY, wristForward),
//                                new SleepAction(0.5),
//                                gripper.toggle()
//                        )
//                );
//                wristX = 17;
//                wristY = 0;
////               gripper.setPosition(TeamConstants.GRIPPER_CLOSE-0.025);
//
//            } else if (gripper.servoPos() != TeamConstants.GRIPPER_CLOSE || gripper.servoPos() != TeamConstants.GRIPPER_OPEN) {
//                gripper.setToToggle();
//            }

            if (Math.sqrt((wristX-oldWristX)*(wristX-oldWristX)+(wristY-oldWristY)*(wristY-oldWristY)) > TeamConstants.bigMoveTolerance && !bigMSkip)
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
//            telemetry.addData("GoodPose: ", (GoodPose));
//            telemetry.addData("Blob Area", bluSamps.getArea());
//            telemetry.addData("GripServo: ", gripper.servoPos());
//            telemetry.addData("WristRotate: ", wristRotate.servoPos());
//            telemetry.addData("WristPivot: ", wristPivot.servoPos());
//            telemetry.addData("Pivot Encoder", armPivot.getPosition());
//            telemetry.addData("LX: ", operator.leftStick_X);
//            telemetry.addData("LY: ", operator.leftStick_Y);
//            telemetry.addData("RY: ", operator.rightStick_Y);
//            telemetry.addData("Ang: ", (Math.toDegrees(-Math.atan2(wristX, wristY))+135));
//            telemetry.addData("TargetX: ", wristX);
//            telemetry.addData("TargetY: ", wristY);
//            telemetry.addData("Length: ", (Math.sqrt(wristX*wristX+wristY*wristY)));
//            telemetry.addData("Test: ", (Math.sqrt(wristX*wristX+wristY*wristY)/(TeamConstants.SLIDE_MAX*TeamConstants.INCHES_PER_COUNT+(408/25.4))));
//            telemetry.addData("ClassAngle: ", (wristPivot.getAngle()));
//            telemetry.addData("ClassServoPos: ", (wristPivot.getServoPos()));
//            telemetry.addData("ClassServoPos: ", (bigMove));
//            telemetry.addData("retractDone: ", (retractIsDone));
//            telemetry.addData("pivotDone: ", (pivotIsDone));
//            telemetry.addData("extendDone: ", (extendIsDone));
//            telemetry.addData("bigMove: ", (bigMove));
//            telemetry.addData("Dist: ", (Math.sqrt((wristX-oldWristX)*(wristX-oldWristX)+(wristY-oldWristY)*(wristY-oldWristY))));
//            telemetry.addData("armBusy: ", (armPivot.getBusy()));
//            telemetry.addData("slideBusy: ", (slide.getBusy()));
//            telemetry.addData("rotateAng: ", (wristRotate.getRotateAcuteAng()));
//            telemetry.addData("PIvotEncoders: ", (armPivot.getPosition()));
//            telemetry.addData("SLideEncoders: ", (slide.getPosition()));
//            telemetry.addData("WristEnabled: ", (wristPivot.isEnabled()));
//            telemetry.addData("X: ", (autoDrive.pose.position.x));
//            telemetry.addData("Y: ", (autoDrive.pose.position.y));
//            telemetry.addData("H: ", (Math.toDegrees(autoDrive.pose.heading.toDouble())));
//            telemetry.addData("EndPose: ", (EndPos));
//            telemetry.addData("Samps: ", (sampleTrajectorys));
//            telemetry.addData("Spes: ", (specimenTrajectorys));
//            telemetry.addData("Spes: ", (specimenTrajectorys));
//            telemetry.addData("CalcX: ", (Functions.reverseTrigX(armPivot.getPosDegrees(), slide.getInches())));
//            telemetry.addData("CalcY: ", (Functions.reverseTrigY(armPivot.getPosDegrees(), slide.getInches())));
//            telemetry.addData("CalcAng: ", (armPivot.getPosDegrees()-45));
//            telemetry.addData("CalcLen: ", (slide.getInches()));
//            telemetry.addData("ArmMainPower: ", (armPivot.getMainPower()));
//            telemetry.addData("ArmSecondaryPower: ", (armPivot.getSecondaryPower()));
//            telemetry.addData("MainMOde: ", (armPivot.getMainMode()));
//            telemetry.addData("SecMOde: ", (armPivot.getSecMode()));
            telemetry.addData("Time: ", (time.milliseconds()));
            time.reset();
//            telemetry.addData("ArmMainDirection: ", (armPivot.getMainDirection()));
//            telemetry.addData("ArmMainVelocity: ", (armPivot.getMainVelocity()));
//            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), autoDrive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


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
        armPivot.periodic();
        autoDrive.updatePoseEstimate();
        if (autoDrive.pose.position.x > 72 || autoDrive.pose.position.x <-72 || autoDrive.pose.position.y > 72 || autoDrive.pose.position.y <-72){
            GoodPose = false;
        }
//        armPivot.periodic(18);
//        drive.periodic(getSlidePosition(), getPivotPosition());
    }
}

