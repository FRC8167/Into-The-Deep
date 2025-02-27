//package org.firstinspires.ftc.teamcode.OpModes_Autonomous;
//
//import android.annotation.SuppressLint;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
//import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
//
//@Disabled
//@Autonomous(name="AutoBlueSubNoHangSM", group="Autonomous", preselectTeleOp = "TeleOp")
//public class AutoBlueSubNoHangSM extends RobotConfiguration implements TeamConstants {
//
//    enum State {INIT, GOBASKET, DROPSAMPLE, GOSAMPLE1, PICKUPSAMPLE1, GOBASKET2, PICKUPSAMPLE3 };
//    State state = State.INIT;
//
//    boolean autoComplete = false;
//
//    @SuppressLint("DefaultLocale")
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        Pose2d initialPose = new Pose2d(12,63.5, -Math.PI/2);
//        initializeRobot(initialPose);
//        AutoWristX = 288.500/25.4;
//        AutoWristY = -288.500/25.4;
//        InitAuto = true;
//        setAlliance(AllianceColor.BLUE);
//
//        armPivot.resetEncoders();
//        slide.resetEncoders();
//
//       // ************************TRAJECTORIES****************************
//        TrajectoryActionBuilder blank = autoDrive.actionBuilder(initialPose)
//                .waitSeconds(0);
//
//        TrajectoryActionBuilder dropStart = autoDrive.actionBuilder(initialPose)
//                .setTangent(Math.toRadians(-45))
//                .splineToLinearHeading(new Pose2d(58, 61, Math.toRadians(45)), Math.toRadians(45));
//
//        TrajectoryActionBuilder sample1 = dropStart.endTrajectory().fresh()
//                .setTangent(Math.toRadians(-135))
//                .splineToLinearHeading(new Pose2d(48,46, Math.toRadians(-90)), Math.toRadians(-100));
//
//        TrajectoryActionBuilder drop1 = sample1.endTrajectory().fresh()
//                .strafeToSplineHeading(new Vector2d(58, 61), Math.toRadians(45));
//        TrajectoryActionBuilder sample2 = drop1.endTrajectory().fresh()
//                .strafeToSplineHeading(new Vector2d(58, 45), Math.toRadians(270));
//        TrajectoryActionBuilder drop2 = sample2.endTrajectory().fresh()
//                .strafeToSplineHeading(new Vector2d(58, 61), Math.toRadians(45));
//        TrajectoryActionBuilder prepTouch = drop2.endTrajectory().fresh()
//                .strafeToSplineHeading(new Vector2d(48, 12), Math.toRadians(180));
//        TrajectoryActionBuilder touch = prepTouch.endTrajectory().fresh()
//                .strafeToSplineHeading(new Vector2d(26, 12), Math.toRadians(180));
//
//
//
//        //**************************TRAJECTORIES -> ACTIONS  *********************
//
//        Action goDropStart = dropStart.build();
//        Action goSample1 = sample1.build();
//        Action goDrop1 = drop1.build();
//        Action goSample2 = sample2.build();
//        Action goDrop2 = drop2.build();
//
//        Action goPrepTouch = prepTouch.build();
//        Action goTouch = touch.build();
//
//
//
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//        newState(State.GOBASKET);
//
//        while(opModeIsActive() && !autoComplete) {
//
//            switch(state) {
//                case GOBASKET:
//                    Actions.runBlocking(goDropStart);
//                    armPivot.armTrig(16,33);
//                    sleep(500);
//                    slide.slideTrig(16,33);
//                    wristPivot.wristTrig(16,33, true);
//
//                    if(autoDrive.){
//
//                    }
//                    new SleepAction(1),
//                    gripper.toggle(),
//                    new SleepAction(0.5)
//                    break;
//                case DROPSAMPLE:
//                    break;
//                case GOSAMPLE1:
//                    break;
//                case PICKUPSAMPLE1:
//                    break;
//                case GOBASKET2:
//                    break;
//                case PICKUPSAMPLE3:
//                    break;
//            }
//
//        }
//
//
//
//        //************************** RUN THE ACTIONS  ****************************
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                new SequentialAction(
//                                        armPivot.armTrig(16,33),
//                                        new SleepAction(0.5),
//                                        slide.slideTrig(16,33),
//                                        wristPivot.wristTrig(16,33, true)
//                                ),
//                                goDropStart
//                        ),
//                        new SleepAction(1),
//                        gripper.toggle(),
//                        new SleepAction(0.5),
//
//                        goSample1,
//
//                        slide.slideToPosition(0),
//                        armPivot.armTrig(28,0),
//                        new SleepAction(0.5),
//                        slide.slideTrig(28,0),
//                        slide.slideTrig(22.5,-7.2),
//                        armPivot.armTrig(22.5,-7.2),
//                        wristPivot.wristTrig(22.5,-7.2, true),
//                        new SleepAction(0.5), // 1
//                        gripper.toggle(),
//                        new SleepAction(0.5),
//                        armPivot.armTrig(16,33),
//                        new ParallelAction(
//                                slide.slideTrig(16,33),
//                                wristPivot.wristTrig(16,33, true),
//                                goDrop1
//                                ),
//                        new SleepAction(0.5),
//                        gripper.toggle(),
//                        new SleepAction(0.5),
//                        goSample2,
//                        slide.slideToPosition(0),
//                        armPivot.armTrig(28,0),
//                        new SleepAction(0.5),
//                        slide.slideTrig(28,0),
//                        slide.slideTrig(22.5,-7.2),
//                        armPivot.armTrig(22.5,-7.2),
//                        wristPivot.wristTrig(22.5,-7.2, true),
//                        new SleepAction(0.5), // 1
//                            gripper.toggle(),
//                        new SleepAction(0.5),
//                        armPivot.armTrig(16,33),
//                        new ParallelAction(
//                                slide.slideTrig(16,33),
//                                wristPivot.wristTrig(16,33, true),
//                                goDrop2
//                                ),
//                        new SleepAction(0.5),//
//                        gripper.toggle(),
//                        new SleepAction(0.5),
//                        goPrepTouch,
//                        new ParallelAction(
//                            armPivot.armTrig(16,10),
//                            slide.slideTrig(16,10),
//                            wristPivot.wristTrig(16,10, true),
//                            goTouch
//                        ),
//                        armPivot.armTrig(16,6.5),
//                        slide.slideTrig(16,6.5),
//                        wristPivot.wristTrig(16,6.5, true)
//                )
//        );
//
//
//
////        Actions.runBlocking(wristPivot.setServoPosition(0.2));
//        AutoWristX = 16;
//        AutoWristY = 7.4;
//        EndPos = new Pose2d(new Vector2d(26, 12), Math.toRadians(180));
//        telemetry.update();
//
//        }
//
//
//    private void newState(State newState) {
//        state = newState;
//    }
//
//}
//
