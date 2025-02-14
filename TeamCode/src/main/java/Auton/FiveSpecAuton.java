package Auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import RoadRunner.MecanumDrive;
import Robot.Robot;

@Config
@Autonomous(name = "5 Specimen", group = "Auton")
public class FiveSpecAuton extends LinearOpMode {

    private final Robot robot = new Robot();
    private MecanumDrive Drive;

    enum OuttakeState {BASE, COLLECTION, SCORING, REVERSESCORING, INIT}

    private static final int BASE = 0, FS = 450, LIFTED = 390, RAISED = 900;
    private static final double OPEN = 0.25, CLOSE = 0.75;
    private static final double UP = 0.40, DOWN = 0.0;
    private static final double STATIONARY = 0.0;

    private double SweepBuffer = 0.185;
    private double ClawBuffer = 0.16;
    private double VerticalSlideBuffer = 0.23;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Face Backwards. Middle of Center 2 Tiles. Up Against Border.");
        telemetry.update();

        Pose2d initialPosition = new Pose2d(0, -62, Math.toRadians(270));
        Drive = new MecanumDrive(hardwareMap, initialPosition); // Run At 100%

        robot.init(hardwareMap);
        robot.scoring.resetScoringEncoders();

        Actions.runBlocking(new ParallelAction(
                // Robot Setup
                new IntakeAction(STATIONARY),
                new HorizontalSlideAction(-250),
                new ClawAction(CLOSE),
                new OutTakeAction(OuttakeState.INIT)
        ));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(

                new VerticalSlideAction(10),

                new ParallelAction(
                        new OutTakeAction(OuttakeState.REVERSESCORING),
                        new VerticalSlideAction(FS),
                        Drive.actionBuilder(initialPosition)
                                .strafeToConstantHeading(new Vector2d(0, -32))
                                .build()
                ),

                new VerticalSlideAction(RAISED + 100),
                new SleepAction(VerticalSlideBuffer + 0.07),
//                new ClawAction(OPEN),
//                new SleepAction(ClawBuffer),


                new ParallelAction(
                        new ClawAction(OPEN),
                        new SleepAction(ClawBuffer),
                        new OutTakeAction(OuttakeState.COLLECTION),
                        new VerticalSlideAction(BASE)
                ),

                // Move to Sample Collection Zone
                Drive.actionBuilder(new Pose2d(0, -32, Math.toRadians(270)))
                        .setReversed(false)
                        .setTangent(5)
                        .splineToLinearHeading(new Pose2d(32, -42, Math.toRadians(80)), .8)
                        .build(),


                new SweeperAction(DOWN),
                new SleepAction(SweepBuffer),

                // Sweep Samples
                // Sample 1
                Drive.actionBuilder(new Pose2d(32, -42, Math.toRadians(80)))
                        .waitSeconds(SweepBuffer)
                        .strafeToLinearHeading(new Vector2d(39, -44), Math.toRadians(-56))
                        .build(),

                new SweeperAction(UP),

                Drive.actionBuilder(new Pose2d(39, -44, Math.toRadians(-56)))
                        .strafeToLinearHeading(new Vector2d(39, -38), Math.toRadians(85))
                        .build(),

                new SweeperAction(DOWN),

                // Sample 2
                Drive.actionBuilder(new Pose2d(39, -38, Math.toRadians(85)))
                        .waitSeconds(SweepBuffer)
                        .strafeToLinearHeading(new Vector2d(46, -44), Math.toRadians(-60))
                        .build(),

                new SweeperAction(UP),
                new SleepAction(SweepBuffer),

                Drive.actionBuilder(new Pose2d(46, -44, Math.toRadians(-60)))
                        .strafeToLinearHeading(new Vector2d(50, -34), Math.toRadians(90))
                        .build(),

                new SweeperAction(DOWN),

                // Sample 3
                Drive.actionBuilder(new Pose2d(50.5, -34, Math.toRadians(90)))
                        .waitSeconds(SweepBuffer)
                        .strafeToLinearHeading(new Vector2d(50.5, -44), Math.toRadians(-70))
                        .build(),

                new SweeperAction(0.7),


                // Collect Specimen #2
                Drive.actionBuilder(new Pose2d(50.5, -44, Math.toRadians(-70)))
                        .strafeToLinearHeading(new Vector2d(40, -72.2), Math.toRadians(90))
                        .build(),

                // Grab Specimen
                new ClawAction(CLOSE),
                new SleepAction(ClawBuffer),

                // Score Specimen #2
                new ParallelAction(
                        new OutTakeAction(OuttakeState.SCORING),
                        new VerticalSlideAction(LIFTED+25),

                        Drive.actionBuilder(new Pose2d(40, -72.2, Math.toRadians(90)))
                                .strafeToConstantHeading(new Vector2d(-6, -32))
                                .build()
                ),

                new VerticalSlideAction(RAISED),
                new SleepAction(VerticalSlideBuffer),
//                new ClawAction(OPEN),
//                new SleepAction(ClawBuffer),

                new ParallelAction(
                        new ClawAction(OPEN),
                        new SleepAction(ClawBuffer),
                        new OutTakeAction(OuttakeState.COLLECTION),
                        new VerticalSlideAction(BASE)
                ),

                // Collect Specimen #3
                Drive.actionBuilder(new Pose2d(-6, -32, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(40, -72.2))
                        .build(),

                // Grab Specimen
                new ClawAction(CLOSE),
                new SleepAction(ClawBuffer),

                // Score Specimen #3
                new ParallelAction(
                        new OutTakeAction(OuttakeState.SCORING),
                        new VerticalSlideAction(LIFTED),

                        Drive.actionBuilder(new Pose2d(40, -72.2, Math.toRadians(90)))
                                .strafeToConstantHeading(new Vector2d(-3, -32))
                                .build()
                ),

                new VerticalSlideAction(RAISED),
                new SleepAction(VerticalSlideBuffer),
//                new ClawAction(OPEN),
//                new SleepAction(ClawBuffer),

                new ParallelAction(

                        new ClawAction(OPEN),
                        new SleepAction(ClawBuffer),
                        new OutTakeAction(OuttakeState.COLLECTION),
                        new VerticalSlideAction(BASE)
                ),


                // Collect Specimen #4
                Drive.actionBuilder(new Pose2d(-3, -32, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(40, -72.2))
                        .build(),

                // Grab Specimen
                new ClawAction(CLOSE),
                new SleepAction(ClawBuffer),

                // Score Specimen #4
                new ParallelAction(
                        new OutTakeAction(OuttakeState.SCORING),
                        new VerticalSlideAction(LIFTED),
                        Drive.actionBuilder(new Pose2d(40, -72.2, Math.toRadians(90)))
                                .strafeToConstantHeading(new Vector2d(3, -32))
                                .build()
                ),

                new VerticalSlideAction(RAISED),
                new SleepAction(VerticalSlideBuffer),
//                new ClawAction(OPEN),
//                new SleepAction(ClawBuffer),

                new ParallelAction(

                        new ClawAction(OPEN),
                        new SleepAction(ClawBuffer),
                        new OutTakeAction(OuttakeState.COLLECTION),
                        new VerticalSlideAction(BASE)
                ),


                // Collect Specimen #6
                Drive.actionBuilder(new Pose2d(3, -32, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(40, -72.2))
                        .build(),

                // Grab Specimen
                new ClawAction(CLOSE),
                new SleepAction(ClawBuffer),

                // Score Specimen #6
                new ParallelAction(
                        new OutTakeAction(OuttakeState.SCORING),
                        new VerticalSlideAction(LIFTED),
                        Drive.actionBuilder(new Pose2d(40, -72.2, Math.toRadians(90)))
                                .strafeToConstantHeading(new Vector2d(6, -32))
                                .build()
                ),

                new VerticalSlideAction(RAISED),
                new SleepAction(VerticalSlideBuffer),
//                new ClawAction(OPEN),
//                new SleepAction(ClawBuffer),

                new ParallelAction(
                        new ClawAction(OPEN),
                        new SleepAction(ClawBuffer),
                        new OutTakeAction(OuttakeState.COLLECTION),
                        new VerticalSlideAction(BASE)
                ),






                Drive.actionBuilder(new Pose2d(6, -32, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(40, -72.2))
                        .build(),

                // Grab Specimen
                new ClawAction(CLOSE),
                new SleepAction(ClawBuffer),

                // Score Specimen #5
                new ParallelAction(
                        new OutTakeAction(OuttakeState.SCORING),
                        new VerticalSlideAction(LIFTED),
                        Drive.actionBuilder(new Pose2d(40, -72.2, Math.toRadians(90)))
                                .strafeToConstantHeading(new Vector2d(7.5, -32))
                                .build()
                ),

                new VerticalSlideAction(RAISED),
                new SleepAction(VerticalSlideBuffer),
//                new ClawAction(OPEN),
//                new SleepAction(ClawBuffer),

                new ParallelAction(
                        new ClawAction(OPEN),
                        new SleepAction(ClawBuffer),
                        new OutTakeAction(OuttakeState.COLLECTION),
                        new VerticalSlideAction(BASE)
                ),

                new SleepAction(30)
        ));
    }

    public class OutTakeAction implements Action {
        private final OuttakeState targetState;

        public OutTakeAction(OuttakeState state) {
            this.targetState = state;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setOuttakeState(targetState);
            telemetryPacket.put("Outtake State", targetState.toString());
            return false;
        }

        private void setOuttakeState(OuttakeState state) {
            switch (state) {
                case BASE:
                    robot.scoring.outtakeArmRotation.setPosition(0.00);
                    robot.scoring.clawPrimaryPivot.setPosition(0.00);
                    break;

                case COLLECTION:
                    robot.scoring.outtakeArmRotation.setPosition(0.37);
                    robot.scoring.clawPrimaryPivot.setPosition(0.15);

                    break;

                case SCORING:
                    robot.scoring.outtakeArmRotation.setPosition(0.85);
                    robot.scoring.clawPrimaryPivot.setPosition(0.88);
                    break;

                case REVERSESCORING: // Default
                    robot.scoring.outtakeArmRotation.setPosition(0.37);
                    robot.scoring.clawPrimaryPivot.setPosition(0.35);

                    break;
                case INIT:
                    robot.scoring.outtakeArmRotation.setPosition(0);
                    robot.scoring.clawPrimaryPivot.setPosition(0.2);

            }
        }
    }

    public class ClawAction implements Action {
        private final double targetState;

        public ClawAction(double STATE) {
            this.targetState = STATE;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setClawState(targetState);
            telemetryPacket.put("Claw State", targetState);
            return false;
        }

        private void setClawState(double STATE) {
            robot.scoring.clawStatus.setPosition(STATE);
        }
    }

    public class SweeperAction implements Action {
        private final double targetState;

        public SweeperAction(double STATE) {
            this.targetState = STATE;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setSweeperState(targetState);
            telemetryPacket.put("Sweeper State", targetState);
            return false;
        }

        private void setSweeperState(double STATE) {
            robot.scoring.sweeper.setPosition(STATE);
        }
    }

    public class HorizontalSlideAction implements Action {
        private final int targetPosition;
        private boolean isStarted = false;

        private final double kP = 0.01; // Tune this
        private final double kD = 0.0005; // Tune this
        private int lastError = 0;
        private long lastTime = System.nanoTime();

        public HorizontalSlideAction(int position) {
            this.targetPosition = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isStarted) {
                isStarted = true;
                robot.scoring.horizontalSlideExtension.setTargetPosition(targetPosition);
                robot.scoring.horizontalSlideExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.scoring.horizontalSlideExtension.setPower(1);
            }

            int currentPosition = robot.scoring.horizontalSlideExtension.getCurrentPosition();
            int error = targetPosition - currentPosition;
            int errorDerivative = error - lastError;
            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastTime) / 1e9;

            double velocity = (errorDerivative / deltaTime);
            double acceleration = velocity / deltaTime;

            lastError = error;
            lastTime = currentTime;

            boolean slidesAreBusy = robot.scoring.horizontalSlideExtension.isBusy();

            telemetryPacket.put("Horizontal Slide Target Position", targetPosition);
            telemetryPacket.put("Horizontal Slide Current Position", currentPosition);
            telemetryPacket.put("Horizontal Slide Busy", slidesAreBusy);
            telemetryPacket.put("Horizontal Slide Velocity", velocity);
            telemetryPacket.put("Horizontal Slide Acceleration", acceleration);

            if (!slidesAreBusy) {
                robot.scoring.horizontalSlideExtension.setPower(0.000375);
                robot.scoring.horizontalSlideExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.scoring.horizontalSlideExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return true;
            }
            return false;
        }
    }

    public class VerticalSlideAction implements Action {
        private final int targetPosition;
        private boolean isStarted = false;


        public VerticalSlideAction(int position) {
            this.targetPosition = position;
            this.isStarted = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isStarted) {
                isStarted = true;
                robot.scoring.verticalSlideExtension1.setTargetPosition(targetPosition);
                robot.scoring.verticalSlideExtension2.setTargetPosition(targetPosition);
                robot.scoring.verticalSlideExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.scoring.verticalSlideExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.scoring.verticalSlideExtension1.setPower(1);
                robot.scoring.verticalSlideExtension2.setPower(1);
            }
//
//            int currentPosition1 = robot.scoring.verticalSlideExtension1.getCurrentPosition();
//            int currentPosition2 = robot.scoring.verticalSlideExtension2.getCurrentPosition();
//            int averagePosition = (currentPosition1 + currentPosition2) / 2;
//            int error = targetPosition - averagePosition;
//            int errorDerivative = error - lastError;
//            long currentTime = System.nanoTime();
//            double deltaTime = (currentTime - lastTime) / 1e9;
//
//            double velocity = (errorDerivative / deltaTime);
//            double acceleration = velocity / deltaTime;
//
//            lastError = error;
//            lastTime = currentTime;

            boolean slidesAreBusy = robot.scoring.verticalSlideExtension1.isBusy() || robot.scoring.verticalSlideExtension2.isBusy();
//
//            telemetryPacket.put("Vertical Slide Target Position", targetPosition);
//            telemetryPacket.put("Vertical Slide Current Position", averagePosition);
//            telemetryPacket.put("Vertical Slide Busy", slidesAreBusy);
//            telemetryPacket.put("Vertical Slide Velocity", velocity);
//            telemetryPacket.put("Vertical Slide Acceleration", acceleration);

            if (!slidesAreBusy) {
                robot.scoring.verticalSlideExtension1.setPower(0.000375);
                robot.scoring.verticalSlideExtension2.setPower(0.000375);
                robot.scoring.verticalSlideExtension1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.scoring.verticalSlideExtension2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return true;
            }
            return false;
        }
    }

    public class IntakeAction implements Action {
        private final double targetState;

        public IntakeAction(double STATE) {
            this.targetState = STATE;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setIntakeState(targetState);
            telemetryPacket.put("Intake State", targetState);
            return false;
        }

        private void setIntakeState(double STATE) {
            robot.scoring.intakePivot.setPosition(STATE);
        }
    }
}