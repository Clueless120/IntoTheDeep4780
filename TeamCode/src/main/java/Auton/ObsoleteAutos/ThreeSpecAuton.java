//// Coded By Gurtej Singh
//
//package Auton;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import Robot.Robot;
//
//@Autonomous(name = "63PtAuton", group = "Auton")
//public class ThreeSpecAuton extends LinearOpMode {
//
//    private final Robot robot = new Robot();
//
//    OuttakeState outtakeState = OuttakeState.SCORING;
//    enum OuttakeState {
//        BASE,
//        COLLECTION,
//        SCORING
//    }
//
//    private final int BASE = 0;
//    private final int RISE = 350;
//    private final int RAISED = 750;
//    private final double Power = 1.0;
//    private final double HOLD = 0.0005;
//
//    ClawState clawState = ClawState.CLOSE;
//    enum ClawState {
//        OPEN,
//        CLOSE
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addLine("Initializing...");
//        telemetry.update();
//
//        robot.init(hardwareMap);
//
//        robot.driveTrain.resetDriveTrainEncoders();
//        robot.scoring.resetScoringEncoders();
//
//        telemetry.addLine("\nWaiting for start...");
//        telemetry.update();
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//            // Start Auton
//            robot.driveTrain.runDriveTrainEncoders();
//            robot.scoring.runScoringEncoders();
//
//            while (opModeIsActive()) {
//
//                // Set Robot (Should Have PreLoaded Specimen)
//                setClawState(ClawState.CLOSE);
//                setOuttakeState(OuttakeState.SCORING);
//                slidesToPosition(RISE);
//
//                // RUN 1
//                // Drive To Bar
//                encoderDrive(30, 0.5);
//                encoderDrive(1, 0.25);
//
//                // Buffer
//                sleep(250);
//
//                // Stop Driving, Raise Slides
//                slidesToPosition(RAISED);
//
//                // Open Claw
//                setClawState(ClawState.OPEN);
//
//                // Reset To Collection Position
//                setOuttakeState(OuttakeState.COLLECTION);
//                slidesToPosition(BASE);
//
//                // Drive To Sample Then Collection Zone
//                encoderDrive(-8, 0.625);
//                encoderStrafe(44, 0.5);
//                encoderDrive(30, 0.625);
//                encoderStrafe(16, 0.5);
//                encoderDrive(-50, 0.625);
//
//                // Human Player Buffer
//                sleep(500);
//
//                // Drive To Specimen, Close Claw
//                encoderDrive(-3, 0.5);
//                setClawState(ClawState.CLOSE);
//
//                // Secondary Buffer
//                sleep(500);
//
//                // Reset To Scoring Position
//                encoderDrive(2, 0.5);
//                setOuttakeState(OuttakeState.SCORING);
//                slidesToPosition(RISE);
//
//
//                // RUN 2
//                // Drive To Bar
//                encoderStrafe(-65, 0.5);
//                encoderDrive(30, 0.5);
//                encoderDrive(2, 0.5);
//
//                // Stop Driving, Raise Slides
//                robot.driveTrain.brake();
//                slidesToPosition(RAISED);
//
//                // Open Claw
//                setClawState(ClawState.OPEN);
//
//                // Reset To Collection Position
//                setOuttakeState(OuttakeState.COLLECTION);
//                slidesToPosition(BASE);
//
//                // Drive To Collection Zone (South-East)
//                encoderDrive(-8, 0.5);
//                encoderStrafe(52, 0.5);
//                encoderDrive(-16, 0.5);
//
//                // Human Player Buffer
//                sleep(500);
//
//                // Drive To Specimen, Close Claw
//                encoderDrive(-5, 0.5);
//                setClawState(ClawState.CLOSE);
//
//                // Secondary Buffer
//                sleep(500);
//
//                // Reset To Scoring Position
//                encoderDrive(2, 0.5);
//                setOuttakeState(OuttakeState.SCORING);
//                slidesToPosition(RISE);
//
//                // RUN 3
//                // Drive To Bar
//                encoderStrafe(-50, 0.5);
//                encoderDrive(30, 0.5);
//                encoderDrive(3, 0.5);
//
//                // Stop Driving, Raise Slides
//                robot.driveTrain.brake();
//                slidesToPosition(RAISED);
//
//                // Open Claw
//                setClawState(ClawState.OPEN);
//
//                // Reset To Scoring Position
//                setOuttakeState(OuttakeState.SCORING);
//                slidesToPosition(BASE);
//
//                // Drive To Park
//                encoderDrive(-24, 1);
//                encoderStrafe(64, 1);
//                encoderDrive(-6, 1);
//                robot.driveTrain.brake();
//
//                telemetry.addLine("Autonomous Complete!");
//                telemetry.update();
//
//                sleep(30000);
//            }
//        }
//    }
//
//    private void setOuttakeState(OuttakeState state) {
//        outtakeState = state;
//
//        switch (state) {
//            case BASE:
//                robot.scoring.outtakeArmRotation.setPosition(0.0);
//                robot.scoring.clawPrimaryPivot.setPosition(0.0);
//                break;
//
//            case COLLECTION:
//                robot.scoring.outtakeArmRotation.setPosition(0.38);
//                robot.scoring.clawPrimaryPivot.setPosition(0.05);
//                break;
//
//            case SCORING: // Default
//                robot.scoring.outtakeArmRotation.setPosition(0.80);
//                robot.scoring.clawPrimaryPivot.setPosition(0.75);
//                break;
//        }
//    }
//
//    private void setClawState(ClawState state) {
//        clawState = state;
//
//        switch (state) {
//            case OPEN:
//                robot.scoring.clawStatus.setPosition(0.25);
//                break;
//
//            case CLOSE:
//                robot.scoring.clawStatus.setPosition(1.00);
//                break;
//        }
//    }
//
//    private void slidesToPosition(int position) {
//        robot.scoring.verticalSlideExtension1.setTargetPosition(position);
//        robot.scoring.verticalSlideExtension2.setTargetPosition(position);
//
//        robot.scoring.verticalSlideExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.scoring.verticalSlideExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.scoring.verticalSlideExtension1.setPower(Power);
//        robot.scoring.verticalSlideExtension2.setPower(Power);
//
//        while (opModeIsActive() && (robot.scoring.verticalSlideExtension1.isBusy() || robot.scoring.verticalSlideExtension2.isBusy())) {
//        }
//
//        robot.scoring.verticalSlideExtension1.setPower(HOLD);
//        robot.scoring.verticalSlideExtension2.setPower(HOLD);
//
//        robot.scoring.verticalSlideExtension1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.scoring.verticalSlideExtension2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    // Encoder Based Drive
//    private void encoderDrive(double YInches, double Power) {
//        final double TICKS_PER_REV = 384.5;
//        final double WHEEL_DIAMETER_INCHES = 4.0;
//        final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
//
//        int targetYTicks = (int) (YInches * TICKS_PER_INCH);
//
//        int frontLeftTarget = robot.driveTrain.frontLeft.getCurrentPosition() + targetYTicks;
//        int frontRightTarget = robot.driveTrain.frontRight.getCurrentPosition() + targetYTicks;
//        int backLeftTarget = robot.driveTrain.backLeft.getCurrentPosition() + targetYTicks;
//        int backRightTarget = robot.driveTrain.backRight.getCurrentPosition() + targetYTicks;
//
//        robot.driveTrain.runDriveTrainEncoders();
//
//        robot.driveTrain.frontLeft.setTargetPosition(frontLeftTarget);
//        robot.driveTrain.frontRight.setTargetPosition(frontRightTarget);
//        robot.driveTrain.backLeft.setTargetPosition(backLeftTarget);
//        robot.driveTrain.backRight.setTargetPosition(backRightTarget);
//
//        robot.driveTrain.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (robot.driveTrain.frontLeft.isBusy() && robot.driveTrain.frontRight.isBusy() && robot.driveTrain.backLeft.isBusy() && robot.driveTrain.backRight.isBusy()) {
//            robot.driveTrain.mecDrive(Power, Power, Power, Power);
//        }
//
//        robot.driveTrain.brake();
//        robot.driveTrain.resetDriveTrainEncoders();
//    }
//
//    // Encoder Based Strafe
//    private void encoderStrafe(double XInches, double Power) {
//        final double TICKS_PER_REV = 384.5;
//        final double WHEEL_DIAMETER_INCHES = 4.0;
//        final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
//
//        int targetXTicks = (int) (XInches * TICKS_PER_INCH);
//
//        int frontLeftTarget = robot.driveTrain.frontLeft.getCurrentPosition() + targetXTicks;
//        int frontRightTarget = robot.driveTrain.frontRight.getCurrentPosition() - targetXTicks;
//        int backLeftTarget = robot.driveTrain.backLeft.getCurrentPosition() - targetXTicks;
//        int backRightTarget = robot.driveTrain.backRight.getCurrentPosition() + targetXTicks;
//
//        robot.driveTrain.runDriveTrainEncoders();
//
//        robot.driveTrain.frontLeft.setTargetPosition(frontLeftTarget);
//        robot.driveTrain.frontRight.setTargetPosition(frontRightTarget);
//        robot.driveTrain.backLeft.setTargetPosition(backLeftTarget);
//        robot.driveTrain.backRight.setTargetPosition(backRightTarget);
//
//        robot.driveTrain.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (robot.driveTrain.frontLeft.isBusy() && robot.driveTrain.frontRight.isBusy() && robot.driveTrain.backLeft.isBusy() && robot.driveTrain.backRight.isBusy()) {
//            robot.driveTrain.mecDrive(Power, -Power, -Power, Power);
//        }
//
//        robot.driveTrain.brake();
//        robot.driveTrain.resetDriveTrainEncoders();
//    }
//}