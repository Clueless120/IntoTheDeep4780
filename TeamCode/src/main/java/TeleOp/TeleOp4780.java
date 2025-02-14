package TeleOp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Robot.Robot;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp4780 extends LinearOpMode {

    private final Robot robot = new Robot();

    OuttakeState outtakeState = OuttakeState.COLLECTION;
    private final double intakeLiftedPosition = 0.00;
    private final double intakeLoweredPosition = 0.30;

    boolean lastAButtonState = false;
    boolean lastBButtonState = false;
    boolean lastXButtonState = false;

    enum OuttakeState {BASE, COLLECTION, SCORING}

    RevBlinkinLedDriver.BlinkinPattern currentPattern;
    RevBlinkinLedDriver.BlinkinPattern targetPattern;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.driveTrain.brake();

        robot.driveTrain.resetDriveTrainEncoders();
        robot.scoring.resetScoringEncoders();

        telemetry.addLine("Initialized! Activate Controller(s)...");
        telemetry.update();

        waitForStart();

        double xOffset = robot.driveTrain.IMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        double yOffset = robot.driveTrain.IMU.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

        while (opModeIsActive()) {

            robot.scoring.sweeper.setPosition(0.7);

            robot.driveTrain.runWithoutDriveTrainEncoders();
            robot.scoring.runScoringEncoders();

            // Drive System
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            double theta = Math.atan2(drive, strafe);
            double power = Math.hypot(strafe, drive);
            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double frontLeftPower = power * cos / max - turn;
            double frontRightPower = power * sin / max + turn;
            double backLeftPower = power * sin / max - turn;
            double backRightPower = power * cos / max + turn;

            if (power + Math.abs(turn) > 1) {
                frontLeftPower /= power + Math.abs(turn);
                frontRightPower /= power + Math.abs(turn);
                backLeftPower /= power + Math.abs(turn);
                backRightPower /= power + Math.abs(turn);
            }

            robot.driveTrain.mecDrive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            if (strafe == 0 && drive == 0) {
                robot.driveTrain.brake();
            }

//            // Anti-Tipping Algorithm
//            double xAngle = robot.driveTrain.IMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES) - xOffset;
//            double yAngle = robot.driveTrain.IMU.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES) - yOffset;
//
//            double threshold = 2.5;
//            double correctionFL = 0.0;
//            double correctionFR = 0.0;
//            double correctionBL = 0.0;
//            double correctionBR = 0.0;
//
//            if (Math.abs(xAngle) > threshold) {
//                double pitchCorrection = xAngle > 0 ? 0.75 : -0.75;
//                correctionFL += pitchCorrection;
//                correctionFR += pitchCorrection;
//                correctionBL += pitchCorrection;
//                correctionBR += pitchCorrection;
//            }
//
//            if (Math.abs(yAngle) > threshold) {
//                double rollCorrection = yAngle > 0 ? 0.75 : -0.75;
//                correctionFL += rollCorrection;
//                correctionFR -= rollCorrection;
//                correctionBL -= rollCorrection;
//                correctionBR += rollCorrection;
//            }
//
//            if (correctionFL != 0.0 || correctionFR != 0.0 || correctionBL != 0.0 || correctionBR != 0.0) {
//                robot.driveTrain.mecDrive(correctionFL, correctionFR, correctionBL, correctionBR);
//            }

            // Drive Telemetry
            telemetry.addLine("----- Drive Info -----");
            telemetry.addData("FL Power", "%.2f", frontLeftPower);
            telemetry.addData("FR Power", "%.2f", frontRightPower);
            telemetry.addData("BL Power", "%.2f", backLeftPower);
            telemetry.addData("BR Power", "%.2f", backRightPower);
            telemetry.addLine();
            telemetry.addData("Drive", "%.2f", drive);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("Strafe", "%.2f", strafe);

            telemetry.addLine("\n");


            double kP = 0.01; // Tune this
            double kD = 0.0005; // Tune this

            // Horizontal Slide Extension Control
            double horizontalSlideExtensionPower = 0;
            if (gamepad1.dpad_down) {
                horizontalSlideExtensionPower = -1.0;
            }
            if (gamepad1.dpad_up) {
                horizontalSlideExtensionPower = 1.0;
            }


            int currentHorizontalSlidePosition = robot.scoring.horizontalSlideExtension.getCurrentPosition();
            int horizontalSlideExtensionMax = 2500;  // Max Value
            int horizontalSlideExtensionMin = -50;  // Min Value
            int horizontalToleranceThreshold = 25;  // Threshold Value (Adjust)

//
//            int error = targetPosition - currentPosition;
//            int errorDerivative = error - lastError;
//
//            double HSP = (kP * error) + (kD * errorDerivative);
//            HSP = Math.max(-1, Math.min(1, HSP));


            if (currentHorizontalSlidePosition < horizontalSlideExtensionMax - horizontalToleranceThreshold) {
                double adjustedPower = horizontalSlideExtensionPower * (1 - (double) (currentHorizontalSlidePosition - (horizontalSlideExtensionMax - horizontalToleranceThreshold)) / horizontalToleranceThreshold);
                robot.scoring.horizontalSlideExtension.setPower(Math.min(horizontalSlideExtensionPower, adjustedPower));
            } else if (robot.scoring.horizontalSlideExtension.getCurrentPosition() > horizontalSlideExtensionMin) {
                double adjustedPower = horizontalSlideExtensionPower * (1 - (double) ((horizontalSlideExtensionMin + horizontalToleranceThreshold) - currentHorizontalSlidePosition) / horizontalToleranceThreshold);
                robot.scoring.horizontalSlideExtension.setPower(Math.max(horizontalSlideExtensionPower, adjustedPower));
            } else {
                robot.scoring.horizontalSlideExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.scoring.horizontalSlideExtension.setPower(0);
            }




            telemetry.addLine("----- Horizontal Slide Extension -----");
            telemetry.addData("Power", "%.2f", horizontalSlideExtensionPower);
            telemetry.addData("Position", "%d", robot.scoring.horizontalSlideExtension.getCurrentPosition());

            telemetry.addLine("\n");


            // Intake Roller Control
            boolean manualForward = gamepad1.right_bumper;
            boolean manualReverse = gamepad1.left_bumper;

            if (manualForward) {
                robot.scoring.intakePivot.setPosition(intakeLoweredPosition);
                robot.scoring.rollerInOut.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.scoring.rollerInOut.setPower(1);
            } else if (manualReverse) {
                robot.scoring.intakePivot.setPosition(intakeLiftedPosition);
                robot.scoring.rollerInOut.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.scoring.rollerInOut.setPower(1);
            } else {
                robot.scoring.intakePivot.setPosition(intakeLiftedPosition);
                robot.scoring.rollerInOut.setPower(0);
            }

            String rollerStatus = "Stopped";
            if (robot.scoring.rollerInOut.getPower() > 0) {
                if (robot.scoring.rollerInOut.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    rollerStatus = "Intaking";
                } else if (robot.scoring.rollerInOut.getDirection() == DcMotorSimple.Direction.REVERSE) {
                    rollerStatus = "Outtaking";
                }
            } else {
                rollerStatus = "Stopped";
            }

            telemetry.addLine("----- Roller Info -----");
            telemetry.addData("Intake Roller Status", rollerStatus);
            telemetry.addData("Servo Position", robot.scoring.intakePivot.getPosition());

            telemetry.addLine("\n");


            // Intaked Colored Specimen Indicator
            int Red1 = robot.scoring.colorSensor.red();
            int Green1 = robot.scoring.colorSensor.green();
            int Blue1 = robot.scoring.colorSensor.blue();

            boolean isIntakedYellow = Red1 > 2500 && Green1 > 3000 && Blue1 < 2500;
            boolean isIntakedRed = Red1 > 1500 && Green1 < 3000 && Blue1 < 2000;
            boolean isIntakedBlue = Red1 < 2000 && Green1 < 3000 && Blue1 > 1500;

            if (isIntakedYellow) {
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            } else if (isIntakedBlue) {
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            } else if (isIntakedRed) {
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            } else {
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }

            if (currentPattern != targetPattern) {
                currentPattern = targetPattern;
                robot.scoring.blinkinLedDriver.setPattern(currentPattern);
            }

            telemetry.addLine("----- Color Sensor(s) / LED Info -----");
            telemetry.addData("LED Pattern", currentPattern);
            telemetry.addLine();
            telemetry.addData("Red 1  ", Red1);
            telemetry.addData("Green 1", Green1);
            telemetry.addData("Blue 1 ", Blue1);
            telemetry.addLine();
            telemetry.addData("Intaked Yellow", isIntakedYellow);
            telemetry.addData("Intaked Blue  ", isIntakedBlue);
            telemetry.addData("Intaked Red   ", isIntakedRed);

            telemetry.addLine("\n");


            // Vertical Slide Extension Control
            double verticalSlideExtensionPower = gamepad1.left_trigger - gamepad1.right_trigger;
            int currentVerticalSlidePosition = (robot.scoring.verticalSlideExtension1.getCurrentPosition() + robot.scoring.verticalSlideExtension2.getCurrentPosition()) / 2;
            int verticalSlideExtensionMin = -75;  // Min Value
            int verticalToleranceThreshold = 25;  // Threshold Value (Adjust)
            double slowFactor = 0.75;

            final double HOLD = 0.0005;
            boolean Holding = false;

            if (currentVerticalSlidePosition > 200 && robot.scoring.verticalSlideExtension1.getCurrentPosition() < 1600 && robot.scoring.verticalSlideExtension2.getCurrentPosition() < 1600 && verticalSlideExtensionPower == 0) {
                robot.scoring.verticalSlideExtension1.setPower(HOLD);
                robot.scoring.verticalSlideExtension2.setPower(HOLD);
                Holding = true;
            } else if (verticalSlideExtensionPower != 0 && currentVerticalSlidePosition > verticalSlideExtensionMin) {
                if (currentVerticalSlidePosition <= verticalToleranceThreshold) {
                    double adjustedPower = verticalSlideExtensionPower * slowFactor;
                    robot.scoring.verticalSlideExtension1.setPower(adjustedPower);
                    robot.scoring.verticalSlideExtension2.setPower(adjustedPower);
                } else {
                    robot.scoring.verticalSlideExtension1.setPower(verticalSlideExtensionPower);
                    robot.scoring.verticalSlideExtension2.setPower(verticalSlideExtensionPower);
                }
            } else {
                robot.scoring.verticalSlideExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.scoring.verticalSlideExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.scoring.verticalSlideExtension1.setPower(0);
                robot.scoring.verticalSlideExtension2.setPower(0);
            }

            telemetry.addLine("----- Vertical Slide Extension -----");
            telemetry.addData("Power", "%.2f", verticalSlideExtensionPower);
            telemetry.addData("HOLD Power", Holding ? String.format("%.4f", HOLD) : "Not Holding");
            telemetry.addData("Position", "%d", currentVerticalSlidePosition);

            telemetry.addLine("\n");


            // Outtake System Control
            boolean currentXButtonState = gamepad1.x;
            if (currentXButtonState && !lastXButtonState) {
                if (robot.scoring.clawStatus.getPosition() == 0.75) {
                    robot.scoring.clawStatus.setPosition(0.25); // Open
                } else {
                    robot.scoring.clawStatus.setPosition(0.75); // Close
                }
            }
            lastXButtonState = currentXButtonState;

            boolean currentAButtonState = gamepad1.a;
            if (currentAButtonState && !lastAButtonState) {
                switch (outtakeState) {
                    case BASE:
                        outtakeState = OuttakeState.SCORING;
                        break;

                    case COLLECTION:
                        outtakeState = OuttakeState.SCORING;
                        break;

                    case SCORING:
                        outtakeState = OuttakeState.COLLECTION;
                        break;
                }
            }
            lastAButtonState = currentAButtonState;

            boolean currentBButtonState = gamepad1.b;
            if (currentBButtonState && !lastBButtonState) {
                switch (outtakeState) {
                    case BASE:
                        outtakeState = OuttakeState.COLLECTION;
                        break;

                    case COLLECTION:
                        outtakeState = OuttakeState.BASE;
                        break;

                    case SCORING:
                        outtakeState = OuttakeState.BASE;
                        break;
                }
            }
            lastBButtonState = currentBButtonState;

            switch (outtakeState) {
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
            }

            telemetry.addLine("----- Outtake System Status -----");
            telemetry.addData("Current Case", outtakeState);
            telemetry.addData("Arm Rotation Position", "%.2f", robot.scoring.outtakeArmRotation.getPosition());
            telemetry.addData("Primary Pivot Position", "%.2f", robot.scoring.clawPrimaryPivot.getPosition());

            double clawPosition = robot.scoring.clawStatus.getPosition();
            String clawStatus = clawPosition == 0.25 ? "Open" : "Closed";
            telemetry.addData("Claw Status", clawStatus);
            telemetry.addData("Claw Position", "%.2f", clawPosition);

            telemetry.addLine("\n");

            telemetry.update();
        }
    }
}