// Coded By Gurtej Singh
// adb connect 192.168.43.1:5555

package Robot;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class Robot {

    // DriveTrain Subsystem
    public class DriveTrain {
        public DcMotor frontLeft, frontRight, backLeft, backRight;
        public BHI260IMU IMU;

        public void init(HardwareMap hwMap) {
            frontLeft = hwMap.dcMotor.get("frontLeft");    // Config 0
            frontRight = hwMap.dcMotor.get("frontRight");   // Config 1
            backLeft = hwMap.dcMotor.get("backLeft");     // Config 2
            backRight = hwMap.dcMotor.get("backRight");    // Config 3

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            IMU = hwMap.get(BHI260IMU.class, "IMU");
            IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(xyzOrientation(90, 90, -60))));
        }

        public void mecDrive(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }

        public void runDriveTrainEncoders() {
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void runWithoutDriveTrainEncoders() {
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void resetDriveTrainEncoders() {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void brake() {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }


    // Scoring Subsystem
    public class Scoring {
        public DcMotor horizontalSlideExtension, verticalSlideExtension1, verticalSlideExtension2, rollerInOut;
        public Servo intakePivot, outtakeArmRotation, clawPrimaryPivot, clawStatus, sweeper;
        public RevBlinkinLedDriver blinkinLedDriver;
        public ColorSensor colorSensor;

        public void init(HardwareMap hwMap) {
            horizontalSlideExtension = hwMap.dcMotor.get("horizontalSlideExtension");  // Config 0
            horizontalSlideExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            horizontalSlideExtension.setDirection(DcMotorSimple.Direction.FORWARD);
            horizontalSlideExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            verticalSlideExtension1 = hwMap.dcMotor.get("verticalSlideExtension1");    // Config 1
            verticalSlideExtension1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalSlideExtension1.setDirection(DcMotorSimple.Direction.REVERSE);
            verticalSlideExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            verticalSlideExtension2 = hwMap.dcMotor.get("verticalSlideExtension2");    // Config 2
            verticalSlideExtension2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalSlideExtension2.setDirection(DcMotorSimple.Direction.REVERSE);
            verticalSlideExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rollerInOut = hwMap.dcMotor.get("rollerInOut");// Config 3
            rollerInOut.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rollerInOut.setDirection(DcMotorSimple.Direction.FORWARD);
            rollerInOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intakePivot = hwMap.servo.get("intakePivot");                // Config 2 (Control)

            sweeper = hwMap.servo.get("sweeper");// Config 1 (Control)

            outtakeArmRotation = hwMap.servo.get("outtakeArmRotation");  // Config 1 (Expansion)

            clawPrimaryPivot = hwMap.servo.get("clawPrimaryPivot");      // Config 3 (Expansion)

            clawStatus = hwMap.servo.get("clawStatus");                  // Config 4 (Expansion)

            blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "LED");  // Config 0 (Control)
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

            colorSensor = hwMap.get(ColorSensor.class, "colorSensor");       // Config I2C Bus 3 (Control)  (Checks If Sample Is Properly Intaked)
            colorSensor.enableLed(true);
        }

        public void runScoringEncoders() {
            horizontalSlideExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalSlideExtension1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalSlideExtension2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rollerInOut.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void resetScoringEncoders() {
            horizontalSlideExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlideExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlideExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


    // Created Instances of Subsystems
    public DriveTrain driveTrain = new DriveTrain();
    public Scoring scoring = new Scoring();

    // Initialize Robot.Robot Hardware
    public void init(HardwareMap hwMap) {
        driveTrain.init(hwMap);
        scoring.init(hwMap);
    }
}