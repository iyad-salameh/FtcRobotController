
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Three Pod Odometry Test", group = "Test")
public class ThreePodOdometryTest extends OpMode {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // goBILDA 4-Bar Odometry Pod specifications
    private static final double WHEEL_DIAMETER_MM = 32.0;
    private static final double TICKS_PER_REV = 2000.0;

    private static final double TICKS_PER_MM =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_MM);

    // Software zero positions
    private int leftZero;
    private int rightZero;
    private int perpendicularZero;

    private boolean previousA = false;

    @Override
    public void init() {

        // Existing robot configuration
        frontLeftDrive = hardwareMap.get(
                DcMotor.class, "frontLeftDrive");

        frontRightDrive = hardwareMap.get(
                DcMotor.class, "frontRightDrive");

        backLeftDrive = hardwareMap.get(
                DcMotor.class, "backLeftDrive");

        backRightDrive = hardwareMap.get(
                DcMotor.class, "backRightDrive");

        // Explicit motor directions from your test setup
        frontLeftDrive.setDirection(
                DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(
                DcMotor.Direction.REVERSE);

        frontRightDrive.setDirection(
                DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(
                DcMotor.Direction.FORWARD);

        // Disable motor encoder-based drive control
        DcMotor[] motors = {
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive
        };

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Let wheels roll while manually moving robot
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetry.addLine("Three Pod Odometry Ready");
        telemetry.addLine("Motor power disabled");
        telemetry.addLine("No IMU used");
        telemetry.update();
    }

    @Override
    public void start() {
        resetSoftwareZero();
    }

    private void resetSoftwareZero() {
        // Encoder port 0 = left parallel pod
        leftZero = frontLeftDrive.getCurrentPosition();

        // Encoder port 3 = right parallel pod
        rightZero = backRightDrive.getCurrentPosition();

        // Encoder port 1 = perpendicular pod
        perpendicularZero =
                frontRightDrive.getCurrentPosition();
    }

    @Override
    public void loop() {

        // Read raw odometry encoder positions
        int leftRaw =
                frontLeftDrive.getCurrentPosition();

        int rightRaw =
                backRightDrive.getCurrentPosition();

        int perpendicularRaw =
                frontRightDrive.getCurrentPosition();

        // Press A once to reset the displayed position
        if (gamepad1.a && !previousA) {
            resetSoftwareZero();
        }
        previousA = gamepad1.a;

        // Calculate ticks relative to software zero
        int leftTicks = leftRaw - leftZero;
        int rightTicks = rightRaw - rightZero;
        int perpendicularTicks =
                perpendicularRaw - perpendicularZero;

        // Convert encoder ticks to millimeters
        double leftMM = leftTicks / TICKS_PER_MM;
        double rightMM = rightTicks / TICKS_PER_MM;
        double perpendicularMM =
                perpendicularTicks / TICKS_PER_MM;

        // Telemetry
        telemetry.addLine("=== RAW ENCODER VALUES ===");

        telemetry.addData("Left Raw", leftRaw);
        telemetry.addData("Right Raw", rightRaw);
        telemetry.addData(
                "Perpendicular Raw", perpendicularRaw);

        telemetry.addLine("=== RELATIVE TICKS ===");

        telemetry.addData("Left Ticks", leftTicks);
        telemetry.addData("Right Ticks", rightTicks);
        telemetry.addData(
                "Perpendicular Ticks", perpendicularTicks);

        telemetry.addLine("=== WHEEL TRAVEL (mm) ===");

        telemetry.addData("Left mm", "%.1f", leftMM);
        telemetry.addData("Right mm", "%.1f", rightMM);
        telemetry.addData(
                "Perpendicular mm", "%.1f", perpendicularMM);

        telemetry.addLine("=== CONTROL ===");
        telemetry.addLine("Press A: Zero readings");
        telemetry.addLine("Manually move robot to test");
        telemetry.addLine("All drive motors stopped");

        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
