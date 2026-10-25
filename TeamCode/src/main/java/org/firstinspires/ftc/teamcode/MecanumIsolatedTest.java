
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@TeleOp(name = "Mecanum Isolated Test", group = "Test")
public class MecanumIsolatedTest extends OpMode {

    // ========================================
    // MOTOR DECLARATIONS
    // ========================================

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // ========================================
    // DRIVING PARAMETERS
    // ========================================

    private static final double MAX_SPEED = 0.25;
    private static final double DEADZONE = 0.10;

    // ========================================
    // NEW: ODOMETRY ZERO VARIABLES
    // ========================================

    private int leftZero = 0;
    private int rightZero = 0;
    private int middleZero = 0;

    private boolean previousA = false;

    // ========================================
    // INITIALIZATION
    // ========================================

    @Override
    public void init() {

        // Motor mapping
        frontLeftDrive = hardwareMap.get(
                DcMotor.class, "frontLeftDrive");

        frontRightDrive = hardwareMap.get(
                DcMotor.class, "frontRightDrive");

        backLeftDrive = hardwareMap.get(
                DcMotor.class, "backLeftDrive");

        backRightDrive = hardwareMap.get(
                DcMotor.class, "backRightDrive");

        // Motor directions
        frontLeftDrive.setDirection(
                DcMotor.Direction.REVERSE);

        backLeftDrive.setDirection(
                DcMotor.Direction.REVERSE);

        frontRightDrive.setDirection(
                DcMotor.Direction.FORWARD);

        backRightDrive.setDirection(
                DcMotor.Direction.FORWARD);

        // Drive motor encoders are not connected.
        // Odometry pods use encoder ports 0, 1, 3.

        frontLeftDrive.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightDrive.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftDrive.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRightDrive.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake when power is zero
        frontLeftDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        backRightDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Isolated Mecanum Ready");
        telemetry.addLine("3-Pod Odometry Enabled");
        telemetry.addLine("No IMU Used");
        telemetry.update();
    }

    // ========================================
    // NEW: INITIAL ODOMETRY ZERO
    // ========================================

    @Override
    public void start() {
        resetOdometryZero();
    }

    // ========================================
    // NEW: RESET ODOMETRY FUNCTION
    // ========================================

    private void resetOdometryZero() {

        // Left pod - Encoder port 0
        leftZero =
                frontLeftDrive.getCurrentPosition();

        // Right pod - Encoder port 3
        rightZero =
                backRightDrive.getCurrentPosition();

        // Middle pod - Encoder port 1
        middleZero =
                frontRightDrive.getCurrentPosition();
    }

    // ========================================
    // MAIN CONTROL LOOP
    // ========================================

    @Override
    public void loop() {

        // Left joystick: forward/backward only
        double forward = deadzone(
                -gamepad1.left_stick_y);

        // Right joystick: rotation only
        double rotate = deadzone(
                gamepad1.right_stick_x);

        // Triggers: strafing
        double strafeLeft = deadzone(
                gamepad1.left_trigger);

        double strafeRight = deadzone(
                gamepad1.right_trigger);

        double right = 0;
        String mode = "STOP";

        // Count active movement controls
        int active = 0;

        if (forward != 0) active++;
        if (rotate != 0) active++;
        if (strafeLeft != 0) active++;
        if (strafeRight != 0) active++;

        // Only one movement at a time
        if (active == 1) {

            if (forward != 0) {
                mode = "FORWARD / BACKWARD";
            }
            else if (rotate != 0) {
                mode = "ROTATION";
            }
            else if (strafeLeft != 0) {
                right = -strafeLeft;
                mode = "STRAFE LEFT";
            }
            else if (strafeRight != 0) {
                right = strafeRight;
                mode = "STRAFE RIGHT";
            }

        } else {

            forward = 0;
            rotate = 0;

            if (active > 1) {
                mode = "MULTIPLE INPUTS BLOCKED";
            }
        }

        // ====================================
        // MECANUM MOTOR CALCULATIONS
        // ====================================

        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        // Normalize motor powers
        double max = Math.max(1.0,
                Math.max(
                        Math.max(Math.abs(fl),
                                Math.abs(fr)),
                        Math.max(Math.abs(bl),
                                Math.abs(br))
                )
        );

        fl = fl / max * MAX_SPEED;
        fr = fr / max * MAX_SPEED;
        bl = bl / max * MAX_SPEED;
        br = br / max * MAX_SPEED;

        // Apply motor powers
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);

        // ====================================
        // DRIVING TELEMETRY
        // ====================================

        telemetry.addData("Mode", mode);
        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Strafe", "%.2f", right);
        telemetry.addData("Rotate", "%.2f", rotate);

        telemetry.addData("FL Power", "%.2f", fl);
        telemetry.addData("FR Power", "%.2f", fr);
        telemetry.addData("BL Power", "%.2f", bl);
        telemetry.addData("BR Power", "%.2f", br);

        // ====================================
        // NEW: THREE-POD ODOMETRY
        // ====================================

        // Read all three encoder positions
        int leftRaw =
                frontLeftDrive.getCurrentPosition();

        int rightRaw =
                backRightDrive.getCurrentPosition();

        int middleRaw =
                frontRightDrive.getCurrentPosition();

        // Press A to reset odometry zero
        if (gamepad1.a && !previousA) {

            leftZero = leftRaw;
            rightZero = rightRaw;
            middleZero = middleRaw;
        }

        previousA = gamepad1.a;

        // Calculate movement relative to zero
        int leftPodTicks = leftRaw - leftZero;
        int rightPodTicks = rightRaw - rightZero;
        int middlePodTicks = middleRaw - middleZero;

        // ====================================
        // ODOMETRY TELEMETRY
        // ====================================

        telemetry.addLine("=== ODOMETRY ===");

        telemetry.addData(
                "Left Pod Ticks", leftPodTicks);

        telemetry.addData(
                "Right Pod Ticks", rightPodTicks);

        telemetry.addData(
                "Middle Pod Ticks", middlePodTicks);

        telemetry.addLine("A = Reset Odometry Zero");

        telemetry.update();
    }

    // ========================================
    // JOYSTICK DEADZONE
    // ========================================

    private double deadzone(double value) {

        if (Math.abs(value) < DEADZONE) {
            return 0;
        }

        return value;
    }

    // ========================================
    // STOP MOTORS
    // ========================================

    @Override
    public void stop() {

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
