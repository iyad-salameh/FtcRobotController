package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Practice autonomous: forward 2m, left 2m, back 2m, right 2m.
 * The robot NEVER intentionally changes its heading.
 *
 * Three dead-wheel quadrature encoders directly wired to hub encoder ports:
 *    left parallel       -> frontLeftDrive encoder port
 *    right parallel      -> backRightDrive encoder port
 *    perpendicular/side  -> frontRightDrive encoder port
 *
 * The four motors use those same port numbers for power. Motor encoder ports
 * can read separate odometry encoders in RUN_WITHOUT_ENCODER mode.
 * No IMU, Pinpoint, Road Runner, or Pedro Pathing required.
 * FTC Dashboard is used only for development visualization.
 *
 * IMPORTANT: Before a full 2m square, replace SIDE_IN with 20.0/2.54,
 * calibrate all three encoder signs, and confirm all four drive directions.
 */
@Config
@Autonomous(name = "TEST - 3 Pods - 2m Square (No IMU)", group = "Odometry")
public class AutoSquare3Pods extends LinearOpMode {

    // ----------- HARDWARE CONFIGURATION -----------
    // Change these names to match the Driver Station configuration.
    private static final String FRONT_LEFT = "frontLeftDrive";
    private static final String FRONT_RIGHT = "frontRightDrive";
    private static final String BACK_LEFT = "backLeftDrive";
    private static final String BACK_RIGHT = "backRightDrive";

    // ----------- MEASUREMENT / CALIBRATION -----------
    public static double SIDE_IN = 20.0 / 2.54; // SAFE FIRST TEST: 20 cm; change to 200.0/2.54 after calibration

    // EXAMPLES ONLY: replace with your actual pod specifications.
    public static double POD_DIAMETER_IN = 48.0 / 25.4;
    public static double TICKS_PER_REV = 2000.0;

    // Measure center-to-center spacing of LEFT and RIGHT pod contact lines.
    public static double TRACK_WIDTH_IN = 12.0;     // PLACEHOLDER

    // Perpendicular pod wheel's forward offset from robot center:
    // positive = in front of center; negative = behind.
    public static double SIDE_OFFSET_IN = 0.0;     // PLACEHOLDER

    // Adjust EACH sign by manually pushing the robot before allowing Auto:
    // push forward => LEFT and RIGHT corrected deltas must be positive;
    // push left    => SIDE corrected delta must be positive;
    // rotate CCW   => corrected RIGHT delta > corrected LEFT delta.
    public static int LEFT_SIGN = 1;
    public static int RIGHT_SIGN = 1;
    public static int SIDE_SIGN = 1;

    // ----------- MOTION CONTROL TUNING -----------
    public static double MAX_TRANSLATION_POWER = 0.25; // match working low-speed TeleOp for initial validation
    public static double MAX_TURN_POWER = 0.23;
    public static double MIN_TRANSLATION_POWER = 0.12;
    public static double KP_FORWARD = 0.045;   // motor power per inch
    public static double KP_LEFT = 0.050;      // motor power per inch
    public static double KP_HEADING = 1.6;     // motor power per radian

    public static double POSITION_TOLERANCE_IN = 1.5;
    public static double HEADING_TOLERANCE_DEG = 6.0;
    public static double SETTLE_TIME_SEC = 0.18;

    // The official FTC autonomous period is 30 s; stop a little earlier.
    public static double LEG_TIMEOUT_SEC = 8.0;
    public static double TOTAL_TIMEOUT_SEC = 28.0;

    private DcMotor fl, fr, bl, br;
    private ThreePodLocalizer localizer;
    private FtcDashboard dashboard;
    private final ElapsedTime totalClock = new ElapsedTime();
    private final ElapsedTime telemetryClock = new ElapsedTime();

    // Store the driven trail and render it on every new Dashboard packet.
    private final double[] trailX = new double[512];
    private final double[] trailY = new double[512];
    private int trailCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class, FRONT_LEFT);
        fr = hardwareMap.get(DcMotor.class, FRONT_RIGHT);
        bl = hardwareMap.get(DcMotor.class, BACK_LEFT);
        br = hardwareMap.get(DcMotor.class, BACK_RIGHT);

        // Common mecanum setup. If forward is incorrect, adapt THESE
        // directions to your robot before attempting automatic movement.
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{fl, fr, bl, br}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }

        // Actual pod ports per working TeleOp: FL=left (0), BR=right (3), FR=side (1).
        // Do NOT reset motor encoders during an active trajectory.
        localizer = new ThreePodLocalizer(fl, br, fr);
        dashboard = FtcDashboard.getInstance();

        // INIT is a useful time to check raw encoder values by HAND.
        while (opModeInInit()) {
            telemetry.addLine("PUSH ROBOT BY HAND to inspect pod tick changes.");
            telemetry.addData("LEFT raw ticks (FL port 0)", fl.getCurrentPosition());
            telemetry.addData("RIGHT raw ticks (BR port 3)", br.getCurrentPosition());
            telemetry.addData("SIDE raw ticks (FR port 1)", fr.getCurrentPosition());
            telemetry.addData("Planned square side (in)", "%.2f", SIDE_IN);
            telemetry.addLine("Calibrate pod signs and geometry before 2m run.");
            telemetry.update();
            sleep(75);
        }
        if (isStopRequested()) return;

        localizer.reset(); // captures current encoder readings as starting pose
        trailCount = 0;
        addTrailPoint(0, 0);
        totalClock.reset();
        telemetryClock.reset();

        // Robot frame at start: +X = front, +Y = robot's LEFT, +heading = CCW.
        double[][] targets = {
                {SIDE_IN, 0},       // 1: front
                {SIDE_IN, SIDE_IN}, // 2: left
                {0, SIDE_IN},       // 3: back
                {0, 0}             // 4: right, return to origin
        };
        String[] names = {"1/4 FORWARD", "2/4 LEFT", "3/4 BACK", "4/4 RIGHT"};

        boolean success = true;
        String finalState = "SQUARE COMPLETE";
        try {
            for (int i = 0; i < targets.length && opModeIsActive(); i++) {
                if (!driveTo(targets[i][0], targets[i][1], names[i])) {
                    success = false;
                    finalState = "ABORTED at " + names[i] + " (timeout/stall)";
                    break;
                }
            }
            if (!opModeIsActive()) {
                success = false;
                finalState = "STOPPED";
            }
        } finally {
            stopDrive();
            localizer.update();
            publish(finalState, 0, 0, true);
        }

        telemetry.addData("Result", success ? "COMPLETE" : "INCOMPLETE");
        telemetry.addData("Final X (cm)", "%.1f", localizer.x * 2.54);
        telemetry.addData("Final Y (cm)", "%.1f", localizer.y * 2.54);
        telemetry.addData("Final heading (deg)", "%.1f", Math.toDegrees(localizer.heading));
        telemetry.addData("Return error (cm)", "%.1f",
                Math.hypot(localizer.x, localizer.y) * 2.54);
        telemetry.update();
    }

    /** Drives toward an ABSOLUTE pose, with target heading always zero. */
    private boolean driveTo(double targetX, double targetY, String stage) {
        ElapsedTime legClock = new ElapsedTime();
        ElapsedTime settleClock = new ElapsedTime();
        boolean settling = false;
        double distanceAtStart = Math.hypot(targetX - localizer.x,
                targetY - localizer.y);
        double checkX = localizer.x;
        double checkY = localizer.y;
        double lastCheckSec = 0;

        while (opModeIsActive()
                && legClock.seconds() < LEG_TIMEOUT_SEC
                && totalClock.seconds() < TOTAL_TIMEOUT_SEC) {

            localizer.update();
            double ex = targetX - localizer.x;
            double ey = targetY - localizer.y;
            double distance = Math.hypot(ex, ey);
            double headingError = wrap(-localizer.heading); // hold 0 rad

            boolean atTarget = distance <= POSITION_TOLERANCE_IN
                    && Math.abs(Math.toDegrees(headingError)) <= HEADING_TOLERANCE_DEG;

            if (atTarget) {
                stopDrive();
                if (!settling) {
                    settleClock.reset();
                    settling = true;
                }
                if (settleClock.seconds() >= SETTLE_TIME_SEC) {
                    publish(stage + " REACHED", targetX, targetY, true);
                    return true;
                }
            } else {
                settling = false;
                // Rotate global target error into ROBOT axes, no IMU needed.
                double c = Math.cos(localizer.heading);
                double s = Math.sin(localizer.heading);
                double forwardError =  c * ex + s * ey;
                double leftError = -s * ex + c * ey;

                double forward = KP_FORWARD * forwardError;
                double left = KP_LEFT * leftError;

                if (distance <= POSITION_TOLERANCE_IN) {
                    forward = 0;
                    left = 0;
                } else {
                    double translationMagnitude = Math.hypot(forward, left);
                    if (translationMagnitude > MAX_TRANSLATION_POWER) {
                        double scale = MAX_TRANSLATION_POWER / translationMagnitude;
                        forward *= scale;
                        left *= scale;
                    } else if (translationMagnitude > 0
                            && translationMagnitude < MIN_TRANSLATION_POWER) {
                        double scale = MIN_TRANSLATION_POWER / translationMagnitude;
                        forward *= scale;
                        left *= scale;
                    }
                }
                double turnCCW = Range.clip(KP_HEADING * headingError,
                        -MAX_TURN_POWER, MAX_TURN_POWER);
                driveRobotCentric(forward, left, turnCCW);
            }

            // Fail-safe if the odometry is unplugged or the robot is immobile.
            if (legClock.seconds() - lastCheckSec > 1.2) {
                double moved = Math.hypot(localizer.x - checkX,
                        localizer.y - checkY);
                if (distance > 6.0 && moved < 0.35) {
                    stopDrive();
                    publish(stage + " - NO ODOMETRY PROGRESS", targetX, targetY, true);
                    return false;
                }
                checkX = localizer.x;
                checkY = localizer.y;
                lastCheckSec = legClock.seconds();
            }

            // Abort a runaway from clearly wrong signs / drive directions.
            if (legClock.seconds() > 1.0 && distance > distanceAtStart + 8.0) {
                stopDrive();
                publish(stage + " - MOVING AWAY FROM TARGET", targetX, targetY, true);
                return false;
            }

            publish(stage, targetX, targetY, false);
            idle();
        }
        stopDrive();
        publish(stage + " - TIMEOUT", targetX, targetY, true);
        return false;
    }

    /** Positive forward; positive left strafe; positive turn CCW. */
    private void driveRobotCentric(double forward, double left, double turnCCW) {
        double pFL = forward - left - turnCCW;
        double pFR = forward + left + turnCCW;
        double pBL = forward + left - turnCCW;
        double pBR = forward - left + turnCCW;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(pFL), Math.abs(pFR)),
                Math.max(Math.abs(pBL), Math.abs(pBR))));
        fl.setPower(pFL / max);
        fr.setPower(pFR / max);
        bl.setPower(pBL / max);
        br.setPower(pBR / max);
    }

    private void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    /** Driver Station data + robot/target/actual trail on FTC Dashboard. */
    private void publish(String status, double targetX, double targetY, boolean force) {
        if (!force && telemetryClock.milliseconds() < 100) return;
        telemetryClock.reset();
        addTrailPoint(localizer.x, localizer.y);

        telemetry.addData("Stage", status);
        telemetry.addData("X (cm)", "%.1f", localizer.x * 2.54);
        telemetry.addData("Y (cm)", "%.1f", localizer.y * 2.54);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(localizer.heading));
        telemetry.addData("Target (cm)", "%.0f, %.0f", targetX * 2.54,
                targetY * 2.54);
        telemetry.addData("Time (s)", "%.1f", totalClock.seconds());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Stage", status);
        packet.put("X (cm)", localizer.x * 2.54);
        packet.put("Y (cm)", localizer.y * 2.54);
        packet.put("Heading (deg)", Math.toDegrees(localizer.heading));
        packet.put("Return distance (cm)", Math.hypot(localizer.x, localizer.y) * 2.54);
        packet.put("Total time (s)", totalClock.seconds());
        packet.put("Left pod ticks", localizer.getLeftRaw());
        packet.put("Right pod ticks", localizer.getRightRaw());
        packet.put("Side pod ticks", localizer.getSideRaw());

        // Dashboard field is 144 x 144 inches and centered at (0,0).
        // Visually center THIS SQUARE so all four corners stay on the map.
        // This is a test-map offset, NOT a real alliance field start pose.
        double offset = SIDE_IN / 2.0;
        Canvas canvas = packet.fieldOverlay();
        canvas.setStroke("#888888").setStrokeWidth(2)
                .strokeRect(-offset, -offset, SIDE_IN, SIDE_IN);

        if (trailCount >= 2) {
            double[] px = new double[trailCount];
            double[] py = new double[trailCount];
            for (int i = 0; i < trailCount; i++) {
                px[i] = trailX[i] - offset;
                py[i] = trailY[i] - offset;
            }
            canvas.setStroke("#1D9D74").setStrokeWidth(2)
                    .strokePolyline(px, py);
        }

        double rx = localizer.x - offset;
        double ry = localizer.y - offset;
        canvas.setStroke("#1169BB").setStrokeWidth(2).strokeCircle(rx, ry, 8);
        canvas.setStroke("#EC752B").setStrokeWidth(3).strokeLine(
                rx, ry,
                rx + 12 * Math.cos(localizer.heading),
                ry + 12 * Math.sin(localizer.heading));
        canvas.setStroke("#D33535").setStrokeWidth(2)
                .strokeCircle(targetX - offset, targetY - offset, 3);
        dashboard.sendTelemetryPacket(packet);
    }

    private void addTrailPoint(double x, double y) {
        if (trailCount > 0 && Math.hypot(x - trailX[trailCount - 1],
                y - trailY[trailCount - 1]) < 0.75) return;
        if (trailCount == trailX.length) {
            System.arraycopy(trailX, 1, trailX, 0, trailX.length - 1);
            System.arraycopy(trailY, 1, trailY, 0, trailY.length - 1);
            trailCount--;
        }
        trailX[trailCount] = x;
        trailY[trailCount] = y;
        trailCount++;
    }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    /** Direct 3-dead-wheel localizer; no gyroscope or Pinpoint computer. */
    private static class ThreePodLocalizer {
        private final DcMotor left, right, side;
        private int previousL, previousR, previousS;
        double x, y, heading;

        ThreePodLocalizer(DcMotor left, DcMotor right, DcMotor side) {
            this.left = left;
            this.right = right;
            this.side = side;
            reset();
        }

        void reset() {
            previousL = left.getCurrentPosition();
            previousR = right.getCurrentPosition();
            previousS = side.getCurrentPosition();
            x = y = heading = 0;
        }

        int getLeftRaw()  { return left.getCurrentPosition(); }
        int getRightRaw() { return right.getCurrentPosition(); }
        int getSideRaw()  { return side.getCurrentPosition(); }

        void update() {
            int nowL = left.getCurrentPosition();
            int nowR = right.getCurrentPosition();
            int nowS = side.getCurrentPosition();

            // Positive left/right = forward, positive side = robot-left.
            double distancePerTick = Math.PI * POD_DIAMETER_IN / TICKS_PER_REV;
            double dL = (nowL - previousL) * LEFT_SIGN * distancePerTick;
            double dR = (nowR - previousR) * RIGHT_SIGN * distancePerTick;
            double dS = (nowS - previousS) * SIDE_SIGN * distancePerTick;

            previousL = nowL;
            previousR = nowR;
            previousS = nowS;

            // Assumes left/right pods straddle the robot center symmetrically.
            double dTheta = (dR - dL) / TRACK_WIDTH_IN; // CCW positive
            double forward = (dL + dR) / 2.0;
            double sideways = dS - SIDE_OFFSET_IN * dTheta;

            // Exact SE(2) integration over this encoder sample (safe near 0).
            double sinc;
            double cosc;
            if (Math.abs(dTheta) < 1e-5) {
                sinc = 1.0 - dTheta * dTheta / 6.0;
                cosc = dTheta / 2.0;
            } else {
                sinc = Math.sin(dTheta) / dTheta;
                cosc = (1.0 - Math.cos(dTheta)) / dTheta;
            }
            double bodyDX = sinc * forward - cosc * sideways;
            double bodyDY = cosc * forward + sinc * sideways;

            double c = Math.cos(heading);
            double s = Math.sin(heading);
            x += c * bodyDX - s * bodyDY;
            y += s * bodyDX + c * bodyDY;
            heading = wrap(heading + dTheta);
        }
    }
}
