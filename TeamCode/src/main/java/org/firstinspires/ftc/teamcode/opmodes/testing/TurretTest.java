package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Turret Positional Aim Test", group = "Testing")
public class TurretTest extends OpMode {

    // =========================================
    // Dashboard Editable Robot Pose
    // =========================================

    public static double ROBOT_X       = 56;
    public static double ROBOT_Y       = 7;
    public static double ROBOT_HEADING = 180; // degrees
    public static double GOAL_X        = 15;
    public static double GOAL_Y        = 141;

    // =========================================
    // Hardware
    // =========================================
    //
    // ASSUMPTION: two Axon positional servos under "turretLeft" /
    // "turretRight", mirroring the CRServo pair in the old TurretTest.
    // Rename these hardwareMap.get() keys if your actual config differs.

    public Servo turretLeft;
    public Servo turretRight;

    // =========================================
    // Panels: live servo-direction toggles
    // =========================================
    //
    // Flip either from the dashboard instead of editing/redeploying code.
    // Applied immediately on change -- no re-init needed mid-test.

    public static boolean reverseLeftServo  = false;
    public static boolean reverseRightServo = false;

    public static double startPos = 0.260;

    // =========================================
    // Geometry / Range (mirrors Turret.java)
    // =========================================

    // gearRatio = servo shaft degrees per turret degree.
    // 210:210.6818 (turret:servo) -> 210.6818/210.
    public static double gearRatio     = 210.6818 / 210.0;
    public static double servoRangeDeg = 355.0;

    public static double deadzoneLowerDeg  = 240.0;
    public static double deadzoneUpperDeg  = 290.0;
    public static double deadzoneMarginDeg = 5.0;

    // =========================================
    // Debug
    // =========================================

    private double  dbgFieldTargetDeg = 0;
    private double  dbgRelativeDeg    = 0;
    private double  dbgServoPos       = 0;
    private boolean dbgBlocked        = false;

    private boolean lastReverseLeft  = false;
    private boolean lastReverseRight = false;

    // =========================================
    // Init
    // =========================================

    @Override
    public void init() {
        turretLeft  = hardwareMap.get(Servo.class, "turretLeft");
        turretRight = hardwareMap.get(Servo.class, "turretRight");

        applyDirections();
        lastReverseLeft  = reverseLeftServo;
        lastReverseRight = reverseRightServo;

        turretLeft.setPosition(startPos);
        turretRight.setPosition(startPos);
    }

    // =========================================
    // Loop
    // =========================================

    @Override
    public void loop() {

        // Live panels toggle for either servo's direction.
        if (reverseLeftServo != lastReverseLeft || reverseRightServo != lastReverseRight) {
            applyDirections();
            lastReverseLeft  = reverseLeftServo;
            lastReverseRight = reverseRightServo;
        }

        // =========================================
        // Field-centric target -- same math as TurretTest
        // =========================================

        double robotHeadingRad = Math.toRadians(ROBOT_HEADING);
        double dx = GOAL_X - ROBOT_X;
        double dy = GOAL_Y - ROBOT_Y;

        double fieldTargetAngle = Math.atan2(dy, dx);
        double relativeAngle    = wrapToPi(fieldTargetAngle - robotHeadingRad);

        // =========================================
        // Positional mapping -- single linear map + clamp.
        //
        // No recovery state machine here: a position servo has exactly one
        // path between any two commandable positions (see Turret.java),
        // so there's nothing to detect or route around. Every frame just
        // computes the target and commands it directly.
        // =========================================

        double servoPos = angleToServoPos(relativeAngle);

        turretLeft.setPosition(servoPos);
        turretRight.setPosition(servoPos);

        dbgFieldTargetDeg = Math.toDegrees(fieldTargetAngle);
        dbgRelativeDeg    = Math.toDegrees(relativeAngle);
        dbgServoPos       = servoPos;
        dbgBlocked        = isAngleInDeadzone(relativeAngle);

        // =========================================
        // Telemetry
        // =========================================

        telemetry.addLine("===== ROBOT =====");
        telemetry.addData("Robot X",       ROBOT_X);
        telemetry.addData("Robot Y",       ROBOT_Y);
        telemetry.addData("Robot Heading", ROBOT_HEADING);

        telemetry.addLine();

        telemetry.addLine("===== GOAL =====");
        telemetry.addData("Goal X", GOAL_X);
        telemetry.addData("Goal Y", GOAL_Y);

        telemetry.addLine();

        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Field Target Deg", dbgFieldTargetDeg);
        telemetry.addData("Relative Aim Deg", dbgRelativeDeg);
        telemetry.addData("In Deadzone",      dbgBlocked);
        telemetry.addData("Servo Position",   dbgServoPos);
        telemetry.addData("Reverse Left",     reverseLeftServo);
        telemetry.addData("Reverse Right",    reverseRightServo);

        telemetry.update();
    }

    // =========================================
    // Direction helper (panels-driven)
    // =========================================

    private void applyDirections() {
        turretLeft.setDirection(reverseLeftServo
                ? Servo.Direction.REVERSE
                : Servo.Direction.FORWARD);
        turretRight.setDirection(reverseRightServo
                ? Servo.Direction.REVERSE
                : Servo.Direction.FORWARD);
    }

    // =========================================
    // Positional mapping (mirrors Turret.angleToServoPos)
    // =========================================

    private static double angleToServoPos(double robotRelativeRad) {
        double targetDeg = Math.toDegrees(robotRelativeRad);

        double deadzoneCenterDeg = (deadzoneLowerDeg + deadzoneUpperDeg) / 2.0;
        double servoCenterDeg    = wrapTo360(deadzoneCenterDeg + 180.0);

        double offsetDeg = angularDeltaDeg(servoCenterDeg, targetDeg);

        double deadzoneWidthDeg   = deadzoneUpperDeg - deadzoneLowerDeg;
        double usableHalfRangeDeg = (360.0 - deadzoneWidthDeg) / 2.0 - deadzoneMarginDeg;

        double turretRangeDeg = servoRangeDeg / gearRatio;
        usableHalfRangeDeg = Math.min(usableHalfRangeDeg, turretRangeDeg / 2.0);

        double clampedOffsetDeg = clamp(offsetDeg, -usableHalfRangeDeg, usableHalfRangeDeg);

        double servoOffsetDeg = clampedOffsetDeg * gearRatio;
        double pos = 0.5 + servoOffsetDeg / servoRangeDeg;
        return clamp(pos, 0.0, 1.0);
    }

    private static boolean isAngleInDeadzone(double robotRelativeRad) {
        double deg = wrapTo360(Math.toDegrees(robotRelativeRad));
        return deg > deadzoneLowerDeg && deg < deadzoneUpperDeg;
    }

    // =========================================
    // Utility
    // =========================================

    private static double angularDeltaDeg(double fromDeg, double toDeg) {
        double diff = (toDeg - fromDeg) % 360.0;
        if (diff > 180.0) diff -= 360.0;
        if (diff <= -180.0) diff += 360.0;
        return diff;
    }

    public static double wrapTo360(double deg) {
        double wrapped = deg % 360.0;
        if (wrapped < 0) wrapped += 360.0;
        return wrapped;
    }

    /** Wraps an angle in radians to (-pi, pi]. */
    public static double wrapToPi(double radians) {
        double twoPi = 2.0 * Math.PI;
        radians %= twoPi;
        if (radians <= -Math.PI) {
            radians += twoPi;
        } else if (radians > Math.PI) {
            radians -= twoPi;
        }
        return radians;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}