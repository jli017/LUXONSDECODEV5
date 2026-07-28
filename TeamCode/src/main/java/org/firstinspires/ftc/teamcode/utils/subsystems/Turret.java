package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.utils.Lebruxon;

@Configurable
public class Turret extends SubsystemBase {

    // =========================
    // Hardware
    // =========================

    public Servo turretServo, turretServo1;

    // =========================
    // Geometry / Range
    // =========================

    // gearRatio = servo shaft degrees per turret degree.
    // 210:210.6818 (turret:servo) -> 210.6818/210.
    public static double gearRatio = 210.6818 / 210.0;
    public static double servoRangeDeg = 355.0;

    // =========================================================================
    // Deadzone -- HARD BLOCK, solved by re-centering instead of path tracking
    // =========================================================================
    //
    // 240deg-290deg (unit-circle, robot-relative) must never be entered.
    //
    // A position servo only has ONE path between any two commandable points,
    // so the trick isn't routing around the deadzone -- it's placing the
    // servo's own zero point so BOTH of its physical hard-stops fall inside
    // the deadzone. Then the deadzone sits at the two extreme ends of the
    // servo's range instead of in the middle of the working range, which
    // makes the legal zone (everything else) one single unbroken interval.
    // Moving between any two points inside a single interval can never
    // leave it -- so a plain clamp is sufficient. No latch, no tracking of
    // "where it last was" needed; every command is safe on its own.
    //
    // servoCenterDeg = the robot-relative angle that maps to servo position
    // 0.5. Chosen as the point exactly opposite the deadzone's midpoint
    // (265deg -> 85deg), so the deadzone lands symmetrically at both ends.
    // Forward (0deg robot-relative) therefore is NOT at position 0.5 -- see
    // angleToServoPos / the constant SERVO_CENTER_DEG below.
    public static double deadzoneLowerDeg = 240.0;
    public static double deadzoneUpperDeg = 290.0;

    private static final double DEADZONE_CENTER_DEG =
            (deadzoneLowerDeg + deadzoneUpperDeg) / 2.0; // 265

    private static final double SERVO_CENTER_DEG =
            wrapTo360(DEADZONE_CENTER_DEG + 180.0); // 85

    // Extra buffer kept clear of the deadzone edges (there's ~22.5deg of
    // slack on each side between the deadzone edge and the servo's true
    // mechanical limit, so this margin eats into that slack rather than
    // needing to touch the hard stop).
    public static double deadzoneMarginDeg = 5.0;

    // =========================
    // Runtime State
    // =========================

    public static double homePos = 0.0; // robot-relative radians, forward
    public boolean enableAim = true;

    // Plain instance field, purely for telemetry -- nothing safety-related
    // depends on this, so it doesn't need to persist across OpModes.
    private double lastCommandedAngle = homePos;

    // =========================
    // Velocity Feedforward ("shoot on the rotate")
    // =========================
    //
    // Compensates for the target's apparent motion caused by the robot's
    // OWN translation and rotation. This is NOT the full "shoot on the
    // move" solution -- it doesn't account for the shot's own flight time
    // vs a laterally-drifting ball, which needs a separate virtual-target
    // offset layered on top of this. This just keeps the turret tracking
    // tightly while the robot is moving/turning.
    public static boolean enableFeedforward = true;

    // TUNE: how long the servo takes to actually reach a commanded step.
    // Bench-test by commanding a step change and timing to ~settled.
    public static double feedforwardLagSeconds = 0.18;

    private final ElapsedTime ffTimer = new ElapsedTime();
    private double lastHeadingRad = 0.0;
    private double lastHeadingTimestampSec = -1.0;

    // Telemetry only.
    public double lastLeadOffsetRad = 0.0;

    // =========================
    // Constructor
    // =========================

    public Turret(HardwareMap hMap) {
        turretServo = hMap.get(Servo.class, "turretLeft");
        turretServo1 = hMap.get(Servo.class, "turretRight");

        // Flip if the mount makes clockwise rotation decrease servo position.
        // turretServo.setDirection(Servo.Direction.REVERSE);

        // Not commanding a position here on purpose -- the hub keeps
        // driving the last PWM signal between OpModes and the Axon's
        // position reference doesn't drift, so the turret is already
        // sitting wherever it physically was. The first update() call
        // issues the first real move once aiming resolves a target.
    }

    // =========================
    // Update
    // =========================

    public void update() {

        double rawTarget; // robot-relative radians

        if (enableAim) {
            double dx, dy;
            Pose robotPose = Lebruxon.drivetrain.follower.getPose();

            if (Lebruxon.shooter.distance > 100) {
                dx = Lebruxon.targetFar.getX() - robotPose.getX();
                dy = Lebruxon.targetFar.getY() - robotPose.getY();
            } else {
                dx = Lebruxon.targetClose.getX() - robotPose.getX();
                dy = Lebruxon.targetClose.getY() - robotPose.getY();
            }

            double fieldTargetAngle = Math.atan2(dy, dx);
            double robotHeading     = Lebruxon.drivetrain.follower.getHeading();
            rawTarget = wrapToPi(fieldTargetAngle - robotHeading);

            if (enableFeedforward) {
                lastLeadOffsetRad = computeLeadOffsetRad(dx, dy, robotHeading);
                rawTarget = wrapToPi(rawTarget + lastLeadOffsetRad);
            } else {
                lastLeadOffsetRad = 0.0;
            }
            // The mapping below stays safe for any input regardless of the
            // lead term's size -- it just clamps harder if the lead pushes
            // the target further out toward the deadzone.

        } else {
            rawTarget = homePos;
        }

        lastCommandedAngle = rawTarget;
        turretServo.setPosition(angleToServoPos(rawTarget));
        turretServo1.setPosition(angleToServoPos(rawTarget));
    }

    // =========================
    // Public Accessors
    // =========================

    public double getTargetAngle() {
        return lastCommandedAngle;
    }

    /**
     * True when a given robot-relative angle falls inside the hard-blocked
     * band.
     */
    public static boolean isAngleInDeadzone(double robotRelativeRad) {
        double deg = wrapTo360(Math.toDegrees(robotRelativeRad));
        return deg > deadzoneLowerDeg && deg < deadzoneUpperDeg;
    }

    /**
     * True when the turret's current commanded target is inside the
     * deadzone -- meaning it's sitting at the nearest legal edge instead of
     * the real target. Shooter logic should check this before firing.
     */
    public boolean isCurrentTargetBlocked() {
        return isAngleInDeadzone(lastCommandedAngle);
    }

    // =========================
    // Velocity Feedforward
    // =========================

    /**
     * Computes a position lead (radians) to add to the raw tracking target,
     * compensating for how much the robot-relative bearing to the goal will
     * change in the time it takes the servo to catch up.
     *
     * @param dx target.x - robotPose.x (field frame)
     * @param dy target.y - robotPose.y (field frame)
     * @param robotHeading current robot heading (field frame, radians)
     */
    private double computeLeadOffsetRad(double dx, double dy, double robotHeading) {
        double r2 = dx * dx + dy * dy;
        if (r2 < 1e-6) return 0.0; // avoid blowing up right on top of the goal

        // Field-relative robot velocity from the path follower.
        // VERIFY: confirm getVelocity() returns field-frame Vector on your
        // Pedro Pathing version -- swap accessors if the API differs.
        com.pedropathing.math.Vector robotVel = Lebruxon.drivetrain.follower.getVelocity();
        double vx = robotVel.getXComponent();
        double vy = robotVel.getYComponent();

        // Line-of-sight sweep rate: how fast the field-relative bearing to
        // a stationary goal changes due to the robot's own translation.
        //   omega_LOS = (dx*vy - dy*vx) / r^2
        double omegaLosField = (dx * vy - dy * vx) / r2;

        // Robot yaw rate via finite difference of heading. Swap for a
        // direct accessor (e.g. an IMU/follower angular-velocity call) if
        // one is available -- it'll be less noisy than differentiating.
        double now = ffTimer.seconds();
        double robotYawRate = 0.0;
        if (lastHeadingTimestampSec >= 0) {
            double dt = now - lastHeadingTimestampSec;
            if (dt > 1e-4) {
                robotYawRate = wrapToPi(robotHeading - lastHeadingRad) / dt;
            }
        }
        lastHeadingRad = robotHeading;
        lastHeadingTimestampSec = now;

        // Rate of change of the ROBOT-RELATIVE target angle (what we
        // actually command) is the field-relative sweep rate minus how
        // fast our own heading is rotating out from under it.
        double requiredAngularVelocity = omegaLosField - robotYawRate;

        return requiredAngularVelocity * feedforwardLagSeconds;
    }

    // =========================
    // Utility
    // =========================

    /**
     * Converts a robot-relative angle into a servo command position.
     * Position 0.5 corresponds to SERVO_CENTER_DEG (85deg), NOT forward --
     * this is what pushes the deadzone out to the two ends of the servo's
     * range instead of the middle. The clamp then keeps every command
     * inside the single legal interval, so crossing the deadzone is not
     * physically producible by any sequence of calls to this method.
     */
    private static double angleToServoPos(double robotRelativeRad) {
        // Stay in TURRET-frame degrees for the offset/clamp math, since
        // SERVO_CENTER_DEG and the deadzone bounds are turret-relative.
        // gearRatio is applied only at the very end, when converting the
        // final turret-frame offset into a servo position delta -- mixing
        // it in earlier caused the clamp to compare mismatched units for
        // any gearRatio != 1.0.
        double targetDeg = Math.toDegrees(robotRelativeRad);
        double offsetDeg = angularDeltaDeg(SERVO_CENTER_DEG, targetDeg);

        double deadzoneWidthDeg = deadzoneUpperDeg - deadzoneLowerDeg;
        double usableHalfRangeDeg = (360.0 - deadzoneWidthDeg) / 2.0 - deadzoneMarginDeg;

        // Also can't exceed what the servo can physically reach, expressed
        // in turret-frame degrees. Matters once gearRatio != 1 -- a large
        // enough ratio could make the servo's own reach the binding
        // constraint instead of the deadzone geometry.
        double turretRangeDeg = servoRangeDeg / gearRatio;
        usableHalfRangeDeg = Math.min(usableHalfRangeDeg, turretRangeDeg / 2.0);

        double clampedOffsetDeg = clamp(offsetDeg, -usableHalfRangeDeg, usableHalfRangeDeg);

        // Convert the turret-frame offset into a servo position delta.
        double servoOffsetDeg = clampedOffsetDeg * gearRatio;
        double pos = 0.5 + servoOffsetDeg / servoRangeDeg;
        return clamp(pos, 0.0, 1.0);
    }

    /** Shortest signed difference (toDeg - fromDeg), wrapped to (-180, 180]. */
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

    public static double wrapToPi(double radians) {
        double wrapped = radians % (2.0 * Math.PI);
        if (wrapped > Math.PI) wrapped -= 2.0 * Math.PI;
        if (wrapped < -Math.PI) wrapped += 2.0 * Math.PI;
        return wrapped;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}