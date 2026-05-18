package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.utils.Lebruxon;
import org.firstinspires.ftc.teamcode.utils.Storage;

@Configurable
public class Turret extends SubsystemBase {

    // =========================
    // Hardware
    // =========================

    public CRServo leftServo;
    public CRServo rightServo;
    public DcMotorEx encoderMotor;

    // =========================
    // Encoder / Gearing
    // =========================

    public static double encoderTicksPerRev = 8192.0;

    // turret pulley = 208mm, encoder pulley = 71mm
    public static double gearRatio = 208.0 / 71.0;

    public static double ticksPerTurretRev =
            encoderTicksPerRev * gearRatio;

    public static double ticksPerRadian =
            ticksPerTurretRev / (2.0 * Math.PI);

    // =========================
    // PID Tuning
    // =========================

    public static double p = 0.7;
    public static double d = 0.09;

    public static double kStatic = 0.025;

    public static double maxPower = 0.55;

    public static double toleranceDeg = 1.5;

    public static double minOutput = 0.015;

    public static double powerFilterGain = 0.22;

    public PIDFController controller =
            new PIDFController(p, 0, d, 0);

    // =========================
    // Hard Limits
    // =========================

    // CCW = positive (Pedro standard)
    // MAX_ANGLE: +240° CCW from home
    // MIN_ANGLE: -70°  CW  from home
    private static final double MAX_ANGLE =
            Math.toRadians(240.0);

    private static final double MIN_ANGLE =
            Math.toRadians(-70.0);

    // Small buffer so we detect "at a limit" before
    // physically hitting the hard stop.
    private static final double LIMIT_BUFFER =
            Math.toRadians(5.0);

    // =========================
    // Runtime State
    // =========================

    // Home = 0.0 in continuous encoder space
    // (turret faces same direction as robot heading)
    public static double homePos = 0.0;

    public boolean enableAim = true;
    public boolean AUTOenableAim = true;

    // Whether the turret is currently executing a
    // long-path wrap-around after hitting a limit.
    private boolean isWrapping = false;

    // The last resolved continuous-space setpoint.
    private double continuousTarget = homePos;

    // =========================
    // Constructor
    // =========================

    public Turret(HardwareMap hMap) {

        leftServo =
                hMap.get(CRServo.class, "turretLeft");

        rightServo =
                hMap.get(CRServo.class, "turretRight");

        encoderMotor =
                hMap.get(DcMotorEx.class, "intake");

        encoderMotor.setMode(
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER
        );

        leftServo.setDirection(CRServo.Direction.REVERSE);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        controller.setTolerance(
                Math.toRadians(toleranceDeg)
        );

        controller.reset();
    }

    // =========================
    // Update
    // =========================

    public void update() {

        controller.setP(p);
        controller.setD(d);
        controller.setTolerance(Math.toRadians(toleranceDeg));

        double rawPos = getRawAngle();
        Storage.turretAngle = rawPos;

        // --- 1. Compute desired robot-relative direction ---
        // wrapToPi result only — NOT used as PID setpoint directly.

        double desiredWrapped;

        if (enableAim) {

            Pose robotPose =
                    Lebruxon.drivetrain.follower.getPose();

            double dx =
                    Lebruxon.goal.getX() - robotPose.getX();

            double dy =
                    Lebruxon.goal.getY() - robotPose.getY();

            double fieldTargetAngle =
                    Math.atan2(dy, dx);

            double robotHeading =
                    Lebruxon.drivetrain.follower.getHeading();

            desiredWrapped =
                    wrapToPi(fieldTargetAngle - robotHeading);

        } else {
            desiredWrapped = wrapToPi(homePos);
        }

        // --- 2. Resolve into a legal continuous-space target ---
        continuousTarget =
                resolveTarget(desiredWrapped, rawPos);

        // --- 3. Hard clamp — never command past limits ---
        double safeTarget =
                clamp(continuousTarget, MIN_ANGLE, MAX_ANGLE);

        // --- 4. Run PID in continuous space ---
        controller.setSetPoint(safeTarget);
        double power = controller.calculate(rawPos);
        double clampedPower =
                clamp(power, -maxPower, maxPower);

        leftServo.setPower(clampedPower);
        rightServo.setPower(clampedPower);
    }

    // =========================
    // Target Resolver
    // =========================

    /**
     * Resolves the desired wrapped angle into a legal
     * continuous-space setpoint with the following rules:
     *
     * NORMAL (not wrapping, turret within limits):
     *   - Project desiredWrapped into continuous space as the
     *     candidate closest to currentRaw (short path).
     *   - If that candidate is inside [MIN, MAX]: use it.
     *   - If outside [MIN, MAX]: fall back to home (0.0).
     *     Never cross a limit to chase the goal.
     *
     * WRAPPING (turret physically at or past a limit):
     *   - The turret must spin back the long way.
     *   - Continue auto-tracking: project desiredWrapped to
     *     the far side so the setpoint moves with the goal
     *     while unwinding.
     *   - As soon as the short-path candidate re-enters
     *     [MIN, MAX], exit wrap mode and take the short path.
     */
    private double resolveTarget(double desiredWrapped,
                                 double currentRaw) {

        final double TWO_PI = 2.0 * Math.PI;

        // Short-path candidate: the instance of desiredWrapped
        // closest to currentRaw in continuous space.
        double turns = Math.floor(currentRaw / TWO_PI);
        double near = turns * TWO_PI + desiredWrapped;

        if (near - currentRaw > Math.PI)  near -= TWO_PI;
        if (near - currentRaw < -Math.PI) near += TWO_PI;

        // Check if turret is physically at or past a limit.
        boolean atMaxLimit =
                currentRaw >= MAX_ANGLE - LIMIT_BUFFER;
        boolean atMinLimit =
                currentRaw <= MIN_ANGLE + LIMIT_BUFFER;

        // --- Enter wrap mode if we've hit a limit ---
        if (atMaxLimit || atMinLimit) {
            isWrapping = true;
        }

        // --- Exit wrap mode if short path is legal again ---
        if (isWrapping
                && near >= MIN_ANGLE
                && near <= MAX_ANGLE) {
            isWrapping = false;
        }

        if (isWrapping) {
            // Long-path candidate: one full revolution away
            // in the direction back into the legal zone.
            double far;
            if (atMaxLimit || currentRaw > 0) {
                // Hit the CCW (MAX) limit — wrap CW (subtract 2π)
                far = near - TWO_PI;
            } else {
                // Hit the CW (MIN) limit — wrap CCW (add 2π)
                far = near + TWO_PI;
            }

            // Clamp the far target to legal range so the PID
            // always has a reachable goal during the unwrap.
            return clamp(far, MIN_ANGLE, MAX_ANGLE);
        }

        // --- Normal mode ---
        if (near >= MIN_ANGLE && near <= MAX_ANGLE) {
            return near;
        }

        // Target is out of bounds — return to home.
        return homePos;
    }

    // =========================
    // Public Accessors
    // =========================

    /** Continuous raw angle from encoder. NEVER wrapped. */
    public double getRawAngle() {
        return -encoderMotor.getCurrentPosition()
                / ticksPerRadian;
    }

    public double getAngle() {
        return getRawAngle();
    }

    public double getTargetAngle() {
        return continuousTarget;
    }

    public boolean isWrapping() {
        return isWrapping;
    }

    // =========================
    // Utility
    // =========================

    public static double wrapToPi(double radians) {
        double twoPi = 2.0 * Math.PI;
        radians %= twoPi;
        if (radians <= -Math.PI) radians += twoPi;
        else if (radians > Math.PI) radians -= twoPi;
        return radians;
    }

    private static double clamp(double val,
                                double min,
                                double max) {
        return Math.max(min, Math.min(max, val));
    }
}