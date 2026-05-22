package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static double gearRatio = 208.0 / 71.0;
    public static double ticksPerTurretRev = encoderTicksPerRev * gearRatio;
    public static double ticksPerRadian = ticksPerTurretRev / (2.0 * Math.PI);

    // =========================
    // PID Tuning
    // =========================

    public static double p = 0.8;
    public static double d = 0.002;

    // Feedforward to overcome internal CR Servo friction
    public static double maxPower = 0.85;
    public static double toleranceDeg = 0.5;

    public PIDFController controller = new PIDFController(p, 0, d, 0);

    // ===================================
    // Hard Limits (Mapped 0 to 2PI Space)
    // ===================================

    // Safe Travel Region: Side A [0°, 240°] and Side B [290°, 360°]
    // Prohibited Deadzone Region: (240°, 290°)
    // Safe travel arc = 310°. Deadzone arc = 50°.
    private static final double LOWER_DEADZONE = Math.toRadians(240.0);
    private static final double UPPER_DEADZONE = Math.toRadians(290.0);

    // Hold positions 5° inside the safe zone so the turret always has
    // enough error to overcome static friction and never strands itself
    // right on the boundary where P output is near zero.
    public static double deadzoneMarginDeg = 5.0;
    private static double LOWER_HOLD;
    private static double UPPER_HOLD;

    // =========================
    // Runtime State
    // =========================

    public static double homePos = 0.0;
    public boolean enableAim = true;
    public boolean AUTOenableAim = true;

    private double currentTargetAngle = homePos;

    // For manual PD derivative term — avoids using PIDFController's
    // stateful setpoint which can accumulate discontinuities.
    private double lastError = 0.0;

    // Tracks last known safe position so deadzone recovery knows which
    // side the turret entered from and retreats back the same way.
    private double lastSafeAngle = homePos;

    // =========================
    // Constructor
    // =========================

    public Turret(HardwareMap hMap) {

        leftServo = hMap.get(CRServo.class, "turretLeft");
        rightServo = hMap.get(CRServo.class, "turretRight");
        encoderMotor = hMap.get(DcMotorEx.class, "intake");

        encoderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Mirrored configuration so servos don't fight each other mechanically.
        leftServo.setDirection(CRServo.Direction.REVERSE);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        controller.setTolerance(Math.toRadians(toleranceDeg));
        controller.reset();

        // Hold positions are 5° inside the safe zone on each side
        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg); // 235°
        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg); // 295°
    }

    // =========================
    // Update
    // =========================

    public void update() {

        // Recompute hold positions in case deadzoneMarginDeg was tuned at runtime
        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg);
        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg);

        // 1. Get current position normalized strictly between 0 and 2PI
        double normalizedPos = getNormalizedAngle();
        Storage.turretAngle = normalizedPos;

        // Track last known safe position so deadzone clamping knows which
        // side we came from. Only update when outside the deadzone.
        if (normalizedPos <= LOWER_DEADZONE || normalizedPos >= UPPER_DEADZONE) {
            lastSafeAngle = normalizedPos;
        }

        // ====================================================================
        // 2. Resolve Target Angle
        // ====================================================================
        // If physically inside the deadzone, override target to the hold position
        // on the side we entered from and fall through to the normal shift+PD loop.
        // Using LOWER_HOLD / UPPER_HOLD instead of the exact boundary gives the
        // PD controller a non-trivial error so it can overcome static friction and
        // pull the turret back cleanly without stalling right on the boundary.
        if (normalizedPos > LOWER_DEADZONE && normalizedPos < UPPER_DEADZONE) {
            currentTargetAngle = (lastSafeAngle <= LOWER_DEADZONE) ? LOWER_HOLD : UPPER_HOLD;
            lastError = 0;

        } else if (enableAim || AUTOenableAim) {
            Pose robotPose = Lebruxon.drivetrain.follower.getPose();
            double dx = Lebruxon.goalShooter.getX() - robotPose.getX();
            double dy = Lebruxon.goalShooter.getY() - robotPose.getY();

            double fieldTargetAngle = wrapToTwoPi(Math.atan2(dy, dx));

            // Wrap heading to [0, 2PI) so it lives in the same space as everything
            // else. Without this, a sign flip at the ±PI boundary causes a ~360° jump
            // in normalizedTarget even when the robot has barely moved.
            double robotHeading = wrapToTwoPi(Lebruxon.drivetrain.follower.getHeading());

            // Calculate the raw relative target and wrap to [0, 2PI) space
            double normalizedTarget = wrapToTwoPi(fieldTargetAngle - robotHeading);

            // If the computed target falls inside the deadzone, clamp to the hold
            // position on the same side the turret is currently on — never the far edge.
            if (normalizedTarget > LOWER_DEADZONE && normalizedTarget < UPPER_DEADZONE) {
                currentTargetAngle = (normalizedPos <= LOWER_DEADZONE) ? LOWER_HOLD : UPPER_HOLD;
            } else {
                currentTargetAngle = normalizedTarget;
            }

        } else {
            // Default home position when aiming is turned off / unavailable
            currentTargetAngle = homePos;
        }

        // ====================================================================
        // 3. Virtual Linear Unrolling & Deadzone Blocking
        // ====================================================================

        // Shifting our coordinate space by UPPER_DEADZONE (290°) places the
        // forbidden deadzone at the very end of our 0 to 2PI data ribbon.
        // After the shift:
        //   Safe travel zone occupies [0°, 310°]  (310° = 360° - 50° deadzone)
        //   Deadzone sits at          [310°, 360°]
        //
        // Because both valid positions live inside [0°, 310°], the straight-line
        // difference (shiftedTarget - shiftedCurrent) always travels through safe
        // territory. There is exactly ONE valid path between any two points —
        // so we must NOT clamp error to (-180°, 180°), which would re-introduce
        // routes through the deadzone for targets >180° apart in shifted space.
        //
        // Safety guard: if |error| > LOWER_DEADZONE (240°) something upstream
        // failed. Wrap once to recover gracefully rather than slamming the turret.
        double shiftedCurrent = wrapToTwoPi(normalizedPos - UPPER_DEADZONE);
        double shiftedTarget  = wrapToTwoPi(currentTargetAngle - UPPER_DEADZONE);

        // Single-path linear error — no shortest-path clamping to PI.
        double error = shiftedTarget - shiftedCurrent;

        // Safety guard only: under normal operation this branch never fires.
        if (error >  LOWER_DEADZONE) error -= 2.0 * Math.PI;
        if (error < -LOWER_DEADZONE) error += 2.0 * Math.PI;

        // ====================================================================
        // 4. PD Output Control — computed directly on error to avoid PIDFController
        //    internal state corruption from discontinuous setpoints.
        // ====================================================================

        double toleranceRad = Math.toRadians(toleranceDeg);

        double derivative = error - lastError;

        // Zero the derivative when settled at either hold position so the
        // D term does not kick the turret back off the limit on the next tick.
        boolean atHoldPos = (Math.abs(normalizedPos - LOWER_HOLD) < toleranceRad)
                || (Math.abs(normalizedPos - UPPER_HOLD) < toleranceRad);
        if (atHoldPos) {
            derivative = 0;
        }

        lastError = error;

        double power = p * error + d * derivative;

        double clampedPower = clamp(power, -maxPower, maxPower);

        leftServo.setPower(clampedPower);
        rightServo.setPower(clampedPower);

        // Keep controller setpoint in sync for external reads (e.g. telemetry, homePos adjust)
        // Use wrapToTwoPi so getSetPoint() always returns a value in [0, 2PI).
        controller.setP(p);
        controller.setD(d);
        controller.setTolerance(toleranceRad);
        controller.setSetPoint(wrapToTwoPi(normalizedPos + error));
    }

    // =========================
    // Public Accessors
    // =========================

    /**
     * Returns the raw encoder orientation normalized strictly between 0 and 2PI.
     * Handles negative raw positions from reverse rotation flawlessly.
     */
    public double getNormalizedAngle() {
        int adjustedTicks = encoderMotor.getCurrentPosition();
        double rawRad = adjustedTicks / ticksPerRadian;
        return wrapToTwoPi(rawRad);
    }

    public double getAngle() {
        return getNormalizedAngle();
    }

    public double getTargetAngle() {
        return currentTargetAngle;
    }

    // =========================
    // Utility
    // =========================

    /**
     * Wraps any angle into a strict positive range of [0, 2PI)
     */
    public static double wrapToTwoPi(double radians) {
        double wrapped = radians % (2.0 * Math.PI);
        if (wrapped < 0) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}