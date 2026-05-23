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
    public static double gearRatio = 208.0 / 71.0;
    public static double ticksPerTurretRev = encoderTicksPerRev * gearRatio;
    public static double ticksPerRadian = ticksPerTurretRev / (2.0 * Math.PI);

    // =========================
    // PID Tuning
    // =========================

    public static double p = 0.8;
    public static double d = 0.002;

    public static double maxPower = 0.85;
    public static double toleranceDeg = 0;

    public PIDFController controller = new PIDFController(p, 0, d, 0);

    // ===================================
    // Hard Limits (Mapped 0 to 2PI Space)
    // ===================================

    // Safe Travel Region: Side A [0°, 240°] and Side B [290°, 360°]
    // Prohibited Deadzone Region: (240°, 290°)
    private static final double LOWER_DEADZONE = Math.toRadians(240.0);
    private static final double UPPER_DEADZONE = Math.toRadians(290.0);

    public static double deadzoneMarginDeg = 0.5;
    private static double LOWER_HOLD;
    private static double UPPER_HOLD;

    // =========================
    // Runtime State
    // =========================

    public static double homePos = 0.0;
    public boolean enableAim = true;
    public boolean AUTOenableAim = true;

    private double currentTargetAngle = homePos;
    private double lastError = 0.0;
    private double lastSafeAngle = homePos;
    private boolean wasInDeadzone = false;

    // =========================
    // Encoder Offset
    // =========================

    // Applied to getCurrentPosition() so the turret's logical zero is consistent
    // across re-inits. Set in the constructor from Storage so auto→teleop works.
    //
    // Formula: rawTicks - encoderOffset = ticks relative to logical zero.
    // When saving: encoderOffset = rawTicks - (savedAngleRad * ticksPerRadian).
    private int encoderOffset = 0;

    // =========================
    // Constructor
    // =========================

    public Turret(HardwareMap hMap) {

        leftServo  = hMap.get(CRServo.class, "turretLeft");
        rightServo = hMap.get(CRServo.class, "turretRight");
        encoderMotor = hMap.get(DcMotorEx.class, "intake");

        encoderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftServo.setDirection(CRServo.Direction.REVERSE);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        controller.setTolerance(Math.toRadians(toleranceDeg));
        controller.reset();

        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg); // 235°
        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg); // 295°

        // Restore absolute position from Storage.
        //
        // Storage.turretEncoderSnapshot = raw ticks when Storage.turretAngle was saved.
        // Storage.turretAngle           = normalized angle (rad) at that moment.
        //
        // We want: (rawNow - offset) / ticksPerRadian == savedAngle  (mod 2PI)
        // So:       offset = rawNow - savedAngle * ticksPerRadian
        //
        // This holds as long as the encoder hasn't been physically moved since the
        // snapshot (which is true across the auto→teleop transition on an FTC robot
        // because the hub stays powered during the init phase).
        int rawNow = encoderMotor.getCurrentPosition();
        int ticksForSavedAngle = (int) Math.round(Storage.turretAngle * ticksPerRadian);
        encoderOffset = rawNow - ticksForSavedAngle;

        // Seed lastSafeAngle from Storage so the deadzone recovery side is correct
        // on the very first tick after re-init.
        lastSafeAngle = Storage.turretAngle;
    }

    // =========================
    // Snapshot (call at end of auto before OpMode stops)
    // =========================

    /**
     * Saves the current absolute turret angle and raw encoder position into Storage
     * so the next OpMode (teleop) can reconstruct the offset and continue tracking
     * from the correct position.
     *
     * Call this from Lebruxon.reset() or at the very end of the auto sequence.
     */
    public void saveToStorage() {
        Storage.turretAngle          = getNormalizedAngle();
        Storage.turretEncoderSnapshot = encoderMotor.getCurrentPosition();
    }

    // =========================
    // Update
    // =========================

    public void update() {

        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg);
        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg);

        double normalizedPos = getNormalizedAngle();

        // Continuously persist so the value is fresh if the OpMode stops unexpectedly.
        Storage.turretAngle = normalizedPos;

        if (normalizedPos <= LOWER_DEADZONE || normalizedPos >= UPPER_DEADZONE) {
            lastSafeAngle = normalizedPos;
        }

        // ====================================================================
        // 2. Resolve Target Angle
        // ====================================================================
        boolean inDeadzone = normalizedPos > LOWER_DEADZONE && normalizedPos < UPPER_DEADZONE;

        if (inDeadzone) {
            currentTargetAngle = (lastSafeAngle <= LOWER_DEADZONE) ? LOWER_HOLD : UPPER_HOLD;

            // Only reset lastError on the FIRST tick entering the deadzone.
            // Resetting every tick caused a D-term spike on every exit.
            if (!wasInDeadzone) {
                lastError = 0.0;
            }

        } else if (enableAim || AUTOenableAim) {
            Pose robotPose = Lebruxon.drivetrain.follower.getPose();
            double dx = Lebruxon.goal.getX() - robotPose.getX();
            double dy = Lebruxon.goal.getY() - robotPose.getY();

            double fieldTargetAngle = wrapToTwoPi(Math.atan2(dy, dx));
            double robotHeading     = wrapToTwoPi(Lebruxon.drivetrain.follower.getHeading());
            double normalizedTarget = wrapToTwoPi(fieldTargetAngle - robotHeading);

            if (normalizedTarget > LOWER_DEADZONE && normalizedTarget < UPPER_DEADZONE) {
                currentTargetAngle = (normalizedPos <= LOWER_DEADZONE) ? LOWER_HOLD : UPPER_HOLD;
            } else {
                currentTargetAngle = normalizedTarget;
            }

        } else {
            currentTargetAngle = homePos;
        }

        wasInDeadzone = inDeadzone;

        // ====================================================================
        // 3. Virtual Linear Unrolling & Deadzone Blocking
        // ====================================================================
        double shiftedCurrent = wrapToTwoPi(normalizedPos     - UPPER_DEADZONE);
        double shiftedTarget  = wrapToTwoPi(currentTargetAngle - UPPER_DEADZONE);

        double error = shiftedTarget - shiftedCurrent;

        if (error >  LOWER_DEADZONE) error -= 2.0 * Math.PI;
        if (error < -LOWER_DEADZONE) error += 2.0 * Math.PI;

        // ====================================================================
        // 4. PD Output
        // ====================================================================
        double toleranceRad = Math.toRadians(toleranceDeg);
        double derivative   = error - lastError;

        boolean atHoldPos = (Math.abs(normalizedPos - LOWER_HOLD) < toleranceRad)
                || (Math.abs(normalizedPos - UPPER_HOLD) < toleranceRad);
        if (atHoldPos) derivative = 0;

        lastError = error;

        double power        = p * error + d * derivative;
        double clampedPower = clamp(power, -maxPower, maxPower);

        leftServo.setPower(clampedPower);
        rightServo.setPower(clampedPower);

        // Sync PIDFController for external atSetPoint() reads.
        controller.setP(p);
        controller.setD(d);
        controller.setTolerance(toleranceRad);
        controller.setSetPoint(currentTargetAngle);
        controller.calculate(normalizedPos);
    }

    // =========================
    // Public Accessors
    // =========================

    /**
     * Returns the encoder position corrected by the stored offset and normalized
     * to [0, 2PI). The offset is applied so that the logical zero matches the
     * angle that was saved in Storage at the end of the previous OpMode.
     */
    public double getNormalizedAngle() {
        int correctedTicks = encoderMotor.getCurrentPosition() - encoderOffset;
        double rawRad = correctedTicks / ticksPerRadian;
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

    public static double wrapToTwoPi(double radians) {
        double wrapped = radians % (2.0 * Math.PI);
        if (wrapped < 0) wrapped += 2.0 * Math.PI;
        return wrapped;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}