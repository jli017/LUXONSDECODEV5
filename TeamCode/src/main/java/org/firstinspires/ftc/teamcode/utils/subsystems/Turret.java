//package org.firstinspires.ftc.teamcode.utils.subsystems;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.seattlesolvers.solverslib.command.SubsystemBase;
//import com.seattlesolvers.solverslib.controller.PIDFController;
//
//import org.firstinspires.ftc.teamcode.utils.Lebruxon;
//import org.firstinspires.ftc.teamcode.utils.Storage;
//
//@Configurable
//public class Turret extends SubsystemBase {
//
//    // =========================
//    // Hardware
//    // =========================
//
//    public CRServo leftServo;
//    public CRServo rightServo;
//    public DcMotorEx encoderMotor;
//
//    // =========================
//    // Encoder / Gearing
//    // =========================
//
//    public static double encoderTicksPerRev = 8192.0;
//    public static double gearRatio = 208.0 / 71.0;
//    public static double ticksPerTurretRev = encoderTicksPerRev * gearRatio;
//    public static double ticksPerRadian = ticksPerTurretRev / (2.0 * Math.PI);
//
//    // =========================
//    // PID Tuning
//    // =========================
//
//    public static double p = 0.8;
//    public static double d = 0.002;
//
//    public static double maxPower = 0.85;
//    public static double toleranceDeg = 0;
//
//    public PIDFController controller = new PIDFController(p, 0, d, 0);
//
//    // ===================================
//    // Hard Limits (Mapped 0 to 2PI Space)
//    // ===================================
//
//    // Safe Travel Region: Side A [0°, 240°] and Side B [290°, 360°]
//    // Prohibited Deadzone Region: (240°, 290°)
//    private static final double LOWER_DEADZONE = Math.toRadians(240.0);
//    private static final double UPPER_DEADZONE = Math.toRadians(290.0);
//
//    public static double deadzoneMarginDeg = 0.5;
//    private static double LOWER_HOLD;
//    private static double UPPER_HOLD;
//
//    // =========================
//    // Runtime State
//    // =========================
//
//    public static double homePos = 0.0;
//    public boolean enableAim = true;
//    public boolean AUTOenableAim = true;
//
//    private double currentTargetAngle = homePos;
//    private double lastError = 0.0;
//    private double lastSafeAngle = homePos;
//    private boolean wasInDeadzone = false;
//
//    // =========================
//    // Encoder Offset
//    // =========================
//
//    // Applied to getCurrentPosition() so the turret's logical zero is consistent
//    // across re-inits. Set in the constructor from Storage so auto→teleop works.
//    //
//    // Formula: rawTicks - encoderOffset = ticks relative to logical zero.
//    // When saving: encoderOffset = rawTicks - (savedAngleRad * ticksPerRadian).
//    private int encoderOffset = 0;
//
//    // =========================
//    // Constructor
//    // =========================
//
//    public Turret(HardwareMap hMap) {
//
//        leftServo  = hMap.get(CRServo.class, "turretLeft");
//        rightServo = hMap.get(CRServo.class, "turretRight");
//        encoderMotor = hMap.get(DcMotorEx.class, "intake");
//
//        encoderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftServo.setDirection(CRServo.Direction.REVERSE);
//        rightServo.setDirection(CRServo.Direction.REVERSE);
//
//        controller.setTolerance(Math.toRadians(toleranceDeg));
//        controller.reset();
//
//        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg); // 235°
//        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg); // 295°
//
//        // Restore absolute position from Storage.
//        //
//        // Storage.turretEncoderSnapshot = raw ticks when Storage.turretAngle was saved.
//        // Storage.turretAngle           = normalized angle (rad) at that moment.
//        //
//        // We want: (rawNow - offset) / ticksPerRadian == savedAngle  (mod 2PI)
//        // So:       offset = rawNow - savedAngle * ticksPerRadian
//        //
//        // This holds as long as the encoder hasn't been physically moved since the
//        // snapshot (which is true across the auto→teleop transition on an FTC robot
//        // because the hub stays powered during the init phase).
//        int rawNow = encoderMotor.getCurrentPosition();
//        int ticksForSavedAngle = (int) Math.round(Storage.turretAngle * ticksPerRadian);
//        encoderOffset = rawNow - ticksForSavedAngle;
//
//        // Seed lastSafeAngle from Storage so the deadzone recovery side is correct
//        // on the very first tick after re-init.
//        lastSafeAngle = Storage.turretAngle;
//    }
//
//    // =========================
//    // Snapshot (call at end of auto before OpMode stops)
//    // =========================
//
//    /**
//     * Saves the current absolute turret angle and raw encoder position into Storage
//     * so the next OpMode (teleop) can reconstruct the offset and continue tracking
//     * from the correct position.
//     *
//     * Call this from Lebruxon.reset() or at the very end of the auto sequence.
//     */
//    public void saveToStorage() {
//        Storage.turretAngle          = getNormalizedAngle();
//        Storage.turretEncoderSnapshot = encoderMotor.getCurrentPosition();
//    }
//
//    // =========================
//    // Update
//    // =========================
//
//    public void update() {
//
//        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg);
//        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg);
//
//        double normalizedPos = getNormalizedAngle();
//
//        // Continuously persist so the value is fresh if the OpMode stops unexpectedly.
//        Storage.turretAngle = normalizedPos;
//
//        if (normalizedPos <= LOWER_DEADZONE || normalizedPos >= UPPER_DEADZONE) {
//            lastSafeAngle = normalizedPos;
//        }
//
//        // ====================================================================
//        // 2. Resolve Target Angle
//        // ====================================================================
//        boolean inDeadzone = normalizedPos > LOWER_DEADZONE && normalizedPos < UPPER_DEADZONE;
//
//        if (inDeadzone) {
//            currentTargetAngle = (lastSafeAngle <= LOWER_DEADZONE) ? LOWER_HOLD : UPPER_HOLD;
//
//            // Only reset lastError on the FIRST tick entering the deadzone.
//            // Resetting every tick caused a D-term spike on every exit.
//            if (!wasInDeadzone) {
//                lastError = 0.0;
//            }
//
//        } else if (enableAim || AUTOenableAim) {
//            Pose robotPose = Lebruxon.drivetrain.follower.getPose();
//            double dx = Lebruxon.goal.getX() - robotPose.getX();
//            double dy = Lebruxon.goal.getY() - robotPose.getY();
//
//            double fieldTargetAngle = wrapToTwoPi(Math.atan2(dy, dx));
//            double robotHeading     = wrapToTwoPi(Lebruxon.drivetrain.follower.getHeading());
//            double normalizedTarget = wrapToTwoPi(fieldTargetAngle - robotHeading);
//
//            if (normalizedTarget > LOWER_DEADZONE && normalizedTarget < UPPER_DEADZONE) {
//                currentTargetAngle = (normalizedPos <= LOWER_DEADZONE) ? LOWER_HOLD : UPPER_HOLD;
//            } else {
//                currentTargetAngle = normalizedTarget;
//            }
//
//
//        } else {
//            currentTargetAngle = homePos;
//        }
//
//        wasInDeadzone = inDeadzone;
//
//        // ====================================================================
//        // 3. Virtual Linear Unrolling & Deadzone Blocking
//        // ====================================================================
//        double shiftedCurrent = wrapToTwoPi(normalizedPos     - UPPER_DEADZONE);
//        double shiftedTarget  = wrapToTwoPi(currentTargetAngle - UPPER_DEADZONE);
//
//        double error = shiftedTarget - shiftedCurrent;
//
//        if (error >  LOWER_DEADZONE) error -= 2.0 * Math.PI;
//        if (error < -LOWER_DEADZONE) error += 2.0 * Math.PI;
//
//        // ====================================================================
//        // 4. PD Output
//        // ====================================================================
//        double toleranceRad = Math.toRadians(toleranceDeg);
//        double derivative   = error - lastError;
//
//        boolean atHoldPos = (Math.abs(normalizedPos - LOWER_HOLD) < toleranceRad)
//                || (Math.abs(normalizedPos - UPPER_HOLD) < toleranceRad);
//        if (atHoldPos) derivative = 0;
//
//        lastError = error;
//
//        double power        = p * error + d * derivative;
//        double clampedPower = clamp(power, -maxPower, maxPower);
//
//        leftServo.setPower(clampedPower);
//        rightServo.setPower(clampedPower);
//
//        // Sync PIDFController for external atSetPoint() reads.
//        controller.setP(p);
//        controller.setD(d);
//        controller.setTolerance(toleranceRad);
//        controller.setSetPoint(currentTargetAngle);
//        controller.calculate(normalizedPos);
//    }
//
//    // =========================
//    // Public Accessors
//    // =========================
//
//    /**
//     * Returns the encoder position corrected by the stored offset and normalized
//     * to [0, 2PI). The offset is applied so that the logical zero matches the
//     * angle that was saved in Storage at the end of the previous OpMode.
//     */
//    public double getNormalizedAngle() {
//        int correctedTicks = encoderMotor.getCurrentPosition() - encoderOffset;
//        double rawRad = correctedTicks / ticksPerRadian;
//        return wrapToTwoPi(rawRad);
//    }
//
//    public double getAngle() {
//        return getNormalizedAngle();
//    }
//
//    public double getTargetAngle() {
//        return currentTargetAngle;
//    }
//
//    // =========================
//    // Utility
//    // =========================
//
//    public static double wrapToTwoPi(double radians) {
//        double wrapped = radians % (2.0 * Math.PI);
//        if (wrapped < 0) wrapped += 2.0 * Math.PI;
//        return wrapped;
//    }
//
//    private static double clamp(double val, double min, double max) {
//        return Math.max(min, Math.min(max, val));
//    }
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

    // Shift frame origin to deadzone midpoint so the 0/2PI wrap seam sits
    // inside the deadzone — a region the turret never occupies — eliminating
    // the jitter that happened when normalizedPos flickered across 0°/360°.
    private static final double FRAME_SHIFT = Math.toRadians(265.0);

    // Pre-shifted deadzone boundaries used for routing checks.
    private static final double SHIFTED_LOWER = wrapToTwoPi(LOWER_DEADZONE - FRAME_SHIFT);
    private static final double SHIFTED_UPPER = wrapToTwoPi(UPPER_DEADZONE - FRAME_SHIFT);

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

    // Latched on deadzone entry. True = came from lower side (<=240°).
    public boolean approachingFromLower = true;

    // Hysteresis: enter latch at LOWER/UPPER_DEADZONE, exit only once the
    // turret has fully cleared back past LOWER_HOLD or UPPER_HOLD.
    public boolean inDeadzoneLatch = false;

    // =========================
    // Encoder Offset
    // =========================

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

        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg); // ~235°
        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg); // ~295°

        int rawNow = encoderMotor.getCurrentPosition();
        int ticksForSavedAngle = (int) Math.round(Storage.turretAngle * ticksPerRadian);
        encoderOffset = rawNow - ticksForSavedAngle;

        approachingFromLower = Storage.turretAngle <= LOWER_DEADZONE;
        inDeadzoneLatch = Storage.turretAngle > LOWER_DEADZONE
                && Storage.turretAngle < UPPER_DEADZONE;
    }

    // =========================
    // Snapshot
    // =========================

    public void saveToStorage() {
        Storage.turretAngle           = getNormalizedAngle();
        Storage.turretEncoderSnapshot = encoderMotor.getCurrentPosition();
    }

    // =========================
    // Update
    // =========================

//    public void update() {
//
//        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg);
//        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg);
//
//        double normalizedPos = getNormalizedAngle();
//        Storage.turretAngle = normalizedPos;
//
//        // ====================================================================
//        // 1. Hysteresis deadzone detection
//        //
//        // Enter latch the moment we cross into (240°, 290°).
//        // Exit latch only once fully clear of LOWER_HOLD or UPPER_HOLD.
//        // ====================================================================
//        if (!inDeadzoneLatch) {
//            if (normalizedPos > LOWER_DEADZONE && normalizedPos < UPPER_DEADZONE) {
//                inDeadzoneLatch = true;
//                approachingFromLower = (normalizedPos - LOWER_DEADZONE)
//                        < (UPPER_DEADZONE - normalizedPos);
//                lastError = 0.0;
//            }
//        } else {
//            if (approachingFromLower && normalizedPos <= LOWER_HOLD) {
//                inDeadzoneLatch = false;
//            } else if (!approachingFromLower && normalizedPos >= UPPER_HOLD) {
//                inDeadzoneLatch = false;
//            }
//        }
//
//        // ====================================================================
//        // 2. Resolve Target Angle
//        // ====================================================================
//
//        if (inDeadzoneLatch) {
//            currentTargetAngle = approachingFromLower ? LOWER_HOLD : UPPER_HOLD;
//
//        } else if (enableAim || AUTOenableAim) {
//            double dx, dy;
//
//            Pose robotPose = Lebruxon.drivetrain.follower.getPose();
//
//            if (Lebruxon.shooter.distance > 100) {
//                dx = Lebruxon.targetFar.getX() - robotPose.getX();
//                dy = Lebruxon.targetFar.getY() - robotPose.getY();
//            }
//            else {
//                dx = Lebruxon.targetClose.getX() - robotPose.getX();
//                dy = Lebruxon.targetClose.getY() - robotPose.getY();
//            }
//
//            double fieldTargetAngle = wrapToTwoPi(Math.atan2(dy, dx));
//            double robotHeading     = wrapToTwoPi(Lebruxon.drivetrain.follower.getHeading());
//            double normalizedTarget = wrapToTwoPi(fieldTargetAngle - robotHeading);
//
//            if (normalizedTarget > LOWER_DEADZONE && normalizedTarget < UPPER_DEADZONE) {
//                double distToLower = normalizedTarget - LOWER_DEADZONE;
//                double distToUpper = UPPER_DEADZONE - normalizedTarget;
//                currentTargetAngle = (distToLower <= distToUpper) ? LOWER_HOLD : UPPER_HOLD;
//            } else {
//                currentTargetAngle = normalizedTarget;
//            }
//
//        } else {
//            currentTargetAngle = homePos;
//        }
//
//        // ====================================================================
//        // 3. Compute error in the shifted frame
//        //
//        // Shift both angles by FRAME_SHIFT (265°) so the wrap seam sits inside
//        // the deadzone. This eliminates the 0/2PI jitter.
//        //
//        // Then, if the shortest-path error in this frame would still route
//        // through the deadzone (SHIFTED_LOWER to SHIFTED_UPPER), force it the
//        // other way. In the shifted frame the deadzone runs from SHIFTED_LOWER
//        // (~335°) to SHIFTED_UPPER (~25°) crossing the 0 point — meaning any
//        // error that is negative and would reach below SHIFTED_UPPER, or
//        // positive and would reach above SHIFTED_LOWER, is going through the
//        // zone and must be flipped.
//        // ====================================================================
//        double shiftedPos    = wrapToTwoPi(normalizedPos      - FRAME_SHIFT);
//        double shiftedTarget = wrapToTwoPi(currentTargetAngle - FRAME_SHIFT);
//
//        double error = shiftedTarget - shiftedPos;
//
//        if (error >  Math.PI) error -= 2.0 * Math.PI;
//        if (error < -Math.PI) error += 2.0 * Math.PI;
//
//        // In the shifted frame the safe region is [SHIFTED_UPPER, SHIFTED_LOWER]
//        // i.e. roughly [25°, 335°]. The deadzone straddles 0° in this frame.
//        // A path going positive from shiftedPos crosses the deadzone if it would
//        // exceed SHIFTED_LOWER. A path going negative crosses if it would drop
//        // below SHIFTED_UPPER. Force the long way in those cases.
//        if (!inDeadzoneLatch) {
//            boolean onLowerShiftedSide = shiftedPos >= SHIFTED_UPPER && shiftedPos <= Math.PI * 2;
//            boolean onUpperShiftedSide = shiftedPos >= 0 && shiftedPos <= SHIFTED_LOWER;
//
//            if (onUpperShiftedSide && error > 0 && (shiftedPos + error) > SHIFTED_LOWER) {
//                error -= 2.0 * Math.PI;
//            } else if (onLowerShiftedSide && error < 0 && (shiftedPos + error) < SHIFTED_UPPER) {
//                error += 2.0 * Math.PI;
//            }
//        }
//
//        // ====================================================================
//        // 4. PD Output
//        // ====================================================================
//        double toleranceRad = Math.toRadians(toleranceDeg);
//        double derivative   = error - lastError;
//
//        boolean atHoldPos = (Math.abs(normalizedPos - LOWER_HOLD) < toleranceRad)
//                || (Math.abs(normalizedPos - UPPER_HOLD) < toleranceRad);
//        if (atHoldPos) derivative = 0;
//
//        lastError = error;
//
//        double power        = p * error + d * derivative;
//        double clampedPower = clamp(power, -maxPower, maxPower);
//
//        leftServo.setPower(clampedPower);
//        rightServo.setPower(clampedPower);
//
//        controller.setP(p);
//        controller.setD(d);
//        controller.setTolerance(toleranceRad);
//        controller.setSetPoint(currentTargetAngle);
//        controller.calculate(normalizedPos);
//    }
    // =========================
    // Update
    // =========================

    public void update() {

        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg);
        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg);

        double normalizedPos = getNormalizedAngle();
        Storage.turretAngle = normalizedPos;

        // ====================================================================
        // 1. Hysteresis deadzone detection
        // ====================================================================
        if (!inDeadzoneLatch) {
            if (normalizedPos > LOWER_DEADZONE && normalizedPos < UPPER_DEADZONE) {
                inDeadzoneLatch = true;
                approachingFromLower = (normalizedPos - LOWER_DEADZONE)
                        < (UPPER_DEADZONE - normalizedPos);
                lastError = 0.0;
            }
        } else {
            if (approachingFromLower && normalizedPos <= LOWER_HOLD) {
                inDeadzoneLatch = false;
            } else if (!approachingFromLower && normalizedPos >= UPPER_HOLD) {
                inDeadzoneLatch = false;
            }
        }

        // ====================================================================
        // 2. Resolve Target Angle
        // ====================================================================
        if (inDeadzoneLatch) {
            currentTargetAngle = approachingFromLower ? LOWER_HOLD : UPPER_HOLD;

        } else if (enableAim) {
            double dx, dy;
            Pose robotPose = Lebruxon.drivetrain.follower.getPose();

            if (Lebruxon.shooter.distance > 100) {
                dx = Lebruxon.targetFar.getX() - robotPose.getX();
                dy = Lebruxon.targetFar.getY() - robotPose.getY();
            } else {
                dx = Lebruxon.targetClose.getX() - robotPose.getX();
                dy = Lebruxon.targetClose.getY() - robotPose.getY();
            }

            double fieldTargetAngle = wrapToTwoPi(Math.atan2(dy, dx));
            double robotHeading     = wrapToTwoPi(Lebruxon.drivetrain.follower.getHeading());
            double normalizedTarget = wrapToTwoPi(fieldTargetAngle - robotHeading);

            if (normalizedTarget > LOWER_DEADZONE && normalizedTarget < UPPER_DEADZONE) {
                double distToLower = normalizedTarget - LOWER_DEADZONE;
                double distToUpper = UPPER_DEADZONE - normalizedTarget;
                currentTargetAngle = (distToLower <= distToUpper) ? LOWER_HOLD : UPPER_HOLD;
            } else {
                currentTargetAngle = normalizedTarget;
            }

        } else {
            currentTargetAngle = homePos;
        }

        // ====================================================================
        // 3. Compute error in the shifted frame
        // ====================================================================
        double shiftedPos    = wrapToTwoPi(normalizedPos      - FRAME_SHIFT);
        double shiftedTarget = wrapToTwoPi(currentTargetAngle - FRAME_SHIFT);

        double error = shiftedTarget - shiftedPos;

        if (error >  Math.PI) error -= 2.0 * Math.PI;
        if (error < -Math.PI) error += 2.0 * Math.PI;

        // Force long-way routing adjustments to avoid deadzone crossing
        if (!inDeadzoneLatch) {
            boolean onLowerShiftedSide = shiftedPos >= SHIFTED_UPPER && shiftedPos <= Math.PI * 2;
            boolean onUpperShiftedSide = shiftedPos >= 0 && shiftedPos <= SHIFTED_LOWER;

            if (onUpperShiftedSide && error > 0 && (shiftedPos + error) > SHIFTED_LOWER) {
                error -= 2.0 * Math.PI;
                lastError -= 2.0 * Math.PI; // FIX: Keep derivative stable!
            } else if (onLowerShiftedSide && error < 0 && (shiftedPos + error) < SHIFTED_UPPER) {
                error += 2.0 * Math.PI;
                lastError += 2.0 * Math.PI; // FIX: Keep derivative stable!
            }
        }

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

        controller.setP(p);
        controller.setD(d);
        controller.setTolerance(toleranceRad);
        controller.setSetPoint(currentTargetAngle);
        controller.calculate(normalizedPos);
    }

    // =========================
    // Public Accessors
    // =========================

    public static double encoderTrim = 0;

    public double getNormalizedAngle() {
        int correctedTicks = encoderMotor.getCurrentPosition() - encoderOffset;
        double rawRad = correctedTicks / ticksPerRadian;
        return wrapToTwoPi(rawRad + encoderTrim);
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
//}