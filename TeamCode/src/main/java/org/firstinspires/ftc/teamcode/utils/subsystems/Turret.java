package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    public static double p = 0.6;
    public static double d = 0.003;

    public static double kV = 0.1;

    public static double maxPower = 0.85;
    public static double toleranceDeg = 0;

    public PIDFController controller = new PIDFController(p, 0, d, 0);

    // ===================================
    // Hard Limits (Mapped 0 to 2PI Space)
    // ===================================

    private static final double LOWER_DEADZONE = Math.toRadians(245.0);
    private static final double UPPER_DEADZONE = Math.toRadians(290.0);

    private static final double FRAME_SHIFT = Math.toRadians(265.0);

    private static final double SHIFTED_LOWER = wrapToTwoPi(LOWER_DEADZONE - FRAME_SHIFT);
    private static final double SHIFTED_UPPER = wrapToTwoPi(UPPER_DEADZONE - FRAME_SHIFT);

    public static double deadzoneMarginDeg = 0.5;
    private static double LOWER_HOLD;
    private static double UPPER_HOLD;

    // =========================
    // Shoot-on-the-Move
    // =========================

    public static boolean enableLeadCompensation = true;
    public static double leadMultiplier = 1.0;

    public double lastLeadX = 0.0;
    public double lastLeadY = 0.0;

    // =========================
    // Predictive aim lock
    // =========================

    public boolean lockedAim = false;
    private double lockedFieldAngle = 0.0;

    public void setLockedTarget(double predictedX, double predictedY) {
        double distToGoal = Math.hypot(
                Lebruxon.goalShooter.getX() - predictedX,
                Lebruxon.goalShooter.getY() - predictedY
        );

        com.seattlesolvers.solverslib.geometry.Vector2d target =
                (distToGoal > 100) ? Lebruxon.targetFar : Lebruxon.targetClose;

        double dx = target.getX() - predictedX;
        double dy = target.getY() - predictedY;

        lockedFieldAngle = wrapToTwoPi(Math.atan2(dy, dx));
        lockedAim = true;
    }

    public void clearLockedTarget() {
        lockedAim = false;
    }

    // =========================
    // Runtime State
    // =========================

    public static double homePos = 0.0;

    public boolean enableAim = true;
    private double currentTargetAngle = homePos;
    private double lastError = 0.0;

    public boolean approachingFromLower = true;
    public boolean inDeadzoneLatch = false;

    // =========================
    // Angular-velocity feedforward state
    // =========================

    private final ElapsedTime feedforwardTimer = new ElapsedTime();
    private double lastTargetAngle = homePos;

    public double lastTargetAngularVelocity = 0.0;

    // =========================
    // Manual jog / home-reset control
    // =========================

    public double manualPower = 0.0;
    public static double manualJogPower = 0.5;

    public boolean homeAdjustUnlocked = false;

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

        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg);
        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg);

        int ticksForSavedAngle = (int) Math.round(Storage.turretAngle * ticksPerRadian);
        encoderOffset = Storage.turretEncoderSnapshot - ticksForSavedAngle;

        currentTargetAngle = Storage.turretAngle;
        lastTargetAngle = Storage.turretAngle;
        feedforwardTimer.reset();

        approachingFromLower = Storage.turretAngle <= LOWER_DEADZONE;
        inDeadzoneLatch = Storage.turretAngle > LOWER_DEADZONE
                && Storage.turretAngle < UPPER_DEADZONE;
    }

    // =========================
    // Snapshot
    // =========================

    public void saveToStorage() {
        int correctedTicks = encoderMotor.getCurrentPosition() - encoderOffset;
        double rawAngleNoTrim = wrapToTwoPi(correctedTicks / ticksPerRadian);

        Storage.turretAngle = rawAngleNoTrim;
        Storage.turretEncoderSnapshot = encoderMotor.getCurrentPosition();
        encoderTrim = 0;
    }

    // =========================
    // Manual home reset
    // =========================

    public void setHomeToCurrentPosition() {
        encoderOffset = encoderMotor.getCurrentPosition();
        encoderTrim = 0;
        homePos = 0.0;
        currentTargetAngle = 0.0;
        lastTargetAngle = 0.0;
        lastError = 0.0;
        manualPower = 0.0;
        Storage.turretAngle = 0.0;
        Storage.turretEncoderSnapshot = encoderMotor.getCurrentPosition();
    }

    // =========================
    // Update
    // =========================

    public void update() {

        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg);
        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg);

        double normalizedPos = getNormalizedAngle();

        int rawTicksNow = encoderMotor.getCurrentPosition() - encoderOffset;
        Storage.turretAngle = wrapToTwoPi(rawTicksNow / ticksPerRadian);
        Storage.turretEncoderSnapshot = encoderMotor.getCurrentPosition();

        // ====================================================================
        // 0. Manual jog override
        // ====================================================================
        if (!enableAim && Math.abs(manualPower) > 0.02) {
            double clampedManual = clamp(manualPower, -1.0, 1.0) * manualJogPower;
            leftServo.setPower(-clampedManual);
            rightServo.setPower(-clampedManual);

            currentTargetAngle = normalizedPos;
            lastTargetAngle = normalizedPos;
            lastError = 0.0;
            controller.setSetPoint(currentTargetAngle);
            feedforwardTimer.reset();
            return;
        }

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

        } else if (lockedAim) {
            double robotHeading = wrapToTwoPi(Lebruxon.drivetrain.follower.getHeading());
            double normalizedTarget = wrapToTwoPi(lockedFieldAngle - robotHeading);

            if (normalizedTarget > LOWER_DEADZONE && normalizedTarget < UPPER_DEADZONE) {
                double distToLower = normalizedTarget - LOWER_DEADZONE;
                double distToUpper = UPPER_DEADZONE - normalizedTarget;
                currentTargetAngle = (distToLower <= distToUpper) ? LOWER_HOLD : UPPER_HOLD;
            } else {
                currentTargetAngle = normalizedTarget;
            }

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

            // FIX: only apply shoot-on-the-move lead compensation for close
            // shots. Far shots (distance > 100) now aim straight at
            // targetFar with no lead offset — keeps far shooting on point
            // instead of getting shifted by a lead correction that was
            // apparently not reliable at that range.
            if (enableLeadCompensation && Lebruxon.shooter.distance <= 100) {
                double timeOfFlight = Lebruxon.shooter.getTimeOfFlight(Lebruxon.shooter.distance);
                Vector robotVel = Lebruxon.drivetrain.follower.getVelocity();

                double leadX = robotVel.getXComponent() * timeOfFlight * leadMultiplier;
                double leadY = robotVel.getYComponent() * timeOfFlight * leadMultiplier;

                lastLeadX = leadX;
                lastLeadY = leadY;

                dx -= leadX;
                dy -= leadY;
            } else {
                lastLeadX = 0.0;
                lastLeadY = 0.0;
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
            currentTargetAngle = homeAdjustUnlocked ? normalizedPos : homePos;
        }

        // ====================================================================
        // 2b. Angular-velocity feedforward
        // ====================================================================
        double dt = feedforwardTimer.seconds();
        feedforwardTimer.reset();

        if (dt > 1e-4 && dt < 0.25) {
            double targetDelta = shortestAngleDiff(currentTargetAngle, lastTargetAngle);
            lastTargetAngularVelocity = targetDelta / dt;
        } else {
            lastTargetAngularVelocity = 0.0;
        }
        lastTargetAngle = currentTargetAngle;

        // ====================================================================
        // 3. Compute error in the shifted frame
        // ====================================================================
        double shiftedPos    = wrapToTwoPi(normalizedPos      - FRAME_SHIFT);
        double shiftedTarget = wrapToTwoPi(currentTargetAngle - FRAME_SHIFT);

        double error = shiftedTarget - shiftedPos;

        if (error >  Math.PI) error -= 2.0 * Math.PI;
        if (error < -Math.PI) error += 2.0 * Math.PI;

        if (!inDeadzoneLatch) {
            boolean onLowerShiftedSide = shiftedPos >= SHIFTED_UPPER && shiftedPos <= Math.PI * 2;
            boolean onUpperShiftedSide = shiftedPos >= 0 && shiftedPos <= SHIFTED_LOWER;

            if (onUpperShiftedSide && error > 0 && (shiftedPos + error) > SHIFTED_LOWER) {
                error -= 2.0 * Math.PI;
                lastError -= 2.0 * Math.PI;
            } else if (onLowerShiftedSide && error < 0 && (shiftedPos + error) < SHIFTED_UPPER) {
                error += 2.0 * Math.PI;
                lastError += 2.0 * Math.PI;
            }
        }

        // ====================================================================
        // 4. PD + velocity-feedforward Output
        // ====================================================================
        double toleranceRad = Math.toRadians(toleranceDeg);
        double derivative   = error - lastError;

        boolean atHoldPos = (Math.abs(normalizedPos - LOWER_HOLD) < toleranceRad)
                || (Math.abs(normalizedPos - UPPER_HOLD) < toleranceRad);
        if (atHoldPos) {
            derivative = 0;
        }

        lastError = error;

        double feedforwardPower = kV * lastTargetAngularVelocity;
        double power        = p * error + d * derivative + feedforwardPower;
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

    private static double shortestAngleDiff(double a, double b) {
        double diff = (a - b) % (2.0 * Math.PI);
        if (diff > Math.PI) diff -= 2.0 * Math.PI;
        if (diff < -Math.PI) diff += 2.0 * Math.PI;
        return diff;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}