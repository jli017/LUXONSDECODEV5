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

    public static double p = 0.7;
    public static double d = 0.002;

    // Angular-velocity feedforward gain. A pure P(D) loop only ever reacts to
    // position error, so it structurally lags a moving setpoint (the setpoint
    // moves every loop while we're aiming during translation / lead compensation).
    // kV feeds forward the target's own angular velocity as commanded power so
    // the turret is already trying to match the target's rotation rate instead
    // of only closing the gap after it appears. Start at 0 on the bench, then
    // increase in small steps while sweeping the robot laterally in front of the
    // goal until the turret visibly stops lagging the target without oscillating.
    public static double kV = 0.1;

    public static double maxPower = 0.85;
    public static double toleranceDeg = 0;

    public PIDFController controller = new PIDFController(p, 0, d, 0);

    // ===================================
    // Hard Limits (Mapped 0 to 2PI Space)
    // ===================================

    // Safe Travel Region: Side A [0°, 240°] and Side B [290°, 360°]
    // Prohibited Deadzone Region: (240°, 290°)
    private static final double LOWER_DEADZONE = Math.toRadians(245.0);
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
    // Shoot-on-the-Move
    // =========================

    // Master enable so lead compensation can be killed independently of enableAim
    // (useful for A/B testing accuracy with/without lead during bring-up).
    public static boolean enableLeadCompensation = true;

    // Scales the raw robotVelocity * timeOfFlight lead offset. Start at 1.0 and
    // tune down/up on the field once lutTimeOfFlight values are bench-measured.
    public static double leadMultiplier = 1.0;

    // Last computed lead offset (inches), exposed for logging/telemetry.
    public double lastLeadX = 0.0;
    public double lastLeadY = 0.0;

    // =========================
    // Predictive aim lock (used while intaking, before a shoot window)
    // =========================
    //
    // enableAim tracks the goal LIVE off the robot's current pose every loop,
    // which is correct right before/during a shot but is wasted, noisy motion
    // while just driving around collecting balls. Since auto paths are known
    // ahead of time, the caller (e.g. the auto OpMode) can instead call
    // setLockedTarget() once with the pose we expect to be at for the NEXT
    // shot. That resolves a single fieldTargetAngle up front — no lead
    // compensation, no re-deriving dx/dy from a constantly-changing distance —
    // and update() just keeps reprojecting that fixed field angle through the
    // CURRENT heading every loop (since heading can still change while we
    // drive/intake). Deliberately alliance-agnostic: it only reads from
    // Lebruxon.goalShooter/targetClose/targetFar, which are already resolved
    // per-alliance in Lebruxon.init() — no mirroring math lives here.
    public boolean lockedAim = false;
    private double lockedFieldAngle = 0.0;

    /**
     * Call once (e.g. on entering a collection segment) with the field pose
     * the robot is expected to be at for the NEXT shot. Picks targetClose vs
     * targetFar the same way the live-aim path does, based on distance from
     * that predicted point to the goal.
     */
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

    /** Call on entering the next shoot-window segment to fall back to full
     *  live tracking (with lead compensation) for final approach precision. */
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

    // Latched on deadzone entry. True = came from lower side (<=240°).
    public boolean approachingFromLower = true;

    // Hysteresis: enter latch at LOWER/UPPER_DEADZONE, exit only once the
    // turret has fully cleared back past LOWER_HOLD or UPPER_HOLD.
    public boolean inDeadzoneLatch = false;

    // =========================
    // Angular-velocity feedforward state
    // =========================

    private final ElapsedTime feedforwardTimer = new ElapsedTime();
    private double lastTargetAngle = homePos;

    // Last computed target angular velocity (rad/s), exposed for telemetry/tuning.
    public double lastTargetAngularVelocity = 0.0;

    // =========================
    // Manual jog / home-reset control
    // =========================

    // Set every loop by TeleOp from Jonathan's triggers: +1 = full CW jog,
    // -1 = full CCW jog, 0 = no jog. Only has effect while enableAim is false.
    public double manualPower = 0.0;

    // Max servo power used while jogging (kept below maxPower for control feel).
    public static double manualJogPower = 0.5;

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

        // Compute HOLD boundaries FIRST before anything reads them
        LOWER_HOLD = LOWER_DEADZONE - Math.toRadians(deadzoneMarginDeg);
        UPPER_HOLD = UPPER_DEADZONE + Math.toRadians(deadzoneMarginDeg);

        // Reconstruct encoder offset using the exact snapshot pair from Storage
        int ticksForSavedAngle = (int) Math.round(Storage.turretAngle * ticksPerRadian);
        encoderOffset = Storage.turretEncoderSnapshot - ticksForSavedAngle;

        // 2. Set the default target angle to wherever Auto left off, NOT homePos
        currentTargetAngle = Storage.turretAngle;
        lastTargetAngle = Storage.turretAngle;
        feedforwardTimer.reset();

        // Now these reads are valid
        approachingFromLower = Storage.turretAngle <= LOWER_DEADZONE;
        inDeadzoneLatch = Storage.turretAngle > LOWER_DEADZONE
                && Storage.turretAngle < UPPER_DEADZONE;
    }

    // =========================
    // Snapshot
    // =========================

    public void saveToStorage() {
        // Save raw angle WITHOUT trim so the snapshot pair is self-consistent
        int correctedTicks = encoderMotor.getCurrentPosition() - encoderOffset;
        double rawAngleNoTrim = wrapToTwoPi(correctedTicks / ticksPerRadian);

        Storage.turretAngle = rawAngleNoTrim;
        Storage.turretEncoderSnapshot = encoderMotor.getCurrentPosition();
        encoderTrim = 0;
    }

    // =========================
    // Manual home reset
    //
    // Call this when Jonathan has jogged the turret (via manualPower, below)
    // to the correct physical home and presses dpad-down. It re-zeros the
    // encoder offset at the CURRENT physical position, so getNormalizedAngle()
    // reads 0 immediately afterward, and makes that the new homePos.
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
        Storage.turretAngle = normalizedPos;

        // ====================================================================
        // 0. Manual jog override (relocating a wrong home position)
        //
        // Only active while aim-assist is OFF and Jonathan is holding a trigger.
        // Deliberately open-loop and bypasses the deadzone guard entirely: the
        // whole point of jogging is to physically find a new zero when the
        // current encoderOffset (and therefore the deadzone mapping) may be
        // wrong, so the deadzone math can't be trusted while doing it.
        // ====================================================================
        if (!enableAim && Math.abs(manualPower) > 0.02) {
            double clampedManual = clamp(manualPower, -1.0, 1.0) * manualJogPower;
            leftServo.setPower(-clampedManual);
            rightServo.setPower(-clampedManual);

            // Keep PD/feedforward state synced to wherever we physically are so
            // releasing the trigger doesn't cause a snap back toward a stale target.
            currentTargetAngle = normalizedPos;
            lastTargetAngle = normalizedPos;
            lastError = 0.0;
            controller.setSetPoint(currentTargetAngle);
            feedforwardTimer.reset();
            return;
        }

        // ====================================================================
        // 1. Hysteresis deadzone detection
        //
        // Enter latch the moment we cross into (240°, 290°).
        // Exit latch only once fully clear of LOWER_HOLD or UPPER_HOLD.
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
            // Predictive lock: field angle was already resolved once by
            // setLockedTarget(); only reproject it through the CURRENT
            // heading, since heading can still change while intaking.
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

            // ================================================================
            // 2a. Shoot-on-the-move lead compensation
            //
            // A ball launched while the robot is translating inherits the
            // robot's field-relative velocity. To land on the real target we
            // aim at a "virtual target" shifted opposite the direction of
            // travel by (robotVelocity * timeOfFlight) — i.e. we aim short of
            // where the robot's motion would otherwise carry the shot, so the
            // inherited velocity component cancels out over the flight time.
            //
            // timeOfFlight currently comes from Shooter's placeholder LUT
            // (Shooter.lutTimeOfFlight) — bench-measure and replace those
            // points before trusting this for real matches.
            //
            // NOTE: verify getVelocity()'s return type/units against your
            // Pedro Pathing version — this assumes a field-relative Vector
            // with getX()/getY() in the same units as robotPose (inches/sec).
            // ================================================================
            if (enableLeadCompensation) {
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
            currentTargetAngle = homePos;
        }

        // ====================================================================
        // 2b. Angular-velocity feedforward
        //
        // Knowing WHERE to aim (2. above) is different from actually getting
        // there in time — while the robot rotates/translates or lead compensation
        // shifts the virtual target, currentTargetAngle itself is moving every
        // loop, and a P(D) loop only ever reacts after an error appears. Estimate
        // the target's own angular velocity (rad/s) and feed it forward directly
        // as commanded power so the turret is already matching the setpoint's
        // rotation rate instead of chasing it from behind.
        //
        // Guarded against the first loop after init/mode-switch and against
        // stalls (dt too large) so a stale dt doesn't produce a bogus velocity
        // spike; also gets naturally zeroed while inDeadzoneLatch holds the
        // target still, and reset whenever manual jog runs (see block 0 above).
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
        //
        // Shift both angles by FRAME_SHIFT (265°) so the wrap seam sits inside
        // the deadzone. This eliminates the 0/2PI jitter.
        //
        // Then, if the shortest-path error in this frame would still route
        // through the deadzone (SHIFTED_LOWER to SHIFTED_UPPER), force it the
        // other way.
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
                lastError -= 2.0 * Math.PI; // Keep derivative stable!
            } else if (onLowerShiftedSide && error < 0 && (shiftedPos + error) < SHIFTED_UPPER) {
                error += 2.0 * Math.PI;
                lastError += 2.0 * Math.PI; // Keep derivative stable!
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

    // Shortest signed angular distance a -> b, wrapped into (-PI, PI].
    // Used so the feedforward velocity estimate doesn't spike when the raw
    // difference crosses the 0/2PI seam.
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