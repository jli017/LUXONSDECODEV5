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

    public static double p = 0.7;
    public static double d = 0.09;

    // Feedforward to overcome internal CR Servo friction
    public static double kStatic = 0.04;
    public static double maxPower = 0.55;
    public static double toleranceDeg = 1.5;
    public static double minOutput = 0.02;

    public PIDFController controller = new PIDFController(p, 0, d, 0);

    // ===================================
    // Hard Limits (Mapped 0 to 2PI Space)
    // ===================================

    // Safe Travel Region: Side A [0°, 240°] and Side B [290°, 360°]
    // Prohibited Deadzone Region: (240°, 290°)
    private static final double LOWER_DEADZONE = Math.toRadians(240.0);
    private static final double UPPER_DEADZONE = Math.toRadians(290.0);

    // =========================
    // Runtime State
    // =========================

    public static double homePos = 0.0;
    public boolean enableAim = false;
    public boolean AUTOenableAim = false;

    private double currentTargetAngle = homePos;

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
    }

    // =========================
    // Update
    // =========================

    public void update() {

        controller.setP(p);
        controller.setD(d);
        controller.setTolerance(Math.toRadians(toleranceDeg));

        // 1. Get current position normalized strictly between 0 and 2PI
        double normalizedPos = getNormalizedAngle();
        Storage.turretAngle = normalizedPos;

        // 2. Resolve Target Angle
        if (enableAim) {
            Pose robotPose = Lebruxon.drivetrain.follower.getPose();
            double dx = Lebruxon.goalShooter.getX() - robotPose.getX();
            double dy = Lebruxon.goalShooter.getY() - robotPose.getY();

            double fieldTargetAngle = Math.atan2(dy, dx);
            double robotHeading = Lebruxon.drivetrain.follower.getHeading();

            // Calculate the raw relative target and wrap to [0, 2PI) space
            double normalizedTarget = wrapToTwoPi(fieldTargetAngle - robotHeading);

            // If target falls inside off-limits zone, force return to home (0.0) until it leaves
            if (normalizedTarget > LOWER_DEADZONE && normalizedTarget < UPPER_DEADZONE) {
                currentTargetAngle = homePos;
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
        // forbidden deadzone at the very end of our 0 to 2PI data ribbon [310°, 360°].
        // This converts our circular path problem into a completely linear one.
        double shiftedCurrent = wrapToTwoPi(normalizedPos - UPPER_DEADZONE);
        double shiftedTarget = wrapToTwoPi(currentTargetAngle - UPPER_DEADZONE);

        // Calculate the linear scalar error. Because the deadzone bounds the edges
        // of this shifted space, the shortest path is always the safe path.
        double error = shiftedTarget - shiftedCurrent;

        // Project the target back out into global PID setpoint space
        double pidSetpoint = normalizedPos + error;

        // ====================================================================
        // 4. Run PID & Output Control
        // ====================================================================
        controller.setSetPoint(pidSetpoint);
        double power = controller.calculate(normalizedPos);

        // Apply friction feedforward only outside tolerance bounds
        if (Math.abs(pidSetpoint - normalizedPos) > Math.toRadians(toleranceDeg)) {
            power += Math.signum(power) * kStatic;
        } else {
            power = 0;
        }

        double clampedPower = clamp(power, -maxPower, maxPower);

        // Apply deadband
        if (Math.abs(clampedPower) < minOutput) {
            clampedPower = 0.0;
        }

        leftServo.setPower(clampedPower);
        rightServo.setPower(clampedPower);
    }

    // =========================
    // Public Accessors
    // =========================

    /**
     * Returns the raw encoder orientation normalized strictly between 0 and 2PI.
     * Handles negative raw positions from reverse rotation flawlessly.
     */
    public double getNormalizedAngle() {
        int adjustedTicks = encoderMotor.getCurrentPosition() - Storage.turretEncoderOffset;
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