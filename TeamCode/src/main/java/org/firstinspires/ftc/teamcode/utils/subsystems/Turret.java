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

    public static double p = 0.7;
    public static double d = 0.09;

    // Feedforward to overcome internal CR Servo friction
    public static double kStatic = 0.04;
    public static double maxPower = 0.55;
    public static double toleranceDeg = 1.5;
    public static double minOutput = 0.02;

    public PIDFController controller = new PIDFController(p, 0, d, 0);

    // =========================
    // Hard Limits
    // =========================

    // CCW = positive (Pedro standard)
    // MAX_ANGLE: +240° CCW from home
    // MIN_ANGLE: -70°  CW  from home
    private static final double MAX_ANGLE = Math.toRadians(240.0);
    private static final double MIN_ANGLE = Math.toRadians(-70.0);

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
        // Adjust which is FORWARD/REVERSE depending on your physical gearing meshing.
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

        double rawPos = getRawAngle();
        Storage.turretAngle = rawPos;

        // 1. Determine Target Angle
        if (enableAim) {
            Pose robotPose = Lebruxon.drivetrain.follower.getPose();
            double dx = Lebruxon.goalShooter.getX() - robotPose.getX();
            double dy = Lebruxon.goalShooter.getY() - robotPose.getY();

            double fieldTargetAngle = Math.atan2(dy, dx);
            double robotHeading = Lebruxon.drivetrain.follower.getHeading();

            // Get angle relative to the robot's front
            double relativeAngle = wrapToPi(fieldTargetAngle - robotHeading);

            // Shift negative angles that actually represent valid travel > 180 deg CCW
            // (e.g., wrapToPi turns +200 deg into -160 deg. This moves it back to +200)
            if (relativeAngle < MIN_ANGLE && (relativeAngle + 2.0 * Math.PI) <= MAX_ANGLE) {
                relativeAngle += 2.0 * Math.PI;
            }

            // If the target is outside our hardware bounds, reset straight to 0
            if (relativeAngle < MIN_ANGLE || relativeAngle > MAX_ANGLE) {
                currentTargetAngle = homePos;
            } else {
                currentTargetAngle = relativeAngle;
            }
        } else {
            // Default home position when aim is disabled
            currentTargetAngle = homePos;
        }

        // 2. Run PID
        controller.setSetPoint(currentTargetAngle);
        double power = controller.calculate(rawPos);

        // Apply friction feedforward ONLY if we are outside of the tolerance bounds
        if (Math.abs(currentTargetAngle - rawPos) > Math.toRadians(toleranceDeg)) {
            power += Math.signum(power) * kStatic;
        } else {
            power = 0; // Turn off entirely when at the target
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

    public double getRawAngle() {
        return -encoderMotor.getCurrentPosition() / ticksPerRadian;
    }

    public double getAngle() {
        return getRawAngle();
    }

    public double getTargetAngle() {
        return currentTargetAngle;
    }

    // =========================
    // Utility
    // =========================

    public static double wrapToPi(double radians) {
        double wrapped = radians % (2.0 * Math.PI);
        if (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        } else if (wrapped <= -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}