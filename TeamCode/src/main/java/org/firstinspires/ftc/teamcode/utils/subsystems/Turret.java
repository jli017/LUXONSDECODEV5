package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

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

    private static final double MAX_ANGLE =
            Math.toRadians(70.0);

    private static final double MIN_ANGLE =
            Math.toRadians(-240.0);

    // =========================
    // Runtime State
    // =========================

    // Robot-relative home angle
    public static double homePos = Math.PI;

    public boolean enableAim = true;
    public boolean AUTOenableAim = true;

    private double filteredPower = 0;

    private double targetAngle = homePos;

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

        // You said these are correct
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

        controller.setTolerance(
                Math.toRadians(toleranceDeg)
        );

        // IMPORTANT:
        // Use continuous raw angle directly.
        // Do NOT wrap turret position.
        double pos = getRawAngle();

        Storage.turretAngle = pos;

        double relativeAngle;

        if (enableAim) {

            Pose robotPose =
                    Lebruxon.drivetrain.follower.getPose();

            double dx =
                    Lebruxon.goal.getX()
                            - robotPose.getX();

            double dy =
                    Lebruxon.goal.getY()
                            - robotPose.getY();

            // Field-relative target angle
            double fieldTargetAngle =
                    Math.atan2(dy, dx);

            double robotHeading =
                    Lebruxon.drivetrain.follower.getHeading();

            // Robot-relative desired turret angle
            relativeAngle =
                    wrapToPi(fieldTargetAngle - robotHeading);

        } else {

            relativeAngle = wrapToPi(homePos);
        }

        // Convert wrapped target into the closest
        // reachable continuous target.

        //controller.setSetPoint(MathUtils.clamp(relativeAngle,Math.toRadians(-90),Math.toRadians(90)));
        controller.setSetPoint(relativeAngle);
        double power = controller.calculate(getRawAngle());
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    // =========================
    // Continuous Target Solver
    // =========================

    /**
     * desiredWrapped is in [-pi, pi]
     * current is continuous encoder space
     *
     * This finds the closest reachable target
     * without violating hard limits.
     */

    // =========================
    // Public Accessors
    // =========================

    /**
     * Continuous raw angle from encoder.
     * NEVER wrapped.
     */
    public double getRawAngle() {

        return -encoderMotor.getCurrentPosition()
                / ticksPerRadian;
    }

    /**
     * Current turret angle in continuous space.
     */
    public double getAngle() {

        return getRawAngle();
    }

    /**
     * Current target in continuous space.
     */
    public double getTargetAngle() {

        return targetAngle;
    }

    // =========================
    // Utility
    // =========================



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
}