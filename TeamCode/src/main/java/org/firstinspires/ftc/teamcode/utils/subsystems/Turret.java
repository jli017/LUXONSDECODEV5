package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    // Encoder motor ONLY for reading position
    public DcMotorEx encoderMotor;

    // =========================
    // Encoder / gearing
    // =========================

    // Encoder CPR
    public static double encoderTicksPerRev = 8192.0;

    // Turret gear ratio:
    // turret pulley = 208mm
    // encoder pulley = 71mm
    //
    // encoder spins faster than turret
    public static double gearRatio = 208.0 / 71.0;

    // ticks per turret revolution
    public static double ticksPerTurretRev =
            encoderTicksPerRev * gearRatio;

    public static double ticksPerRadian =
            ticksPerTurretRev / (2.0 * Math.PI);

    // =========================
    // PID
    // =========================
    public static double p = 1.8;
    public static double d = 0.03;

    public PIDFController controller =
            new PIDFController(p, 0, d, 0);

    public double tolerance = Math.toRadians(2);

    // =========================
    // Limits / wrapping
    // =========================

    // Home is backwards
    public double homePos = Math.PI;

    // Prevent wire twisting
    // turret will stay within [-PI, PI]
    // and automatically reverse direction
    public double minAngle = -2.0 * Math.PI;
    public double maxAngle = 2.0 * Math.PI;

    // =========================
    // Auto aim
    // =========================
    public boolean enableAim = false;
    public boolean AUTOenableAim = false;

    // Desired angle
    private double targetAngle = Math.PI;

    public Turret(HardwareMap hMap) {

        leftServo = hMap.get(CRServo.class, "turretLeft");
        rightServo = hMap.get(CRServo.class, "turretRight");

        encoderMotor = hMap.get(DcMotorEx.class, "intake");

        encoderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse one servo if mounted opposite
        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        controller.setTolerance(tolerance);

        targetAngle = homePos;
    }

    // =========================
    // Current turret angle
    // =========================
    public double getAngle() {

        double ticks = encoderMotor.getCurrentPosition();

        // Convert encoder ticks -> turret radians
        double angle = ticks / ticksPerRadian;

        return wrapToPi(angle);
    }

    // =========================
    // Smart shortest-path setAngle
    // =========================
    public void setAngle(double angle) {

        angle = wrapToPi(angle);

        double current = getAngle();

        // shortest angular path
        double delta = wrapToPi(angle - current);

        targetAngle = current + delta;

        // keep inside safe range
        if (targetAngle > maxAngle) {
            targetAngle -= 2.0 * Math.PI;
        }

        if (targetAngle < minAngle) {
            targetAngle += 2.0 * Math.PI;
        }

        controller.setSetPoint(targetAngle);
    }

    public void update() {

        if (enableAim) {

            Pose pos = Lebruxon.drivetrain.follower.getPose();

            double deltaX = Lebruxon.goal.getX() - pos.getX();
            double deltaY = Lebruxon.goal.getY() - pos.getY();

            double fieldTargetAngle = Math.atan2(deltaY, deltaX);

            double robotAngle =
                    Lebruxon.drivetrain.follower.getHeading();

            // turret-relative angle
            double relativeAngle =
                    wrapToPi(fieldTargetAngle - robotAngle);

            setAngle(relativeAngle);

        }
         else {

            setAngle(homePos);
        }

        // Update PID
        controller.setP(p);
        controller.setD(d);

        double currentAngle = getAngle();

        Storage.turretAngle = currentAngle;

        double power = controller.calculate(currentAngle);

        // Clamp power
        power = Math.max(-1.0, Math.min(1.0, power));

        // Stop if close enough
        if (controller.atSetPoint()) {
            power = 0;
        }

        // Apply to BOTH servos
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    // =========================
    // Angle wrapping
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