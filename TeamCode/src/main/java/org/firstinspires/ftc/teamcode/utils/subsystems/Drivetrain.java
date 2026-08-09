package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.Lebruxon;
import org.firstinspires.ftc.teamcode.utils.Storage;

@Configurable
public class Drivetrain extends SubsystemBase {
    public Follower follower;
    public static double fast = 1;
    public static boolean turbo = false;
    public static boolean lock = false;

    // =========================
    // Heading Lock
    // =========================

    // Target heading (degrees) to lock to, per alliance, while `lock` is true.
    public static double blueLockHeadingDeg = 145.0;
    public static double redLockHeadingDeg = 35.0;

    // PIDF gains for the heading-lock loop, using SolversLib's PIDFController
    // (same class Turret.java uses). IMPORTANT: the error fed into this
    // controller is in RADIANS, not degrees, and the output is motor power
    // clamped to [-1, 1] — so these gains are NOT on a "per degree" scale.
    // A 100° error is ~1.75 rad, and the largest possible error (robot facing
    // exactly opposite the target) is PI rad (~3.14). So headingLockP alone
    // needs to keep P * PI reasonably close to 1, not blow way past it —
    // e.g. P = 0.3 gives ~0.94 power at the worst-case error, which is why
    // that's the starting point below, not something like 0.6 which would
    // already be commanding full+ power (get clamped) for any error over ~60°.
    public static double headingLockP = 0.3;
    public static double headingLockI = 0.0;
    public static double headingLockD = 0.02;
    public static double headingLockF = 0.0;
    public static double headingLockMaxPower = 0.9;

    // Feed this controller a pre-wrapped shortest-path error (see
    // shortestAngleDiff) rather than raw headings directly, since it has no
    // built-in awareness of the 0/2PI wrap seam.
    private final PIDFController headingLockController =
            new PIDFController(headingLockP, headingLockI, headingLockD, headingLockF);

    private double lockPower = 0.0;

    // Last computed heading-lock error (radians), exposed for telemetry/tuning.
    public double lastHeadingLockError = 0.0;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.update();
    }

    public void drive(Gamepad gamepad1) {

        // ====================================================================
        // Heading lock: drive lockPower toward whichever alliance's target
        // heading (145° blue / 35° red) via SolversLib's PIDFController.
        // Reset the controller's internal state whenever lock isn't engaged
        // so there's no integral windup or stale-derivative spike the next
        // time it's turned back on.
        // ====================================================================
        if (lock) {
            boolean blueAlliance = Storage.alliance == Lebruxon.Alliance.BLUE
                    || Storage.alliance == Lebruxon.Alliance.BLUECLOSE
                    || Storage.alliance == Lebruxon.Alliance.BLUESQ;
            double targetHeadingDeg = blueAlliance ? blueLockHeadingDeg : redLockHeadingDeg;
            lockPower = updateHeadingLock(Math.toRadians(targetHeadingDeg));
        } else {
            lockPower = 0.0;
            headingLockController.reset();
        }

        if(Storage.alliance == Lebruxon.Alliance.BLUE || Storage.alliance == Lebruxon.Alliance.BLUECLOSE || Storage.alliance == Lebruxon.Alliance.BLUESQ ) {
            if (!lock) {
                follower.setTeleOpDrive(
                        gamepad1.left_stick_y * 0.8,
                        gamepad1.left_stick_x * 0.8,
                        -gamepad1.right_stick_x * 0.8,
                        false
                );
            }
            else {
                follower.setTeleOpDrive(
                        gamepad1.left_stick_y * 0.8,
                        gamepad1.left_stick_x * 0.8,
                        lockPower,
                        false
                );
            }
        }
        else {
            if (!lock) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * 0.8,
                        -gamepad1.left_stick_x * 0.8,
                        -gamepad1.right_stick_x * 0.8,
                        false
                );
            }
            else {
                follower.setTeleOpDrive(
                        gamepad1.left_stick_y * 0.8,
                        gamepad1.left_stick_x * 0.8,
                        lockPower,
                        false
                );
            }
        }

    }

    // =========================
    // Heading-lock PIDF
    // =========================

    private double updateHeadingLock(double targetHeadingRad) {
        double currentHeadingRad = follower.getPose().getHeading();

        // Shortest-path error, wrapped into (-PI, PI], so a target of 145°
        // and a current heading of e.g. 350° produces a small correction the
        // short way around rather than a near-360° error.
        double error = shortestAngleDiff(targetHeadingRad, currentHeadingRad);
        lastHeadingLockError = error;

        // Sync tunable gains every loop so dashboard edits take effect live.
        headingLockController.setPIDF(headingLockP, headingLockI, headingLockD, headingLockF);

        // Feed the pre-wrapped error in directly: setpoint = 0, measurement =
        // -error, so the controller's internal (setpoint - measurement) works
        // out to exactly our wrapped error, sign and all.
        headingLockController.setSetPoint(0);
        double power = headingLockController.calculate(-error);

        return clamp(power, -headingLockMaxPower, headingLockMaxPower);
    }

    // Shortest signed angular distance target -> current, wrapped into (-PI, PI].
    private static double shortestAngleDiff(double target, double current) {
        double diff = (target - current) % (2.0 * Math.PI);
        if (diff > Math.PI) diff -= 2.0 * Math.PI;
        if (diff < -Math.PI) diff += 2.0 * Math.PI;
        return diff;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public void update(){
        follower.update();
        Storage.pose = follower.getPose();
    }
}