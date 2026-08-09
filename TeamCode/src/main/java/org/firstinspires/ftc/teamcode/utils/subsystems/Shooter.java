package org.firstinspires.ftc.teamcode.utils.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.utils.Lebruxon;
import org.firstinspires.ftc.teamcode.utils.Storage;


@Configurable
public class Shooter extends SubsystemBase {
    public Motor shooter1;
    public Motor shooter2;
    public ServoEx hood;
    public ServoEx stopper;

    public boolean idle;
    public int add;

    public static double P = 0.001;//0.006 0.000389
    public static double D = 0.0;
    public static double F = 0.000385;//0.0008
    public PIDFController controller = new PIDFController(P, 0, D, F);
    public static double TOLERANCE = 100;

    public static double STOPPER_OPEN = 0.35;
    public static double STOPPER_CLOSED = 0.14;
    public static double TRANSFER_UP = 0.85;
    public static double TRANSFER_DOWN = 0.5;
    public static double HOOD_MIN = 0;
    public static double HOOD_MAX = 1;

    public static double HOOD_NEAR = 0;


    public static double idleVeloMultiplier = 0.0;

    public static double multiplier = 0.65;

    InterpLUT lutVelocity = new InterpLUT();
    InterpLUT lutHood = new InterpLUT();

    // =========================
    // Time-of-Flight LUT (shoot-on-the-move)
    // =========================
    // Maps distance (inches) -> ball flight time (seconds).
    // PLACEHOLDER VALUES ONLY. These need to be bench/field measured, e.g. with
    // a stopwatch or high-speed camera timing release-to-impact at each distance
    // bucket, then replaced here. Roughly interpolated from the existing
    // velocity LUT for now (higher exit velocity + flatter hood angle at range
    // -> shorter flight time per inch, so the curve isn't perfectly linear).
    InterpLUT lutTimeOfFlight = new InterpLUT();
    public double timeOfFlight;

    public double distance;
    public double power;
    public boolean shooterBlah;
    public Pose pos;
    //double currentVelocity = 0;

    // =========================
    // Shoot-on-the-move: radial velocity compensation
    // =========================
    // Turret's lead compensation only corrects the TANGENTIAL component of
    // robot velocity (perpendicular to the line to the goal) — it shifts aim
    // angle. The RADIAL component (straight toward/away from the goal)
    // doesn't need a new aim angle, but it does change how far the shot
    // effectively has to travel by the time it arrives, so it needs to
    // change shot power/hood instead. effectiveDistance is `distance`
    // corrected for that, and it's what actually gets fed into the
    // velocity/hood LUTs below (raw `distance` is left alone since Turret
    // still uses it for its own far/close target selection).
    public static boolean enableRadialCompensation = true;
    public static double radialMultiplier = 1.0;

    // Positive = closing on the goal, negative = moving away. Telemetry only.
    public double radialVelocity = 0.0;
    public double effectiveDistance = 0.0;

    public Shooter(HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooterMotor", Motor.GoBILDA.BARE);
        shooter2 = new Motor(hMap, "shooterMotor2", Motor.GoBILDA.BARE);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hMap, "HoodServo");
        stopper = new ServoEx(hMap, "StopperServo");

        shooter1.setInverted(true);
        shooter2.setInverted(true);
        controller.setTolerance(TOLERANCE);
        controller.setSetPoint(0);
        lutVelocity.add(-30, 1000);
        lutVelocity.add(0, 1300);
        lutVelocity.add(27.5, 1300);
        lutVelocity.add(32, 1380);
        lutVelocity.add(49, 1480);
        lutVelocity.add(61, 1600);
        lutVelocity.add(78, 1740);
        lutVelocity.add(105.2, 1850);
        lutVelocity.add(109.25, 1980);
        lutVelocity.add(118, 2020);
        lutVelocity.add(122.5, 2080);
        lutVelocity.add(200, 2160);

        lutHood.add(-30, 1);
        lutHood.add(0, 1);
        lutHood.add(27.5, 1);
        lutHood.add(32, 0.9);
        lutHood.add(49, 0.78);
        lutHood.add(61, 0.72);
        lutHood.add(78, 0.66);
        lutHood.add(105.2, 0.6);
        lutHood.add(109.25, 0.5);
        lutHood.add(118, 0.5);
        lutHood.add(122.5, 0.47);
        lutHood.add(200, 0.47);

        // TODO(bench-tune): replace with measured flight times per distance.
        // Close zone (< ~100") and far zone (>= ~100") both represented.
        lutTimeOfFlight.add(-30, 0.55);
        lutTimeOfFlight.add(0, 0.55);
        lutTimeOfFlight.add(30, 0.65);
        lutTimeOfFlight.add(61, 0.68);
        lutTimeOfFlight.add(105.2, 0.7);
        lutTimeOfFlight.add(122.5, 0.77);
        lutTimeOfFlight.add(200, 0.90);


        lutVelocity.createLUT();
        lutHood.createLUT();
        lutTimeOfFlight.createLUT();
        controller.setP(P);
        controller.setF(F);

        shooterBlah = false;
        refreshDistance();
    }

    public void update() {
        refreshDistance();

        double targetVelocity;
        if (idle) {
            if (!shooterBlah) {
                targetVelocity = (distance < 100) ? (lutVelocity.get(effectiveDistance) + add) : 1500;
            } else {
                targetVelocity = lutVelocity.get(effectiveDistance) + add;
            }
        } else {
            if (!shooterBlah) {
                targetVelocity = 0;
            } else {
                targetVelocity = lutVelocity.get(effectiveDistance) + add;
            }
        }

        driveToVelocity(targetVelocity);
    }

    // =========================
    // Distance / radial-velocity refresh
    // =========================

    /**
     * Refreshes pos, distance, timeOfFlight, and the radial-velocity-corrected
     * effectiveDistance. Called once per update() so every branch below works
     * off the same snapshot instead of re-deriving it four separate times.
     */
    private void refreshDistance() {
        pos = Lebruxon.drivetrain.follower.getPose();
        distance = Math.hypot(
                Lebruxon.goalShooter.getX() - pos.getX(),
                Lebruxon.goalShooter.getY() - pos.getY()
        );
        timeOfFlight = lutTimeOfFlight.get(distance);
        updateRadialCompensation();
    }

    /**
     * Projects robot velocity onto the robot->goal unit vector to get the
     * radial (closing/opening) speed, then shifts distance by how much ground
     * that speed covers over the ball's flight time. A single pass using the
     * raw-distance timeOfFlight is accurate enough given how coarse
     * lutTimeOfFlight already is — no need to iterate to convergence.
     */
    private void updateRadialCompensation() {
        if (distance > 1e-6) {
            Vector robotVel = Lebruxon.drivetrain.follower.getVelocity();
            double ux = (Lebruxon.goalShooter.getX() - pos.getX()) / distance;
            double uy = (Lebruxon.goalShooter.getY() - pos.getY()) / distance;
            radialVelocity = robotVel.getXComponent() * ux + robotVel.getYComponent() * uy;
        } else {
            radialVelocity = 0.0;
        }

        if (enableRadialCompensation) {
            effectiveDistance = Math.max(0.0, distance - radialVelocity * timeOfFlight * radialMultiplier);
        } else {
            effectiveDistance = distance;
        }
    }

    private void driveToVelocity(double targetVelocity) {
        controller.setSetPoint(targetVelocity);
        hood.set(lutHood.get(effectiveDistance));
        double currentVelocity = getVelocity();
        power = controller.calculate(currentVelocity);
        setPower(power);
    }


    public void setShooter(boolean s) {
        shooterBlah = s;
    }

    public void setVelocity(double velocity) {
        controller.setSetPoint(velocity);
        // currentVelocity = velocity;
    }

    public double getVelocity() {
        return shooter2.getCorrectedVelocity();
    }

    /**
     * Returns the current placeholder time-of-flight estimate (seconds) for the
     * shooter's last-computed distance. Turret.update() reads this to compute
     * the shoot-on-the-move lead offset.
     */
    public double getTimeOfFlight() {
        return timeOfFlight;
    }

    /**
     * Looks up time-of-flight (seconds) for an arbitrary distance (inches).
     */
    public double getTimeOfFlight(double dist) {
        return lutTimeOfFlight.get(dist);
    }

    public void setPower(double power) {
        power = clamp(power, -1.0, 1.0);
        shooter1.set(power);
        shooter2.set(-power);
    }

    public void autoPower(boolean shooterOn, boolean hoodOn) {
        shooterBlah = shooterOn;

        if (shooterOn) {
            controller.setP(P);
            controller.setF(F);
        }
    }


    public void closeStopper() {
        stopper.set(STOPPER_CLOSED);
    }

    public void openStopper() {
        stopper.set(STOPPER_OPEN);
    }

    public void resetHood() {
        setHoodPercent(100);
    }

    public void raiseHood() {
        setHoodPercent(HOOD_NEAR);
    }

    public void setHoodPercent(double percent) {
        hood.set( HOOD_MAX * percent);
    }
    public void setCurrentHoodPercent(double percent) {hood.set(hood.getRawPosition()*percent);}
}