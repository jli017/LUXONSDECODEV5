
package org.firstinspires.ftc.teamcode.utils.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
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

    public static double P = 0.001;//0.006 0.000389
    public static double D = 0.0;
    public static double F = 0.000385;//0.0008
    public PIDFController controller = new PIDFController(P, 0, D, F);
    public static double TOLERANCE = 50;

    public static double STOPPER_OPEN = 0.35;
    public static double STOPPER_CLOSED = 0.2;
    public static double TRANSFER_UP = 0.85;
    public static double TRANSFER_DOWN = 0.5;
    public static double HOOD_MIN = 0;
    public static double HOOD_MAX = 1;

    public static double HOOD_NEAR = 0;


    public static double idleVeloMultiplier = 0.0;

    public static double multiplier = 0.65;

    InterpLUT lutVelocity = new InterpLUT();
    InterpLUT lutHood = new InterpLUT();
    public double distance;
    public double power;
    public boolean shooterBlah;
    public Pose pos;
    //double currentVelocity = 0;

    public Shooter(HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooterMotor", Motor.GoBILDA.BARE);
        shooter2 = new Motor(hMap, "shooterMotor2", Motor.GoBILDA.BARE);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hMap, "HoodServo");
        stopper = new ServoEx(hMap, "StopperServo");

        shooter1.setInverted(false);
        shooter2.setInverted(false);
        controller.setTolerance(TOLERANCE);
        controller.setSetPoint(0);
        lutVelocity.add(0, 1360);
        lutVelocity.add(29.7, 1440);
        lutVelocity.add(42.7, 1480);
        lutVelocity.add(57.7, 1580);
        lutVelocity.add(73.2, 1700);
        lutVelocity.add(87.7, 1860);
        lutVelocity.add(105.2, 1920);
        lutVelocity.add(109.25, 2040);
        lutVelocity.add(118, 2080);
        lutVelocity.add(122.5, 2120);
        lutVelocity.add(200, 2200);

        lutHood.add(0, 1);
        lutHood.add(29.7, 0.95);
        lutHood.add(42.7, 0.85);
        lutHood.add(57.7, 0.7);
        lutHood.add(73.2, 0.66);
        lutHood.add(87.7, 0.66);
        lutHood.add(105.2, 0.6);
        lutHood.add(109.25, 0.5);
        lutHood.add(118, 0.5);
        lutHood.add(122.5, 0.47);
        lutHood.add(200, 0.47);


        lutVelocity.createLUT();
        lutHood.createLUT();
        pos = Lebruxon.drivetrain.follower.getPose();
        controller.setP(P);
        controller.setF(F);


        shooterBlah = false;
        Pose ShooterRobotPose = Lebruxon.drivetrain.follower.getPose();
        distance = Math.hypot(Lebruxon.goalShooter.getX()- ShooterRobotPose.getX(), Lebruxon.goalShooter.getY()-ShooterRobotPose.getY());


    }

    public void update() {
        if (!shooterBlah) {
            controller.setSetPoint(0);
            setPower(0);
            return;
        }
        pos = Lebruxon.drivetrain.follower.getPose();

        distance = Math.hypot(
               Lebruxon.goalShooter.getX() - pos.getX(),
                Lebruxon.goalShooter.getY() - pos.getY()
        );

        double currentVelocity = getVelocity();
        double targetVelocity = lutVelocity.get(distance);

        controller.setSetPoint(targetVelocity);

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
            controller.setSetPoint(lutVelocity.get(distance));
        } else {
            controller.setSetPoint(0);
        }

        if (hoodOn) {
            hood.set(lutHood.get(distance));
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
