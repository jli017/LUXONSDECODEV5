
package org.firstinspires.ftc.teamcode.utils.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    public static double F =0.000385;//0.0008
    public PIDFController controller = new PIDFController(P, 0, D, F);
    public static double TOLERANCE = 50;

    public static double STOPPER_OPEN = 0.3;
    public static double STOPPER_CLOSED = 0.1;
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
        lutVelocity.add(0, 1390);
        lutVelocity.add(25, 1400);
        lutVelocity.add(44.5, 1450);
        lutVelocity.add(52.5, 1500);
        lutVelocity.add(69, 1580);
        lutVelocity.add(93, 1740);
        lutVelocity.add(105.5, 1800);
        lutVelocity.add(133, 1950);
        lutVelocity.add(144.5, 2040);
        lutVelocity.add(157.5, 2140);

        lutHood.add(0, 0.6);
        lutHood.add(25, 0.6);
        lutHood.add(44.5, 0.23);
        lutHood.add(52.5, 0.16);
        lutHood.add(69, 0.1);
        lutHood.add(93, 0.09);
        lutHood.add(105.5, 0);
        lutHood.add(133, 0);
        lutHood.add(144.5, 0);
        lutHood.add(157.5, 0);

        lutVelocity.createLUT();
        lutHood.createLUT();
        pos = Lebruxon.drivetrain.follower.getPose();
        controller.setP(P);
        controller.setF(F);


        shooterBlah = false;
        distance = Math.hypot(Lebruxon.goalShooter.getX()-Storage.pose.getX(), Lebruxon.goalShooter.getY()-Storage.pose.getY());


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
        setHoodPercent(0);
    }

    public void raiseHood() {
        setHoodPercent(HOOD_NEAR);
    }

    public void setHoodPercent(double percent) {
        hood.set( HOOD_MAX * percent);
    }
    public void setCurrentHoodPercent(double percent) {hood.set(hood.getRawPosition()*percent);}
}
