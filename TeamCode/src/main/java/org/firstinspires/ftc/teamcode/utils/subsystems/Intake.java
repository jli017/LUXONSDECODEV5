package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {

    public Motor intake, transfer;
    public DistanceSensor distance;

    private static final double INTAKE_POWER = -1.0;
    private static final double OUTTAKE_POWER = 0.25;

    private double minPower = 0.0;
    private double intakePower = 0.0;
    private double transferPower = 0.0;

    public Intake(HardwareMap hMap) {
        intake = new Motor(hMap, "intake");
        transfer = new Motor(hMap, "transfer");
        distance = hMap.get(DistanceSensor.class, "distance");

        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.set(minPower);
        transfer.set(minPower);
    }

    public void setPower(double intake, double transfer) {
        this.intakePower = intake;
        this.transferPower = transfer;
    }

    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    public void periodic() {
        double inpow = Math.max(Math.abs(intakePower), Math.abs(minPower));
        if (inpow == Math.abs(minPower)) {
            intake.set(minPower);
        } else {
            intake.set(intakePower);
        }

        double trpow = Math.max(Math.abs(transferPower), Math.abs(minPower));
        if (trpow == Math.abs(minPower)) {
            transfer.set(minPower);
        } else {
            transfer.set(transferPower);
        }
    }

    public void update() {
        double inpow = Math.max(Math.abs(intakePower), Math.abs(minPower));
        if (inpow == Math.abs(minPower)) {
            intake.set(minPower);
        } else {
            intake.set(intakePower);
        }

        double trpow = Math.max(Math.abs(transferPower), Math.abs(minPower));
        if (trpow == Math.abs(minPower)) {
            transfer.set(minPower);
        } else {
            transfer.set(transferPower);
        }

        double dist = distance.getDistance(DistanceUnit.CM);

//        if (dist < 3){
//            intake.set(minPower);
//            transfer.set(minPower);
//        }
    }
}