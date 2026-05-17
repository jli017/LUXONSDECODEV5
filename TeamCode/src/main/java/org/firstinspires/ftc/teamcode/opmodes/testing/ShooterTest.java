package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@TeleOp
public class ShooterTest extends OpMode {
    Motor flywheel1, flywheel2;

    DcMotor intake, transfer;

    Servo hood;

    int speed = 0;

    double power = 0;
    double position = 0.5;

    public static double P = 0.001;//0.006 0.000389
    public static double D = 0.0;
    public static double F =0.000385;//0.0008
    public PIDFController controller = new PIDFController(P, 0, D, F);

    //I like this code

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        flywheel1 = new Motor(hardwareMap, "1", Motor.GoBILDA.BARE);
        flywheel2 = new Motor(hardwareMap, "2", Motor.GoBILDA.BARE);
        flywheel1.setInverted(true);
        flywheel1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        hood = hardwareMap.get(Servo.class, "hood");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {

        // Robot-centric mecanum drive
        transfer.setPower(gamepad1.right_trigger);
        intake.setPower(gamepad1.left_trigger);

        if (gamepad1.dpadUpWasPressed()) {
            speed += 100;
        }
        else if (gamepad1.dpadDownWasPressed()) {
            speed -= 100;
        }
        else if (gamepad1.circleWasPressed()){
           speed = 0;
        }
        else if (gamepad1.rightBumperWasPressed()){
            position += 0.05;
        }
        else if (gamepad1.leftBumperWasPressed()){
            position -= 0.05;
        }

        controller.setSetPoint(speed);

        power = controller.calculate(speed);

        hood.setPosition(position);
        flywheel1.set(power);
        flywheel2.set(power);

        telemetry.addData("speed ", speed);
        telemetry.addData("position ", position);
    }
}
