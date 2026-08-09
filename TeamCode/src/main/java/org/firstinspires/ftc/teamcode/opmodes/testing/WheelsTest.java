package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class WheelsTest extends OpMode {
    DcMotor wheel1;
    DcMotor wheel2;
    DcMotor wheel3;
    DcMotor wheel4;

    DcMotor intake, shooter1, shooter2;

    Servo stopper;

    double power = 0;


    //I like this code

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        wheel1 = hardwareMap.get(DcMotor.class, "frontright");
        wheel2 = hardwareMap.get(DcMotor.class, "backright");
        wheel3 = hardwareMap.get(DcMotor.class, "frontleft");
        wheel4 = hardwareMap.get(DcMotor.class, "backleft");
        wheel1.setDirection(DcMotor.Direction.REVERSE);
//        wheel3.setDirection(DcMotor.Direction.REVERSE);
        shooter1 = hardwareMap.get(DcMotor.class, "2");
        shooter2 = hardwareMap.get(DcMotor.class, "3");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        stopper = hardwareMap.get(Servo.class, "1");
        stopper.setPosition(0.15);
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed()){
            power +=  0.1;
        }
        if (gamepad1.dpadDownWasPressed()){
            power -=  0.1;
        }

        if (gamepad1.cross){
            stopper.setPosition(0.50);
        }
        else {
            stopper.setPosition(0.15);
        }

        // Robot-centric mecanum drive
        intake.setPower(gamepad1.left_trigger);

        double y = -gamepad1.left_stick_y; // Forward/back
        double x = -gamepad1.left_stick_x;  // Strafe
        double rx = 0.65 * gamepad1.right_stick_x; // Rotate

        // Mecanum formulas
        double frontLeft = y + x + rx;
        double backLeft = y - x + rx;
        double frontRight = y - x - rx;
        double backRight = y + x - rx;

        // Normalize powers
        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(frontLeft),
                        Math.max(
                                Math.abs(backLeft),
                                Math.max(
                                        Math.abs(frontRight),
                                        Math.abs(backRight)
                                )
                        )
                )
        );

        frontLeft /= max;
        backLeft /= max;
        frontRight /= max;
        backRight /= max;

        // Apply powers
        wheel3.setPower(frontLeft);   // lF
        wheel4.setPower(backLeft);    // lB
        wheel1.setPower(frontRight);  // rF
        wheel2.setPower(backRight);   // rB
        shooter1.setPower(power);
        shooter2.setPower(power);

    }
}
