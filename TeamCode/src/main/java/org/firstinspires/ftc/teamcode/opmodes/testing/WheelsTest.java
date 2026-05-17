package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class WheelsTest extends OpMode {
    DcMotor wheel1;
    DcMotor wheel2;
    DcMotor wheel3;
    DcMotor wheel4;

    DcMotor intake, transfer;


    //I like this code

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        wheel1 = hardwareMap.get(DcMotor.class, "rF");
        wheel2 = hardwareMap.get(DcMotor.class, "rB");
        wheel3 = hardwareMap.get(DcMotor.class, "lF");
        wheel4 = hardwareMap.get(DcMotor.class, "lB");
        wheel3.setDirection(DcMotor.Direction.REVERSE);
        wheel4.setDirection(DcMotor.Direction.REVERSE);
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

        double y = -gamepad1.left_stick_y; // Forward/back
        double x = gamepad1.left_stick_x;  // Strafe
        double rx = gamepad1.right_stick_x; // Rotate

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
    }
}
