package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

@Configurable
@TeleOp
public class ServoTest extends OpMode {
    CRServo servo;
    CRServo servo2;
    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "turretLeft");
        servo2 = hardwareMap.get(CRServo.class, "turretRight");
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper){
            servo.setPower(1);
            servo2.setPower(1);
        }
        else if (gamepad1.right_bumper){
            servo.setPower(-1);
            servo2.setPower(-1);
        }
        else {
            servo.setPower(0);
            servo2.setPower(0);
        }
    }
}
