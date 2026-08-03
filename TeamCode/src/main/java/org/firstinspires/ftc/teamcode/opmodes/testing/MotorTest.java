package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@TeleOp
public class MotorTest extends OpMode {

    private DcMotorEx motor1;

    // GoBILDA 28 ticks/rev encoder
    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Right trigger = forward, Left trigger = reverse
        double power = gamepad1.right_trigger - gamepad1.left_trigger;

        motor1.setPower(power);

        // Velocity in encoder ticks per second
        double ticksPerSecond = motor1.getVelocity();

        // Convert to RPM
        double rpm = (ticksPerSecond / TICKS_PER_REV) * 60.0;

        telemetry.addData("Power", "%.2f", power);
        telemetry.addData("Velocity (ticks/sec)", "%.2f", ticksPerSecond);
        telemetry.addData("RPM", "%.2f", rpm);
        telemetry.addData("Encoder Position", motor1.getCurrentPosition());
        telemetry.update();
    }
}