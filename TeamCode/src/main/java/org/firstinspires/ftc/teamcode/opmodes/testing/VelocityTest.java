package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.Lebruxon;
import org.firstinspires.ftc.teamcode.utils.Storage;
import org.firstinspires.ftc.teamcode.utils.subsystems.Shooter;

@TeleOp
@Configurable
public class VelocityTest extends CommandOpMode {
    Shooter shooter;
    public static double hood = 1;
    public static double velo = 0;

    public static double P = 0.001;
    public static double F = 0.000385;

    JoinedTelemetry tele;

    @Override
    public void initialize() {
        tele = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        Lebruxon.init(hardwareMap, Lebruxon.MatchState.TELEOP, Storage.alliance);

        Lebruxon.shooter.resetHood();
        Lebruxon.shooter.closeStopper();

        Command shootWithIntake = Lebruxon.shootWithIntake();

        GamepadEx samai = new GamepadEx(gamepad1);

        samai.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shootWithIntake);

        Lebruxon.turret.enableAim = true;
    }

    @Override
    public void run() {
        Lebruxon.shooter.setHoodPercent(hood);
        Lebruxon.shooter.setVelocity(velo);
        Lebruxon.update();

        double intakePower = gamepad1.left_trigger;
        double transferPower = gamepad1.right_trigger;

        Lebruxon.intake.setPower(intakePower, transferPower);

        tele.addData("velocity", Lebruxon.shooter.getVelocity());
        tele.addData("target", velo);
        tele.update();
    }
}