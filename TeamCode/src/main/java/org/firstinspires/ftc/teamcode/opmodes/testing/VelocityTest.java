package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

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
        shooter = new Shooter(hardwareMap);
        tele = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);


    }

    @Override
    public void run() {
        shooter.setHoodPercent(hood);
        shooter.controller.setP(P);
        shooter.controller.setF(F);
        shooter.setVelocity(velo);
        shooter.update();

        tele.addData("velocity", shooter.shooter1.getCorrectedVelocity());
        tele.addData("hoodPercent", shooter.hood.getRawPosition());
        tele.addData("raw velocity", shooter.shooter1.encoder.getRawVelocity());
        tele.addData("target", velo);
        tele.addData("power", shooter.shooter2.get());
        tele.update();
    }
}