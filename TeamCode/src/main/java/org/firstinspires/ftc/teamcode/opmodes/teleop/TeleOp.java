package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.Lebruxon;
import org.firstinspires.ftc.teamcode.utils.Storage;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Configurable
public class TeleOp extends CommandOpMode {

    public static double increment = 0.0875;

    @Override
    public void initialize() {
        Lebruxon.init(hardwareMap, Lebruxon.MatchState.TELEOP, Storage.alliance);
        Lebruxon.update();

        Command prime = Lebruxon.prime();
        Command shoot = Lebruxon.shoot();
        Command shootWithIntake = Lebruxon.shootWithIntake();


        GamepadEx samai = new GamepadEx(gamepad1);
        GamepadEx jonathan = new GamepadEx(gamepad2);

        // samai controls
        samai.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(prime);
        samai.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed( new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            prime.cancel();
                            shoot.cancel();
                            shootWithIntake.cancel();
                        }),
                        Lebruxon.reset()
                ));

        samai.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shoot);

        samai.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if(Storage.alliance == Lebruxon.Alliance.BLUE || Storage.alliance == Lebruxon.Alliance.BLUESQ) {
                        Pose b = new Pose(135.5, 7.8125, Math.toRadians(90));
                        Lebruxon.drivetrain.follower.setPose(b);
                        Storage.pose = b;

                    } else {
                        Pose r = new Pose(8.5, 7.8125, Math.toRadians(90));
                        Lebruxon.drivetrain.follower.setPose(r);
                        Storage.pose = r;
                    }
                }) );

        samai.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (!Lebruxon.turret.enableAim) {
                        Lebruxon.turret.enableAim = true;
                    } else {
                        Lebruxon.turret.enableAim = false;
                    }
                }) );

        jonathan.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            double pos = Lebruxon.turret.controller.getSetPoint();
            Lebruxon.turret.homePos = pos - increment;
        }));

        jonathan.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            double pos = Lebruxon.turret.controller.getSetPoint();
            Lebruxon.turret.homePos = pos + increment;
        }));
        jonathan.getGamepadButton(GamepadKeys.Button.START).whenPressed(new InstantCommand(() -> {
            Lebruxon.init(hardwareMap, Lebruxon.MatchState.TELEOP, Storage.alliance);
        }));
    }

    public void run() {
        super.run();
        Lebruxon.update();
        Lebruxon.drivetrain.drive(gamepad1);
        double intakePower = gamepad1.left_trigger;
        double transferPower = gamepad1.right_trigger;

        Lebruxon.intake.setPower(intakePower, transferPower);

        telemetry.addData("error", Lebruxon.shooter.controller.getPositionError());
        telemetry.addData("position", Lebruxon.drivetrain.follower.getPose().getX());
        telemetry.addData("position", Lebruxon.drivetrain.follower.getPose().getY());
        telemetry.addData("heading", Lebruxon.drivetrain.follower.getPose().getHeading());
        telemetry.addData("goal", Lebruxon.goalShooter.getX());
        telemetry.addData("goal", Lebruxon.goalShooter.getY());
        telemetry.addData("distance", Lebruxon.shooter.distance);
        telemetry.addData("ActualVelo", Lebruxon.shooter.controller.getSetPoint());
        telemetry.addData("atSetPoint", Lebruxon.shooter.controller.atSetPoint());
        telemetry.addData("velo", Lebruxon.shooter.getVelocity());
        telemetry.addData("storage angle", Storage.turretAngle);
        telemetry.update();
    }


}