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
import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import org.firstinspires.ftc.teamcode.utils.Lebruxon;
import org.firstinspires.ftc.teamcode.utils.Storage;
import org.firstinspires.ftc.teamcode.utils.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.subsystems.Turret;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Configurable
public class TeleOp extends CommandOpMode {

    public static double increment = 0.0175;

    @Override
    public void initialize() {
        Lebruxon.init(hardwareMap, Lebruxon.MatchState.TELEOP, Storage.alliance);

        // FIX: enableAim was never set to true after the broken
        // CommandScheduler.schedule(turret.enableAim = true) line was removed.
        Lebruxon.turret.enableAim = false;

        Lebruxon.update();

        Command prime = Lebruxon.prime();
        Command shoot = Lebruxon.shoot();
        Command shootWithIntake = Lebruxon.shootWithIntake();

        GamepadEx samai = new GamepadEx(gamepad1);
        GamepadEx jonathan = new GamepadEx(gamepad2);

        // samai controls
        samai.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                        .whenPressed(new InstantCommand(() -> {
                            Lebruxon.shooter.idle = !Lebruxon.shooter.idle;
                        }));
        samai.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(prime);
        samai.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            prime.cancel();
                            shoot.cancel();
                            shootWithIntake.cancel();
                            Lebruxon.shooter.closeStopper();
                        }),
                        Lebruxon.reset()
                ));

        samai.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shootWithIntake);

        samai.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (Storage.alliance == Lebruxon.Alliance.BLUE || Storage.alliance == Lebruxon.Alliance.BLUECLOSE) {
                        Pose b = new Pose(135.5, 9, Math.toRadians(90));
                        Lebruxon.drivetrain.follower.setPose(b);
                        Storage.pose = b;
                    } else {
                        Pose r = new Pose(8.5, 9, Math.toRadians(90));
                        Lebruxon.drivetrain.follower.setPose(r);
                        Storage.pose = r;
                    }
                }));

        samai.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    Lebruxon.turret.enableAim = !Lebruxon.turret.enableAim;
                }));

        // FIX: homePos adjustments now use getNormalizedAngle() — always in [0, 2PI) —
        // instead of controller.getSetPoint(), which was unbounded and corrupted homePos
        // whenever pidSetpoint drifted outside the normalized range.
        jonathan.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            Turret.encoderTrim = Turret.wrapToTwoPi(Turret.encoderTrim - increment);
        }));

        jonathan.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            Turret.encoderTrim = Turret.wrapToTwoPi(Turret.encoderTrim + increment);
        }));

        // FIX: Preserve enableAim across re-init so a DPAD_UP re-init doesn't silently
        // reset turret state.
        jonathan.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> {
            boolean savedAim = Lebruxon.turret.enableAim;
            double savedHome = Turret.homePos;
            Lebruxon.init(hardwareMap, Lebruxon.MatchState.TELEOP, Storage.alliance);
            Lebruxon.turret.enableAim = savedAim;
            Turret.homePos = savedHome;
        }));

        jonathan.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(() -> {
            Lebruxon.shooter.add -= 100;
        }));
        jonathan.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(() -> {
            Lebruxon.shooter.add += 100;
        }));

        jonathan.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(new InstantCommand(() -> {
            Lebruxon.shooter.add = 0;
        }));

        Lebruxon.shooter.resetHood();
        Lebruxon.shooter.closeStopper();
    }

    public void run() {
        super.run();
        Lebruxon.update();
        Lebruxon.drivetrain.drive(gamepad1);

        double intakePower, transferPower;

        if (!gamepad1.cross) {
            if (gamepad1.right_trigger < 0.2 && gamepad1.left_trigger > 0.2) {
                intakePower = gamepad1.left_trigger;
                transferPower = gamepad1.right_trigger;
            } else {
                intakePower = gamepad1.right_trigger;
                transferPower = gamepad1.right_trigger;
            }
            Lebruxon.intake.setPower(intakePower, transferPower);
        }
        else {
            Lebruxon.intake.setPower(-0.8, -0.8);
        }

        if (Lebruxon.intake.dist < 3){
            gamepad1.rumble(300);
        }

        Drivetrain.turbo = gamepad1.left_stick_button;

        telemetry.addData("turret angle (deg) ",   Math.toDegrees(Lebruxon.turret.getNormalizedAngle()));
        telemetry.addData("turret target (deg) ",  Math.toDegrees(Lebruxon.turret.getTargetAngle()));
        telemetry.addData("turret enableAim ",     Lebruxon.turret.enableAim);
        telemetry.addData("turret homePos (deg) ", Math.toDegrees(Lebruxon.turret.homePos));
        telemetry.addData("shooter error ",        Lebruxon.shooter.controller.getPositionError());
        telemetry.addData("robot x ",              Lebruxon.drivetrain.follower.getPose().getX());
        telemetry.addData("robot y ",              Lebruxon.drivetrain.follower.getPose().getY());
        telemetry.addData("heading (deg) ",        Math.toDegrees(Lebruxon.drivetrain.follower.getPose().getHeading()));
        telemetry.addData("distance ",             Lebruxon.shooter.distance);
        telemetry.addData("shooter setpoint ",     Lebruxon.shooter.controller.getSetPoint());
        telemetry.addData("shooter atSetPoint ",   Lebruxon.shooter.controller.atSetPoint());
        telemetry.addData("shooter velo ",         Lebruxon.shooter.getVelocity());
        telemetry.addData("turret pos deg ",    Math.toDegrees(Lebruxon.turret.getNormalizedAngle()));
        telemetry.addData("turret target deg ", Math.toDegrees(Lebruxon.turret.getTargetAngle()));
        telemetry.addData("inDeadzone ",        Lebruxon.turret.getNormalizedAngle() > Math.toRadians(240) && Lebruxon.turret.getNormalizedAngle() < Math.toRadians(290));
        telemetry.addData("velo add ", Lebruxon.shooter.add);
        telemetry.update();
    }
}
