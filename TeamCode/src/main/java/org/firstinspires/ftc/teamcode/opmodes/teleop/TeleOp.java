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

    private GamepadEx samai;
    private GamepadEx jonathan;

    @Override
    public void initialize() {
        Lebruxon.init(hardwareMap, Lebruxon.MatchState.TELEOP, Storage.alliance);

        Lebruxon.turret.enableAim = false;

        Lebruxon.update();

        Command prime = Lebruxon.prime();
        Command shoot = Lebruxon.shoot();
        Command shootWithIntake = Lebruxon.shootWithIntake();

        samai = new GamepadEx(gamepad1);
        jonathan = new GamepadEx(gamepad2);

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

        jonathan.getGamepadButton(GamepadKeys.Button.SHARE)
                .whenPressed(new InstantCommand(() -> {
                    if (Storage.alliance == Lebruxon.Alliance.BLUE || Storage.alliance == Lebruxon.Alliance.BLUECLOSE || Storage.alliance == Lebruxon.Alliance.BLUESQ) {
                        if (Lebruxon.shooter.distance > 100) {
                            Pose b = new Pose(135.5, 9, Math.toRadians(90));
                            Lebruxon.drivetrain.follower.setPose(b);
                            Storage.pose = b;
                        }
                        else {
                            Pose b = new Pose(33, 131, Math.toRadians(90));
                            Lebruxon.drivetrain.follower.setPose(b);
                            Storage.pose = b;
                        }
                    } else {
                        if (Lebruxon.shooter.distance > 100) {
                            Pose r = new Pose(8.5, 9, Math.toRadians(90));
                            Lebruxon.drivetrain.follower.setPose(r);
                            Storage.pose = r;
                        }
                        else {
                            Pose b = new Pose(111, 131, Math.toRadians(90));
                            Lebruxon.drivetrain.follower.setPose(b);
                            Storage.pose = b;
                        }
                    }
                }));

        // Blocks Samai from re-enabling aim while Jonathan's manual home
        // adjustment is still unlocked, so an in-progress adjustment can't
        // be silently abandoned by aim snapping back on mid-jog.
        samai.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (!Lebruxon.turret.enableAim && Lebruxon.turret.homeAdjustUnlocked) {
                        return;
                    }
                    Lebruxon.turret.enableAim = !Lebruxon.turret.enableAim;
                }));

        jonathan.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> {
            boolean savedAim = Lebruxon.turret.enableAim;
            double savedHome = Turret.homePos;
            Lebruxon.init(hardwareMap, Lebruxon.MatchState.TELEOP, Storage.alliance);
            Lebruxon.turret.enableAim = savedAim;
            Turret.homePos = savedHome;
            Lebruxon.turret.homeAdjustUnlocked = false;
        }));

        // FIX: Jonathan's dpad-down now owns both ends of the manual reset
        // sequence instead of requiring aim to already be off.
        //   1st press: force aim off and unlock — triggers below can now
        //     freely jog the turret without the PD loop fighting them.
        //   2nd press: zero the encoder at wherever it's been jogged to,
        //     make that the new homePos, and relock — handing control back
        //     to Samai's DPAD_DOWN, which was blocked above while unlocked.
        jonathan.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> {
            if (!Lebruxon.turret.homeAdjustUnlocked) {
                Lebruxon.turret.enableAim = false;
                Lebruxon.turret.homeAdjustUnlocked = true;
            } else {
                Lebruxon.turret.setHomeToCurrentPosition();
                Lebruxon.turret.homeAdjustUnlocked = false;
            }
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

        if (!gamepad1.square) {
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

        Drivetrain.lock = gamepad1.cross;

        if (!Lebruxon.turret.enableAim) {
            double cw = jonathan.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            double ccw  = jonathan.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            Lebruxon.turret.manualPower = (cw * 0.5) - (ccw * 0.5);
        } else {
            Lebruxon.turret.manualPower = 0;
        }

        telemetry.addData("turret enableAim ",     Lebruxon.turret.enableAim);
        telemetry.addData("turret homeAdjustUnlocked ", Lebruxon.turret.homeAdjustUnlocked);
        telemetry.addData("turret manual power ",  Lebruxon.turret.manualPower);
        telemetry.addData("turret target vel (deg/s) ", Math.toDegrees(Lebruxon.turret.lastTargetAngularVelocity));
        telemetry.addData("robot x ",              Lebruxon.drivetrain.follower.getPose().getX());
        telemetry.addData("robot y ",              Lebruxon.drivetrain.follower.getPose().getY());
        telemetry.addData("heading (deg) ",        Math.toDegrees(Lebruxon.drivetrain.follower.getPose().getHeading()));
        telemetry.addData("distance ",             Lebruxon.shooter.distance);
        telemetry.addData("shooter setpoint ",     Lebruxon.shooter.controller.getSetPoint());
        telemetry.addData("shooter atSetPoint ",   Lebruxon.shooter.controller.atSetPoint());
        telemetry.addData("shooter velo ",         Lebruxon.shooter.getVelocity());
        telemetry.addData("velo add ", Lebruxon.shooter.add);
        telemetry.update();
    }
}