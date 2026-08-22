package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.utils.Paths;
import org.firstinspires.ftc.teamcode.utils.Lebruxon;

@Autonomous(preselectTeleOp="TeleOp")
public class FarBlue extends CommandOpMode {

    Paths paths;

    @Override
    public void initialize() {
        Lebruxon.init(hardwareMap, Lebruxon.MatchState.AUTO, Lebruxon.Alliance.BLUE);
        paths = new Paths(Lebruxon.drivetrain.follower, Lebruxon.Alliance.BLUE);
        Lebruxon.drivetrain.follower.setMaxPower(1);

        Lebruxon.turret.enableAim = true;
        Lebruxon.shooter.idle = true;

        schedule(new SequentialCommandGroup(
                Lebruxon.prime(),
                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                Lebruxon.shootWithIntake(),
                Lebruxon.reset(),
                //
                new InstantCommand(() -> {
                    Lebruxon.drivetrain.follower.setMaxPower(1);
                    Lebruxon.intake.setPower(1, 1);
                }),
                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakePPG1),
                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakePPG2),
                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.scorePPG),
                new InstantCommand(() -> {
                    Lebruxon.intake.setPower(0, 0);
                    Lebruxon.drivetrain.follower.setMaxPower(1);
                }),
                Lebruxon.prime(),
                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                Lebruxon.shootWithIntake(),
                Lebruxon.reset(),

                new InstantCommand(() -> {
                    Lebruxon.drivetrain.follower.setMaxPower(1);
                    Lebruxon.intake.setPower(1, 1);
                }),
                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakepgp1),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    Lebruxon.intake.setPower(0, 0);
                    Lebruxon.drivetrain.follower.setMaxPower(1);
                }),

                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.shootpgp),
                Lebruxon.prime(),
                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                Lebruxon.shootWithIntake(),
                new InstantCommand(() -> {
                    Lebruxon.drivetrain.follower.setMaxPower(1);
                    Lebruxon.intake.setPower(1, 1);
                }),
                Lebruxon.reset(),

                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.swipefirst),
                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.swipefar),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    Lebruxon.intake.setPower(0, 0);
                    Lebruxon.drivetrain.follower.setMaxPower(1);
                }),
                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.swipelast),
                Lebruxon.prime(),
                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                Lebruxon.shootWithIntake(),
                new InstantCommand(() -> {
                    Lebruxon.drivetrain.follower.setMaxPower(1);
                    Lebruxon.intake.setPower(1, 1);
                }),
                Lebruxon.reset(),

                new InstantCommand(() -> {
                    Lebruxon.drivetrain.follower.setMaxPower(1);
                    Lebruxon.intake.setPower(1, 1);
                }),
                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakepgp1),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    Lebruxon.intake.setPower(0, 0);
                    Lebruxon.drivetrain.follower.setMaxPower(1);
                }),

                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.shootpgp),
                Lebruxon.prime(),
                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                Lebruxon.shootWithIntake(),
                new InstantCommand(() -> {
                    Lebruxon.drivetrain.follower.setMaxPower(1);
                    Lebruxon.intake.setPower(1, 1);
                }),
                Lebruxon.reset(),

                new InstantCommand(() -> Lebruxon.shooter.idle = false),
                new InstantCommand(() -> Lebruxon.turret.enableAim = false),

                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.park)
        ));

    }

    @Override
    public void run() {
        super.run();
        Lebruxon.update();
        telemetry.addData("goal", Lebruxon.goal);
        telemetry.addData("shooter current velocity", Lebruxon.shooter.getVelocity());
        telemetry.addData("shooter set Velocity", Lebruxon.shooter.power);
        telemetry.update();
    }
}
