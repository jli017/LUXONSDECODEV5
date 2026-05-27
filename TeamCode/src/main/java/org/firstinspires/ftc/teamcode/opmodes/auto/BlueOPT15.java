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
    public class BlueOPT15 extends CommandOpMode {

        Paths paths;

        @Override
        public void initialize() {
            Lebruxon.init(hardwareMap, Lebruxon.MatchState.AUTO, Lebruxon.Alliance.BLUESQ);
            paths = new Paths(Lebruxon.drivetrain.follower, Lebruxon.Alliance.BLUESQ);
            Lebruxon.drivetrain.follower.setMaxPower(1);


            Lebruxon.turret.enableAim = true;
            Lebruxon.shooter.idle = true;

            schedule(new SequentialCommandGroup(

                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.ClosestartToScore),
                    Lebruxon.prime(),
                    new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                    Lebruxon.shootWithIntake(),
                    Lebruxon.reset(),

                    new InstantCommand(() -> {
                        Lebruxon.drivetrain.follower.setMaxPower(1);
                        Lebruxon.intake.setPower(1, 1);
                    }),
                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CloseintakePGP1),
                    new WaitCommand(50),
                    new InstantCommand(() -> Lebruxon.intake.setPower(0, 0)),
                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.turn2),


                    new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(1)),
                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.ClosescorePGP),
                    Lebruxon.prime(),
                    new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                    Lebruxon.shootWithIntake(),
                    Lebruxon.reset(),

                    new InstantCommand(() -> {
                        Lebruxon.drivetrain.follower.setMaxPower(1);
                        Lebruxon.intake.setPower(1, 1);
                    }),
                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CloseIntakeG1),
                    new WaitCommand(50),
                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CloseIntakeTurn),
                    new WaitCommand(800),

                    new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(1)),
                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.ClosescoreG1),
                    new InstantCommand(() -> Lebruxon.intake.setPower(0, 0)),
                    Lebruxon.prime(),
                    new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                    Lebruxon.shootWithIntake(),
                    Lebruxon.reset(),

                    new InstantCommand(() -> {
                        Lebruxon.drivetrain.follower.setMaxPower(1);
                        Lebruxon.intake.setPower(1, 1);
                    }),
                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CloseIntakeG1),
                    new WaitCommand(50),
                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CloseIntakeTurn),
                    new WaitCommand(800),
                   // new FollowPathCommand(Lebruxon.drivetrain.follower, paths.turn1),
                    new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(1)),
                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.ClosescoreG1),
                    new InstantCommand(() -> Lebruxon.intake.setPower(0, 0)),
                    Lebruxon.prime(),
                    new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                    Lebruxon.shootWithIntake(),
                    Lebruxon.reset(),

                    new InstantCommand(() -> {
                        Lebruxon.drivetrain.follower.setMaxPower(1);
                        Lebruxon.intake.setPower(1,1);
                    }),
                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.Closeintakelast),
                    new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(1)),
                    new FollowPathCommand(Lebruxon.drivetrain.follower, paths.Closescorefinal),
                    new InstantCommand(() -> Lebruxon.intake.setPower(0, 0)),
                    Lebruxon.prime(),
                    new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                    Lebruxon.shootWithIntake(),
                    Lebruxon.reset(),

                    new InstantCommand(() -> {
                        Lebruxon.drivetrain.follower.setMaxPower(1);
                        Lebruxon.intake.setPower(0,0);
                        Lebruxon.intake.setMinPower(0);
                    }),
                    new InstantCommand(() -> Lebruxon.shooter.autoPower(false,false))
                   // new FollowPathCommand(Lebruxon.drivetrain.follower, paths.Closepark)



            ));
        }

        @Override
        public void run() {
            super.run();
            Lebruxon.update();
            telemetry.addData("turret angle", Math.toDegrees(Lebruxon.turret.getAngle()));
            telemetry.addData("setpoint", Lebruxon.turret.controller.getSetPoint());
            telemetry.addData("goal", Lebruxon.goal);
            telemetry.addData("shooter current velocity", Lebruxon.shooter.getVelocity());
            telemetry.addData("shooter set Velocity", Lebruxon.shooter.power);
            telemetry.update();
        }
    }
