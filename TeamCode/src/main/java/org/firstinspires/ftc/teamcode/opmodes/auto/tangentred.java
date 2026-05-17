//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitUntilCommand;
//import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
//
//import org.firstinspires.ftc.teamcode.utils.Paths2;
//import org.firstinspires.ftc.teamcode.utils.Lebruxon;
//@Autonomous(preselectTeleOp="TeleOp")
//public class tangentred extends CommandOpMode {
//    Paths2 paths;
//
//    @Override
//    public void initialize() {
//        Lebruxon.init(hardwareMap, Lebruxon.MatchState.AUTO, Lebruxon.Alliance.RED);
//        paths = new Paths2(Lebruxon.drivetrain.follower, Lebruxon.Alliance.RED);
//        Lebruxon.drivetrain.follower.setMaxPower(0.9);
//
//        schedule(new SequentialCommandGroup(
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.ScorePre),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                new InstantCommand(() -> Lebruxon.intake.setPower(1, 1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CollectLastRow),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0, 0)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.ScoreLastRow),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                new InstantCommand(() -> Lebruxon.intake.setPower(1, 1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CollectHP1),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.HP1_to_HP2),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.HP2_to_HP1),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.HP1_to_Score),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                new InstantCommand(() -> Lebruxon.intake.setPower(1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CollectHP1),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.HP1_to_HP2),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.HP2_to_HP1),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.HP1_to_Score),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                new InstantCommand(() -> Lebruxon.intake.setPower(1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CollectHP1),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.HP1_to_HP2),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.HP2_to_HP1),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.HP1_to_Score),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                new InstantCommand(() -> Lebruxon.intake.setPower(1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CollectOffHP),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.OffHP1_to_OffHP2),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.OffHP2_to_OffHP1),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.HP1_to_Score), // Back to shoot2
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset()
//        ));
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        Lebruxon.update();
//    }
//}
