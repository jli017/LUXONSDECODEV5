//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.command.WaitUntilCommand;
//import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
//
//import org.firstinspires.ftc.teamcode.utils.Paths2;
//import org.firstinspires.ftc.teamcode.utils.Lebruxon;
//
//@Autonomous(preselectTeleOp="TeleOp")
//public class Red24 extends CommandOpMode {
//
//    Paths2 paths2;
//
//    @Override
//    public void initialize() {
//        // Initialize for the new Red Square position
//        Lebruxon.init(hardwareMap, Lebruxon.MatchState.AUTO, Lebruxon.Alliance.REDSQ);
//        paths2 = new Paths2(Lebruxon.drivetrain.follower, Lebruxon.Alliance.REDSQ);
//
//        Lebruxon.drivetrain.follower.setMaxPower(1);
//
//        schedule(new SequentialCommandGroup(
//                // preloaded 3
//                Lebruxon.reset(),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.ScorePre),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                new WaitCommand(125),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                // intake middle
//                new InstantCommand(() -> Lebruxon.intake.setPower(1)),
//                new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.collect1),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.Score1),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                new WaitCommand(150),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                // gate 1
//                // new SequentialCommandGroup(
//                new InstantCommand(() -> Lebruxon.intake.setPower(1)),
//                new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(0.8)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.collect2,true),
//                new WaitUntilCommand(() -> !Lebruxon.drivetrain.follower.isBusy()),
//                // ),
//
//                new WaitCommand(2000),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//                new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.score2),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                new WaitCommand(125),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                // gate
//                // new SequentialCommandGroup(
//                new InstantCommand(() -> Lebruxon.intake.setPower(1)),
//                new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(0.8)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.collect3, true),
//                new WaitUntilCommand(() -> !Lebruxon.drivetrain.follower.isBusy()),
//                //),
//
//                new WaitCommand(2200),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//                new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.score3),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                new WaitCommand(125),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                // last row intake
//                new InstantCommand(() -> Lebruxon.intake.setPower(1)),
//                new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(0.95)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.collect4),
//                new WaitCommand(100),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.score4),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                new WaitCommand(125),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                new InstantCommand(() -> Lebruxon.intake.setPower(1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.collect5),
//                new WaitCommand(100),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.score5),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                new WaitCommand(125),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//                new InstantCommand(() -> Lebruxon.shooter.autoPower(false,false)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths2.leave)
//
//        ));
//    }
//    @Override
//    public void run() {
//        super.run();
//        Lebruxon.update();
//        telemetry.addData("turret angle", Math.toDegrees(Lebruxon.turret.getAngle()));
//        telemetry.addData("setpoint", Lebruxon.turret.controller.getSetPoint());
//        telemetry.addData("goal", Lebruxon.goal);
//        telemetry.addData("shooter current velocity", Lebruxon.shooter.getVelocity());
//        telemetry.addData("shooter set Velocity", Lebruxon.shooter.power);
//        telemetry.update();
//    }
//}