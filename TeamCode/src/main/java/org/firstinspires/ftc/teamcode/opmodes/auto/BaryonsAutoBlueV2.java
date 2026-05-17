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
//import org.firstinspires.ftc.teamcode.utils.Paths;
//import org.firstinspires.ftc.teamcode.utils.Lebruxon;
//
//@Autonomous(preselectTeleOp="TeleOp")
//public class BaryonsAutoBlueV2 extends CommandOpMode {
//
//    Paths paths;
//
//    @Override
//    public void initialize() {
//        Lebruxon.init(hardwareMap, Lebruxon.MatchState.AUTO, Lebruxon.Alliance.BLUE);
//        paths = new Paths(Lebruxon.drivetrain.follower, Lebruxon.Alliance.BLUE);
//        Lebruxon.drivetrain.follower.setMaxPower(1);
//
//        schedule(new SequentialCommandGroup(
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.startToScore1),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                    Lebruxon.intake.setPower(1);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakepgp1),
//                new WaitCommand(200),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.checkIntakepgp1),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakepgp1again),
//                new WaitCommand(200),
//                new InstantCommand(() -> {
//                    Lebruxon.intake.setPower(0);
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                }),
//
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.shootpgp),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                    Lebruxon.intake.setPower(1);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakepgp1),
//                new WaitCommand(200),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.checkIntakepgp1),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakepgp1again),
//                new WaitCommand(200),
//                new InstantCommand(() -> {
//                    Lebruxon.intake.setPower(0);
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                }),
//
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.shootpgp),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakepgp1),
//                new WaitCommand(200),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.checkIntakepgp1),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakepgp1again),
//                new WaitCommand(200),
//                new InstantCommand(() -> {
//                    Lebruxon.intake.setPower(0);
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                }),
//
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.shootpgp),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
///**
// new InstantCommand(() -> Mosby.intake.setPower(1)),
// new FollowPathCommand(Mosby.drivetrain.follower, paths.intakepgp1),
// new WaitCommand(200),
// new FollowPathCommand(Mosby.drivetrain.follower, paths.checkIntakepgp1),
// new FollowPathCommand(Mosby.drivetrain.follower, paths.intakepgp1again),
// new WaitCommand(200),
// new InstantCommand(() -> {
// Mosby.intake.setPower(0);
// Mosby.drivetrain.follower.setMaxPower(1);
// }),
//
// new FollowPathCommand(Mosby.drivetrain.follower, paths.shootpgp),
// Mosby.prime(),
// new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
// Mosby.shootWithIntake(),
// Mosby.reset(),
// **/
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.park)
//        ));
//
//    }
//
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
