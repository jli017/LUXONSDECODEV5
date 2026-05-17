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
//public class BlueOPT12 extends CommandOpMode {
//
//    Paths paths;
//
//    @Override
//    public void initialize() {
//        Lebruxon.init(hardwareMap, Lebruxon.MatchState.AUTO, Lebruxon.Alliance.BLUECLOSE);
//        paths = new Paths(Lebruxon.drivetrain.follower, Lebruxon.Alliance.BLUECLOSE);
//        Lebruxon.drivetrain.follower.setMaxPower(0.8);
//
//
//        schedule(new SequentialCommandGroup(
//
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.ClosestartToScore),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(0.8);
//                    Lebruxon.intake.setPower(1);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CloseintakePPG),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//
//
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.ClosescorePPG),
//                Lebruxon.prime(),
//                new WaitCommand(500),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                    Lebruxon.intake.setPower(1);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CloseintakePGP1),
//                new WaitCommand(100),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.turn1),
//
//
//                new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.ClosescorePGP),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                    Lebruxon.intake.setPower(1);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CloseintakeGPP1),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.CloseintakeGPP2),
//                new WaitCommand(400),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//
//
//                new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.ClosescoreGPP),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                    Lebruxon.intake.setPower(0);
//                    Lebruxon.intake.setMinPower(0);
//                }),
//                new InstantCommand(() -> Lebruxon.shooter.autoPower(false,false)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.Closepark)
//        ));
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
