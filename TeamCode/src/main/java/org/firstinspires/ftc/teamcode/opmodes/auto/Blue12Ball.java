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
//public class Blue12Ball extends CommandOpMode {
//
//    Paths paths;
//
//    @Override
//    public void initialize() {
//        Lebruxon.init(hardwareMap, Lebruxon.MatchState.AUTO, Lebruxon.Alliance.BLUE);
//        paths = new Paths(Lebruxon.drivetrain.follower, Lebruxon.Alliance.BLUE);
//        Lebruxon.drivetrain.follower.setMaxPower(0.8);
//
//        schedule(new SequentialCommandGroup(
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.startToScore),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(0.8);
//                    Lebruxon.intake.setPower(0);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakeGPP1),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0.8)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakeGPP2),
//                new WaitCommand(400),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                    //Mosby.intake.setPower(0.5);
//                    //Mosby.intake.setMinPower(0.5);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.scoreGPP),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                    Lebruxon.intake.setPower(0);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakePGP1),
//                new WaitCommand(150),
//                new InstantCommand(() -> {
//                    Lebruxon.intake.setPower(1);
//                    Lebruxon.drivetrain.follower.setMaxPower(0.8);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakePGP2),
//                new WaitCommand(400),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//
//
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                    //Mosby.intake.setPower(0.5);
//                   // Mosby.intake.setMinPower(0.5);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.scorePGP),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                    Lebruxon.intake.setPower(0);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakePPG1),
//                new WaitCommand(350),
//                new InstantCommand(() -> {
//                    Lebruxon.intake.setPower(1);
//                    Lebruxon.drivetrain.follower.setMaxPower(0.8);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakePPG2),
//                new WaitCommand(400),
//                new InstantCommand(() -> Lebruxon.intake.setPower(0)),
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                   // Mosby.intake.setPower(0.5);
//                    //Mosby.intake.setMinPower(0.5);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.scorePPG),
//                Lebruxon.prime(),
//                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
//                Lebruxon.shootWithIntake(),
//                Lebruxon.reset(),
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(1);
//                    Lebruxon.intake.setPower(0);
//                    Lebruxon.intake.setMinPower(0);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.park)
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
