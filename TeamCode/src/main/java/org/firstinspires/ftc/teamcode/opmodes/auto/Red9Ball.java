//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
//
//import org.firstinspires.ftc.teamcode.utils.Paths;
//import org.firstinspires.ftc.teamcode.utils.Lebruxon;
//
//@Autonomous(preselectTeleOp="TeleOp")
//public class Red9Ball extends CommandOpMode {
//
//    Paths paths;
//
//    @Override
//    public void initialize() {
//        Lebruxon.init(hardwareMap, Lebruxon.MatchState.AUTO, Lebruxon.Alliance.RED);
//        paths = new Paths(Lebruxon.drivetrain.follower, Lebruxon.Alliance.RED);
//        Lebruxon.drivetrain.follower.setMaxPower(0.7);
//
//        schedule(new SequentialCommandGroup(
//
//                new InstantCommand(() -> {
//                    Lebruxon.intake.setPower(0.5);
//                    Lebruxon.intake.setMinPower(0.5);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.startToScore),
//
//                Lebruxon.shootOptimized(),
//
//                new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(0.5)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakeGPP1),
//                new InstantCommand(() -> Lebruxon.intake.setPower(1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakeGPP2),
//                new WaitCommand(500),
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(0.6);
//                    Lebruxon.intake.setPower(0.5);
//                    Lebruxon.intake.setMinPower(0.5);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.scoreGPP),
//
//                Lebruxon.shootOptimized(),
//
//                new InstantCommand(() -> Lebruxon.drivetrain.follower.setMaxPower(0.5)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakePGP1),
//                new InstantCommand(() -> Lebruxon.intake.setPower(1)),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.intakePGP2),
//                new WaitCommand(500),
//
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(0.6);
//                    Lebruxon.intake.setPower(0.5);
//                    Lebruxon.intake.setMinPower(0.5);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.scorePGP),
//
//                Lebruxon.shootOptimized(),
//
//
//                new InstantCommand(() -> {
//                    Lebruxon.drivetrain.follower.setMaxPower(0.6);
//                    Lebruxon.intake.setPower(0);
//                    Lebruxon.intake.setMinPower(0);
//                }),
//                new FollowPathCommand(Lebruxon.drivetrain.follower, paths.park)
//        ));
//    }
//
//
//    @Override
//    public void run() {
//        super.run();
//        Lebruxon.update();
//        telemetry.addData("turret angle", Math.toDegrees(Lebruxon.turret.getAngle()));
//        telemetry.addData("setpoint", Lebruxon.turret.controller.getSetPoint());
//        telemetry.addData("goal", Lebruxon.goal);
//        telemetry.update();
//    }
//}
