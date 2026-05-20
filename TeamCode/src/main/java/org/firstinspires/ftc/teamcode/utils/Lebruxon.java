package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.utils.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.subsystems.Turret;

import java.util.concurrent.atomic.AtomicBoolean;

@Configurable
public class Lebruxon {
    public enum MatchState {
        AUTO,
        TELEOP
    }

    public enum Alliance {
        RED,
        BLUE,

        REDCLOSE,
        BLUECLOSE,
        BLUESQ,

        REDSQ
    }

    public static final Pose BLUE_START_POSE = new Pose(56, 7, Math.toRadians(180));
    public static final Pose BLUE_SQ_START_POSE = new Pose(33, 135, Math.toRadians(90));
    public static final Pose RED_SQ_START_POSE = new Pose(144 - BLUE_SQ_START_POSE.getX(), BLUE_SQ_START_POSE.getY(), Math.toRadians(0));
    public static final Pose CLOSE_BLUE_START_POSE = new Pose(21, 123, Math.toRadians(144));
    public static final Pose RED_START_POSE = new Pose(144 - BLUE_START_POSE.getX(), BLUE_START_POSE.getY(), Math.toRadians(0));
    public static final Pose CLOSE_RED_START_POSE = new Pose(144 - CLOSE_BLUE_START_POSE.getX(), CLOSE_BLUE_START_POSE.getY(), Math.toRadians(36));
    public static final Vector2d BLUE_GOALPIDF = new Vector2d(15, 141);
    public static final Vector2d RED_GOALPIDF = new Vector2d(128, 141);
    public static final Vector2d BLUE_GOAL = new Vector2d(2, 144);
    public static final Vector2d RED_GOAL = new Vector2d(140, 144);
    public static MatchState matchState;
    public static Alliance alliance;
    public static Drivetrain drivetrain;
    public static Turret turret;
    public static Intake intake;
    public static Shooter shooter;
    public static Pose startPose;
    public static Vector2d goal;
    public static Vector2d goalShooter;

    public static int failsafeDelay = 100;
    public static int flywheelThreshhold = 100;
    public static double primeIntakeSpeed1 = 0.8;
    public static double primeIntakeSpeed2 = 0.5;
    public static double primeIntakeSpeed3 = 0;

    public static void init(HardwareMap hardwareMap, MatchState matchState, Alliance alliance) {
        Lebruxon.matchState = matchState;
        Lebruxon.alliance = alliance;
        //Mosby.startPose = alliance == Alliance.RED? RED_START_POSE : BLUE_START_POSE;
        //Mosby.goal = alliance == Alliance.RED? RED_GOAL : BLUE_GOAL;
        //Mosby.goalShooter = alliance == Alliance.RED? RED_GOALPIDF : BLUE_GOALPIDF;
        switch (alliance) {
            case RED:
                Lebruxon.startPose = RED_START_POSE;
                Lebruxon.goal = RED_GOAL;
                Lebruxon.goalShooter = RED_GOALPIDF;
                break;

            case BLUE:
                Lebruxon.startPose = BLUE_START_POSE;
                Lebruxon.goal = BLUE_GOAL;
                Lebruxon.goalShooter = BLUE_GOALPIDF;
                break;

            case REDCLOSE:
                Lebruxon.startPose = CLOSE_RED_START_POSE;
                Lebruxon.goal = RED_GOAL;
                Lebruxon.goalShooter = RED_GOALPIDF;
                break;

            case BLUECLOSE:
                Lebruxon.startPose = CLOSE_BLUE_START_POSE;
                Lebruxon.goal = BLUE_GOAL;
                Lebruxon.goalShooter = BLUE_GOALPIDF;
                break;

            case BLUESQ:
                Lebruxon.startPose = BLUE_SQ_START_POSE;
                Lebruxon.goal = BLUE_GOAL;
                Lebruxon.goalShooter = BLUE_GOALPIDF;
                break;

            case REDSQ:
                Lebruxon.startPose = RED_SQ_START_POSE;
                Lebruxon.goal = RED_GOAL;
                Lebruxon.goalShooter = RED_GOALPIDF;
                break;
        }


        drivetrain = new Drivetrain(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);


        Storage.alliance = alliance;

        Lebruxon.drivetrain.follower.setStartingPose(matchState == MatchState.AUTO ? startPose : Storage.pose);
        Lebruxon.drivetrain.follower.update();

        CommandScheduler.getInstance().registerSubsystem(drivetrain, turret, intake, shooter);

        CommandScheduler.getInstance().schedule(turret.enableAim = true);
    }

    public static void update() {
        drivetrain.update();
        turret.update();
        intake.update();
        shooter.update();
    }

    public static InstantCommand reset() {
        return new InstantCommand(() -> {
            intake.setMinPower(0);
            shooter.controller.reset();
            shooter.autoPower(false, false);
            shooter.setVelocity(Shooter.idleVeloMultiplier);
            shooter.closeStopper();
            shooter.resetHood();
        });
    }

    public static InstantCommand prime() {
        return new InstantCommand(() -> {
            //intake.setPower(0, 0);
            intake.setMinPower(0);

            //shooter.openStopper();
            shooter.autoPower(true, true); // ONLY place
        });
    }

    public static Command shoot() {
        AtomicBoolean usedTimeout = new AtomicBoolean(true);
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                new InstantCommand(() -> {
                    shooter.openStopper();
                }),

                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    intake.setPower(1, 1);
                                    intake.setMinPower(1);
                                }),
                                new WaitCommand(300),
                                new InstantCommand(() -> {
                                    intake.setPower(0, 0);
                                    intake.setMinPower(0);
                                })
                        ),
                        new InstantCommand(),
                        usedTimeout::get
                )
        );
    }

    public static Command shootWithIntake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    shooter.openStopper();
                }),

                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),

                new ConditionalCommand(
                        // TRUE branch
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    intake.setPower(1, 1);
                                    intake.setMinPower(1);
                                }),
                                new WaitCommand(100),
                                new InstantCommand(() -> {
                                    shooter.setCurrentHoodPercent(0.8);
                                }),
                                new WaitCommand(150),
                                new InstantCommand(() -> {
                                    shooter.setCurrentHoodPercent(0.8);
                                }),
                                new WaitCommand(130),
                                new InstantCommand(() -> {
                                }),
                                new WaitCommand(100),
                                new InstantCommand(() -> {
                                    intake.setPower(0, 0);
                                    intake.setMinPower(0);
                                }),
                                reset()
                        ),

                        // FALSE branch (do nothing)
                        new InstantCommand(),

                        // condition
                        () -> Lebruxon.shooter.controller.atSetPoint()
                )
        );
    }

}
