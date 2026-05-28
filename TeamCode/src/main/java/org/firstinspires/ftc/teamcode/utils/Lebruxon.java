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

    public static final Pose BLUE_START_POSE       = new Pose(56, 8.5, Math.toRadians(180));
    public static final Pose BLUE_SQ_START_POSE    = new Pose(33, 131, Math.toRadians(90));
    public static final Pose RED_SQ_START_POSE     = new Pose(144 - BLUE_SQ_START_POSE.getX(), BLUE_SQ_START_POSE.getY(), Math.toRadians(0));
    public static final Pose CLOSE_BLUE_START_POSE = new Pose(21, 123, Math.toRadians(144));
    public static final Pose RED_START_POSE        = new Pose(144 - BLUE_START_POSE.getX(), BLUE_START_POSE.getY(), Math.toRadians(0));
    public static final Pose CLOSE_RED_START_POSE  = new Pose(144 - CLOSE_BLUE_START_POSE.getX(), CLOSE_BLUE_START_POSE.getY(), Math.toRadians(36));
    public static final Vector2d BLUE_GOALPIDF     = new Vector2d(15.5, 129);
    public static final Vector2d RED_GOALPIDF      = new Vector2d(128.5, 129);
    public static final Vector2d BLUE_GOAL         = new Vector2d(0, 144);
    public static final Vector2d RED_GOAL          = new Vector2d(144, 144);
    public static final Vector2d BLUE_GOAL_CLOSE         = new Vector2d(7, 138);
    public static final Vector2d RED_GOAL_CLOSE          = new Vector2d(137, 138);
    public static final Vector2d BLUE_GOAL_FAR         = new Vector2d(7, 141);
    public static final Vector2d RED_GOAL_FAR         = new Vector2d(137, 141);

    public static MatchState matchState;
    public static Alliance   alliance;
    public static Drivetrain drivetrain;
    public static Turret     turret;
    public static Intake     intake;
    public static Shooter    shooter;
    public static Pose       startPose;
    public static Vector2d   goal;
    public static Vector2d   goalShooter;
    public static Vector2d targetClose;
    public static Vector2d targetFar;


    public static int    failsafeDelay      = 100;
    public static int    flywheelThreshhold = 100;
    public static double primeIntakeSpeed1  = 0.8;
    public static double primeIntakeSpeed2  = 0.5;
    public static double primeIntakeSpeed3  = 0;

    public static void init(HardwareMap hardwareMap, MatchState matchState, Alliance alliance) {
        Lebruxon.matchState = matchState;
        Lebruxon.alliance   = alliance;

        switch (alliance) {
            case RED:
                Lebruxon.startPose   = RED_START_POSE;
                Lebruxon.goal        = RED_GOAL;
                Lebruxon.goalShooter = RED_GOALPIDF;
                Lebruxon.targetClose = RED_GOAL_CLOSE;
                Lebruxon.targetFar = RED_GOAL_FAR;
                break;
            case BLUE:
                Lebruxon.startPose   = BLUE_START_POSE;
                Lebruxon.goal        = BLUE_GOAL;
                Lebruxon.goalShooter = BLUE_GOALPIDF;
                Lebruxon.targetClose = BLUE_GOAL_CLOSE;
                Lebruxon.targetFar = BLUE_GOAL_FAR;
                break;
            case REDCLOSE:
                Lebruxon.startPose   = CLOSE_RED_START_POSE;
                Lebruxon.goal        = RED_GOAL;
                Lebruxon.goalShooter = RED_GOALPIDF;
                Lebruxon.targetClose = RED_GOAL_CLOSE;
                Lebruxon.targetFar = RED_GOAL_FAR;
                break;
            case BLUECLOSE:
                Lebruxon.startPose   = CLOSE_BLUE_START_POSE;
                Lebruxon.goal        = BLUE_GOAL;
                Lebruxon.goalShooter = BLUE_GOALPIDF;
                Lebruxon.targetClose = BLUE_GOAL_CLOSE;
                Lebruxon.targetFar = BLUE_GOAL_FAR;
                break;
            case BLUESQ:
                Lebruxon.startPose   = BLUE_SQ_START_POSE;
                Lebruxon.goal        = BLUE_GOAL;
                Lebruxon.goalShooter = BLUE_GOALPIDF;
                Lebruxon.targetClose = BLUE_GOAL_CLOSE;
                Lebruxon.targetFar = BLUE_GOAL_FAR;
                break;
            case REDSQ:
                Lebruxon.startPose   = RED_SQ_START_POSE;
                Lebruxon.goal        = RED_GOAL;
                Lebruxon.goalShooter = RED_GOALPIDF;
                Lebruxon.targetClose = RED_GOAL_CLOSE;
                Lebruxon.targetFar = RED_GOAL_FAR;
                break;
        }

        drivetrain = new Drivetrain(hardwareMap);
        turret     = new Turret(hardwareMap);   // reads Storage in constructor to restore offset
        intake     = new Intake(hardwareMap);
        shooter    = new Shooter(hardwareMap);

        Storage.alliance = alliance;

        Lebruxon.drivetrain.follower.setStartingPose(matchState == MatchState.AUTO ? startPose : Storage.pose);
        Lebruxon.drivetrain.follower.update();

        CommandScheduler.getInstance().registerSubsystem(drivetrain, turret, intake, shooter);
    }

    public static void update() {
        drivetrain.update();
        turret.update();
        intake.update();
        shooter.update();
        turret.saveToStorage();
        Storage.pose = Lebruxon.drivetrain.follower.getPose(); // ADD THIS
    }

    public static InstantCommand reset() {
        return new InstantCommand(() -> {
            intake.setMinPower(0);
            shooter.autoPower(false, false);
            // Snapshot the turret's current angle and raw encoder position into
            // Storage every time we reset. This is the authoritative save point —
            // calling it here ensures that after the last auto reset() the values
            // are fresh before teleop re-inits and reads them.
            turret.saveToStorage();
            Storage.pose = Lebruxon.drivetrain.follower.getPose(); // ADD THIS
        });
    }

    public static InstantCommand prime() {
        return new InstantCommand(() -> {
            intake.setMinPower(0);
            shooter.autoPower(true, true);
        });
    }

    public static Command shoot() {
        AtomicBoolean usedTimeout = new AtomicBoolean(true);
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                new InstantCommand(() -> shooter.openStopper()),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    intake.setPower(1, 1);
                                    intake.setMinPower(1);
                                }),
                                new WaitCommand(1500),
                                new InstantCommand(() -> {
                                    intake.setPower(0, 0);
                                    intake.setMinPower(0);
                                })
                        ),
                        new InstantCommand(() -> shooter.autoPower(false, false)),
                        usedTimeout::get
                )
        );
    }

    public static Command shootWithIntake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.openStopper()),
                new WaitUntilCommand(() -> Lebruxon.shooter.controller.atSetPoint()),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    if(Lebruxon.shooter.distance>100) {
                                        intake.setPower(0.7, 0.7);
                                        intake.setMinPower(0.7);
                                    } else {
                                        intake.setPower(0.95, 0.95);
                                        intake.setMinPower(0.95);
                                    }
                                }),
                                new WaitCommand(80),
                                new InstantCommand(() -> shooter.setCurrentHoodPercent(1.1)),
                                new WaitCommand(150),
                                new InstantCommand(() -> shooter.setCurrentHoodPercent(1.1)),
                                new WaitCommand(400),
                                new InstantCommand(() -> {
                                    intake.setPower(0, 0);
                                    intake.setMinPower(0);
                                    shooter.closeStopper();
                                }),
                                reset()
                        ),
                        new InstantCommand(() -> {
                            shooter.autoPower(false, false);
                            shooter.closeStopper();
                        }),                        () -> Lebruxon.shooter.controller.atSetPoint()
                )
        );
    }
}