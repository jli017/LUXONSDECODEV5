package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.utils.Lebruxon;
import org.firstinspires.ftc.teamcode.utils.Paths;

import java.util.HashSet;
import java.util.Set;

/**
 * Blue-alliance 18-ball autonomous.
 *
 * Drives the whole BigBoiBlue PathChain as ONE continuous follow (never
 * stopping between waypoints, so velocity — and therefore the shoot-on-the-
 * move lead compensation — stays meaningful throughout) and fires at each
 * "return toward the goal" segment instead of splitting the route into
 * separate PathChains.
 *
 * ASSUMPTIONS — verify before running:
 * - CommandOpMode base class, OpMode package, and the nested FollowPathCommand
 *   are the same guesses called out in the previous version of this file.
 * - SHOOT_WINDOW_PATH_INDICES below is inferred from BigBoiBlue's geometry
 *   (the addPath() segments that return near the goal cluster around
 *   (50-54, 72-77)) — confirm these indices actually line up with your
 *   intended scoring points before trusting this on the field.
 */
@Autonomous(preselectTeleOp="TeleOp")
public class BlueOPT21 extends CommandOpMode {

    // Indices (0-based, matching addPath() call order in Paths.BigBoiBlue)
    // of segments where the robot is heading back toward the goal and should
    // be allowed to fire. Segment 9 (final move to ~(51,115)) is included —
    // that's the parking pose and a shot is wanted there too.
    private static final Set<Integer> SHOOT_WINDOW_PATH_INDICES =
            new HashSet<>(java.util.Arrays.asList(1, 3, 5, 7, 9));

    private Paths paths;

    @Override
    public void initialize() {
        Lebruxon.init(hardwareMap, Lebruxon.MatchState.AUTO, Lebruxon.Alliance.BLUE);
        paths = new Paths(Lebruxon.drivetrain.follower, Lebruxon.Alliance.BLUE);

        Lebruxon.turret.enableAim = true;

        // Runs Lebruxon.update() (turret/shooter/drivetrain update + lead
        // compensation) every scheduler tick for the whole OpMode.
        schedule(new RunCommand(Lebruxon::update));

        schedule(
                new SequentialCommandGroup(
                        // Spin up before the robot starts moving — spin-up lag
                        // means priming after the move starts misses the early
                        // shooting window.
                        Lebruxon.prime(),

                        new ParallelDeadlineGroup(
                                new FollowPathCommand(paths.BiggerBoiBlue),
                                new ShootAtPathWindows(SHOOT_WINDOW_PATH_INDICES)
                        ),

                        new WaitCommand(500),
                        new InstantCommand(() -> Lebruxon.shooter.autoPower(false,false)),
                        new InstantCommand(() -> Lebruxon.shooter.idle = false),
                        new InstantCommand(() -> Lebruxon.turret.enableAim = false),
                        Lebruxon.reset()
                )
        );
    }

    /**
     * Watches which segment of the active PathChain the follower is currently
     * on. Each time it enters a new segment listed in windowIndices that it
     * hasn't already fired for, it checks turret + shooter readiness AND
     * that the robot's field X position is past 50 (so it doesn't fire right
     * at the start of a return segment while still near the intake side) —
     * if all three hold, it schedules Lebruxon.shootWithIntake() in the
     * background — the shot sequence runs concurrently with the path, it
     * does not pause it.
     *
     * If the robot is still in the same shoot-window segment next loop and
     * hasn't fired yet (turret/shooter weren't ready in time), it keeps
     * checking every loop until either it fires or the segment changes.
     */
    private static class ShootAtPathWindows extends CommandBase {
        private final Set<Integer> windowIndices;
        private int lastFiredIndex = -1;
        private int lastSeenIndex = -1;

        ShootAtPathWindows(Set<Integer> windowIndices) {
            this.windowIndices = windowIndices;
        }

        @Override
        public void execute() {
            // getCurrentPathNumber() returns a double; floor it down to the
            // actual integer segment index (e.g. 3.0 or 3.7 both mean "on
            // segment 3").
            int currentIndex = (int) Math.floor(Lebruxon.drivetrain.follower.getCurrentPathNumber());

            // Entering a new segment resets whether we've fired in THIS window
            // — lastFiredIndex only blocks re-firing within the same segment.
            if (currentIndex != lastSeenIndex) {
                lastSeenIndex = currentIndex;
            }

            if (!windowIndices.contains(currentIndex)) {
                return;
            }

            if (lastFiredIndex == currentIndex) {
                // Already fired once for this pass through this segment.
                return;
            }

            boolean turretReady  = Lebruxon.turret.controller.atSetPoint();
            boolean shooterReady = Lebruxon.shooter.controller.atSetPoint();
            boolean positionReady = Lebruxon.drivetrain.follower.getPose().getX() > 50;

            if (turretReady && shooterReady && positionReady) {
                CommandScheduler.getInstance().schedule(Lebruxon.shootWithIntake());
                lastFiredIndex = currentIndex;
            }
        }

        @Override
        public boolean isFinished() {
            // Runs for the whole path; the ParallelDeadlineGroup's deadline
            // (FollowPathCommand) is what actually ends this.
            return false;
        }
    }

    /**
     * Minimal PedroPathing follow wrapper. Replace with your project's
     * existing FollowPathCommand if one already exists.
     */
    private static class FollowPathCommand extends CommandBase {
        private final com.pedropathing.paths.PathChain pathChain;

        FollowPathCommand(com.pedropathing.paths.PathChain pathChain) {
            this.pathChain = pathChain;
            addRequirements(Lebruxon.drivetrain);
        }

        @Override
        public void initialize() {
            Lebruxon.drivetrain.follower.followPath(pathChain, true);
        }

        @Override
        public boolean isFinished() {
            return !Lebruxon.drivetrain.follower.isBusy();
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                Lebruxon.drivetrain.follower.breakFollowing();
            }
        }
    }
}