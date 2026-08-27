package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.utils.Lebruxon;
import org.firstinspires.ftc.teamcode.utils.Paths;

import java.util.HashSet;
import java.util.Set;

/**
 * Blue-alliance 21-ball autonomous.
 *
 * Drives the whole BiggerBoiBlue PathChain as ONE continuous follow (never
 * stopping between waypoints, so velocity — and therefore the shoot-on-the-
 * move lead compensation — stays meaningful throughout), runs intake during
 * every segment that isn't a shoot window, and fires at each "return toward
 * the goal" segment.
 *
 * ASSUMPTIONS / THINGS TO VERIFY BEFORE RUNNING — see chat notes:
 * - CommandOpMode base class, OpMode package, and the nested FollowPathCommand
 *   are unverified guesses (called out in earlier versions of this file).
 * - SHOOT_WINDOW_PATH_INDICES is now derived from BiggerBoiBlue's actual
 *   12-segment addPath() sequence in Paths.java (confirmed, not guessed):
 *   odd indices 1,3,5,7,9,11 are the "return toward goal" segments.
 * - intake.setPower(1, 1) / setMinPower(1) as the "collecting" power level
 *   is a guess — swap in whatever power your Intake subsystem actually
 *   uses for full-speed collection.
 */
@Autonomous(preselectTeleOp = "TeleOp")
public class Blue21SOTM extends CommandOpMode {

    // Indices (0-based, matching addPath() call order in Paths.BiggerBoiBlue)
    // of segments where the robot is heading back toward the goal and should
    // be allowed to fire. Verified against BiggerBoiBlue's actual 12-segment
    // structure: even indices (0,2,4,6,8,10) are collection runs, odd indices
    // (1,3,5,7,9,11) are returns toward the goal. Segment 11 is the final
    // return, ending near (52.3, 112.1) — the parking-area shot.
    private static final Set<Integer> SHOOT_WINDOW_PATH_INDICES =
            new HashSet<>(java.util.Arrays.asList(1, 3, 5, 7, 9, 11));

    private Paths paths;

    @Override
    public void initialize() {
        Lebruxon.init(hardwareMap, Lebruxon.MatchState.AUTO, Lebruxon.Alliance.BLUESQ);
        paths = new Paths(Lebruxon.drivetrain.follower, Lebruxon.Alliance.BLUESQ);

        Lebruxon.turret.enableAim = false;
        Lebruxon.shooter.idle = false;

        // Runs Lebruxon.update() (turret/shooter/drivetrain update + lead
        // compensation) every scheduler tick for the whole OpMode.
        schedule(new RunCommand(Lebruxon::update));

        schedule(
                new SequentialCommandGroup(
                        // Spin up before the robot starts moving — spin-up lag
                        // means priming after the move starts misses the early
                        // shooting window.
                        new ParallelDeadlineGroup(
                                new FollowPathCommand(paths.BiggerBoiBlue),
                                new ParallelCommandGroup(
                                        new ShootAtPathWindows(SHOOT_WINDOW_PATH_INDICES),
                                        new IntakeDuringCollection(SHOOT_WINDOW_PATH_INDICES)
                                )
                        ),

                        // Gives the last-triggered shot (segment 9, at the tail end
                        // of the path) time to finish its intake feed before the
                        // shutdown steps cut turret aim / shooter spin. This is a
                        // FIXED delay, not tied to the actual shot command finishing
                        // — shootWithIntake()'s own timing is roughly
                        // WaitUntilCommand(atSetPoint) + 80ms + 150ms + 450ms once
                        // triggered, so 500ms covers the feed itself but NOT any
                        // wait-for-atSetPoint delay if the shot was still spinning up
                        // when the path ended. If the last shot sometimes fires late,
                        // consider tracking the actual command (as discussed earlier)
                        // instead of a fixed wait.
                        new WaitCommand(500),
                        new InstantCommand(() -> Lebruxon.shooter.autoPower(false, false)),
                        new InstantCommand(() -> Lebruxon.shooter.idle = false),
                        new InstantCommand(() -> Lebruxon.turret.enableAim = false),
                        new InstantCommand(() -> Lebruxon.intake.setPower(0, 0)),
                        new InstantCommand(() -> Lebruxon.intake.setMinPower(0)),
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

            boolean positionReady = Lebruxon.drivetrain.follower.getPose().getX() > 45;

            if (positionReady) {
                CommandScheduler.getInstance().schedule(Lebruxon.shootNow());
                //CommandScheduler.getInstance().schedule(Lebruxon.shootWithIntake());
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
     * Runs intake during every segment NOT in windowIndices (the "go collect
     * balls" segments) and stops it while in a shoot-window segment.
     *
     * Deliberately EDGE-TRIGGERED — it only issues a new intake.setPower()
     * call the instant the segment index crosses in or out of a window,
     * not on every execute() tick. This matters: shootWithIntake() (fired
     * from ShootAtPathWindows) also drives intake power directly while
     * feeding a ball into the shooter. If this command re-asserted "intake
     * off" every single tick while inside a shoot-window segment, it would
     * stomp shootWithIntake()'s feed pulse the very next tick and the ball
     * would never actually get fed. Setting power only on the transition
     * means this command gets out of the way for the whole duration of the
     * window, and shootWithIntake() is free to drive intake however it needs
     * to, then hand control back (it already resets to 0 power itself when
     * the shot finishes) before this command resumes on the next transition.
     */
    private static class IntakeDuringCollection extends CommandBase {
        private final Set<Integer> windowIndices;
        private int lastSeenIndex = -1;
        private Boolean lastCommandedOn = null; // null = nothing commanded yet

        IntakeDuringCollection(Set<Integer> windowIndices) {
            this.windowIndices = windowIndices;
        }

        @Override
        public void execute() {
            int currentIndex = (int) Math.floor(Lebruxon.drivetrain.follower.getCurrentPathNumber());

            if (currentIndex == lastSeenIndex) {
                return; // still in the same segment — don't re-issue the command
            }
            lastSeenIndex = currentIndex;

            boolean shouldBeOn = !windowIndices.contains(currentIndex);

            if (lastCommandedOn != null && lastCommandedOn == shouldBeOn) {
                return; // no state change needed
            }

            if (shouldBeOn) {
                Lebruxon.intake.setPower(1, 1);
                Lebruxon.intake.setMinPower(1);
            } else {
                Lebruxon.intake.setPower(0, 0);
                Lebruxon.intake.setMinPower(0);
            }
            lastCommandedOn = shouldBeOn;
        }

        @Override
        public boolean isFinished() {
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