package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Configurable
@TeleOp(name = "Turret Aim Test", group = "Testing")
public class TurretTest extends OpMode {

    // =========================================
    // Dashboard Editable Robot Pose
    // =========================================

    public static double ROBOT_X       = 56;
    public static double ROBOT_Y       = 7;
    public static double ROBOT_HEADING = 180; // degrees
    public static double GOAL_X        = 15;
    public static double GOAL_Y        = 141;

    // =========================================
    // Hardware
    // =========================================

    public CRServo   leftServo;
    public CRServo   rightServo;
    public DcMotorEx encoderMotor;

    // =========================================
    // Encoder / Gearing
    // =========================================

    public static double encoderTicksPerRev = 8192.0;
    public static double gearRatio          = 210.0 / 71.0;
    public static double ticksPerTurretRev  = encoderTicksPerRev * gearRatio;
    public static double ticksPerRadian     = ticksPerTurretRev / (2.0 * Math.PI);
    public static double turretZeroOffset   = 0;

    // =========================================
    // Hard Limits
    // =========================================

    private static final double MAX_ANGLE    = Math.toRadians(70.0);
    private static final double MIN_ANGLE    = Math.toRadians(-240.0);

    // Trigger recovery this many degrees before the hard stop
    private static final double LIMIT_BUFFER = Math.toRadians(1.0);

    // =========================================
    // PID Tuning
    // =========================================

    public static double p               = 0.42;
    public static double d               = 0.03;
    public static double kStatic         = 0.025;
    public static double maxPower        = 0.55;
    public static double toleranceDeg    = 1.5;
    public static double minOutput       = 0.015;
    public static double filterGain      = 0.18;
    public static double powerFilterGain = 0.22;

    public PIDFController controller = new PIDFController(p, 0, d, 0);

    // =========================================
    // State Machine
    // =========================================

    private enum TurretState {
        NORMAL,
        RECOVERING
    }

    private TurretState turretState    = TurretState.NORMAL;
    private double      recoveryTarget = 0;
    private boolean     recoveringCCW  = false;

    // =========================================
    // Runtime State
    // =========================================

    public double filteredAngle = 0;
    public double filteredPower = 0;
    public double targetAngle   = 0;

    // Debug
    private double  dbgFieldTarget   = 0;
    private double  dbgRelativeAngle = 0;
    private double  dbgAimOnLine     = 0;
    private boolean dbgShortBlocked  = false;
    private double  dbgPosWrapped    = 0;

    // =========================================
    // Init
    // =========================================

    @Override
    public void init() {

        leftServo    = hardwareMap.get(CRServo.class,   "turretLeft");
        rightServo   = hardwareMap.get(CRServo.class,   "turretRight");
        encoderMotor = hardwareMap.get(DcMotorEx.class, "intake");

        encoderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftServo.setDirection(CRServo.Direction.REVERSE);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        controller.setTolerance(Math.toRadians(toleranceDeg));
        controller.reset();

        filteredAngle = 0;
        filteredPower = 0;
        targetAngle   = 0;
        turretState   = TurretState.NORMAL;
    }

    // =========================================
    // Loop
    // =========================================

    @Override
    public void loop() {

        controller.setP(p);
        controller.setD(d);
        controller.setTolerance(Math.toRadians(toleranceDeg));

        // =========================================
        // Filtered continuous angle
        // =========================================

        filteredAngle = filteredAngle * (1.0 - filterGain)
                + getAngle()    * filterGain;

        double pos = filteredAngle;

        // =========================================
        // Field-centric target
        // =========================================

        double robotHeadingRad = Math.toRadians(ROBOT_HEADING);

        double dx = GOAL_X - ROBOT_X;
        double dy = GOAL_Y - ROBOT_Y;

        double fieldTargetAngle = Math.atan2(dy, dx);

        // Robot-relative aim angle in [-pi, pi]
        double relativeAngle = wrapToPi(fieldTargetAngle - robotHeadingRad);

        // Current position wrapped to [-pi, pi] — used only for delta math
        double posWrapped = wrapToPi(pos);

        // Shortest angular delta to the target
        double shortDelta = wrapToPi(relativeAngle - posWrapped);

        // Longest delta is the other way around the circle
        double longDelta = (shortDelta >= 0)
                ? shortDelta - 2.0 * Math.PI
                : shortDelta + 2.0 * Math.PI;

        // Candidate targets in unwrapped encoder space
        double shortTarget = pos + shortDelta;
        double longTarget  = pos + longDelta;

        boolean shortFits = shortTarget >= MIN_ANGLE && shortTarget <= MAX_ANGLE;
        boolean longFits  = longTarget  >= MIN_ANGLE && longTarget  <= MAX_ANGLE;

        double  aimOnLine;
        boolean shortBlocked;

        if (shortFits) {
            aimOnLine    = shortTarget;
            shortBlocked = false;
        } else if (longFits) {
            aimOnLine    = longTarget;
            shortBlocked = true;
        } else {
            // Neither fits — park on whichever limit shortTarget overshot
            aimOnLine    = (shortTarget > MAX_ANGLE) ? MAX_ANGLE : MIN_ANGLE;
            shortBlocked = true;
        }

        dbgFieldTarget   = fieldTargetAngle;
        dbgRelativeAngle = relativeAngle;
        dbgAimOnLine     = aimOnLine;
        dbgShortBlocked  = shortBlocked;
        dbgPosWrapped    = posWrapped;

        // =========================================
        // State Machine
        // =========================================

        switch (turretState) {

            case NORMAL: {

                targetAngle = aimOnLine;

                double error = targetAngle - pos;

                boolean nearCWLimit  = pos >= MAX_ANGLE - LIMIT_BUFFER;
                boolean nearCCWLimit = pos <= MIN_ANGLE + LIMIT_BUFFER;

                if (nearCWLimit && error > 0) {
                    // Pinned at CW stop, target wants even more CW -> must unwind CCW
                    enterRecovery(pos, relativeAngle, true);

                } else if (nearCCWLimit && error < 0) {
                    // Pinned at CCW stop, target wants even more CCW -> must unwind CW
                    enterRecovery(pos, relativeAngle, false);
                }

                break;
            }

            case RECOVERING: {

                // Drive toward the fixed recovery target set at entry.
                // Do NOT update recoveryTarget here — chasing a moving target
                // during recovery causes overshoot and stale-target lock-up.
                targetAngle = recoveryTarget;

                // Exit as soon as the real goal is reachable on either path.
                // No proximity check needed: NORMAL will immediately compute
                // the correct aimOnLine on the very next frame.
                if (shortFits || longFits) {
                    turretState = TurretState.NORMAL;
                    controller.reset();
                }

                break;
            }
        }

        // =========================================
        // PID
        // =========================================

        double error = targetAngle - pos;

        controller.setSetPoint(targetAngle);

        double power;

        if (Math.abs(error) < Math.toRadians(toleranceDeg)) {

            power = 0;
            controller.reset();

        } else {

            power = controller.calculate(pos);

            if (Math.abs(power) > minOutput) {
                power += Math.signum(power) * kStatic;
            }

            power = Math.max(-maxPower, Math.min(maxPower, power));
        }

        filteredPower = filteredPower * (1.0 - powerFilterGain)
                + power         * powerFilterGain;

        if (Math.abs(filteredPower) < 0.01) {
            filteredPower = 0;
        }

        leftServo.setPower(filteredPower);
        rightServo.setPower(filteredPower);

        // =========================================
        // Telemetry
        // =========================================

        telemetry.addLine("===== ROBOT =====");
        telemetry.addData("Robot X",        ROBOT_X);
        telemetry.addData("Robot Y",        ROBOT_Y);
        telemetry.addData("Robot Heading",  ROBOT_HEADING);

        telemetry.addLine();

        telemetry.addLine("===== GOAL =====");
        telemetry.addData("Goal X", GOAL_X);
        telemetry.addData("Goal Y", GOAL_Y);

        telemetry.addLine();

        telemetry.addLine("===== TURRET =====");
        telemetry.addData("State",            turretState.name());
        telemetry.addData("Recovering CCW",   recoveringCCW);
        telemetry.addData("Position Deg",     Math.toDegrees(pos));
        telemetry.addData("Pos Wrapped Deg",  Math.toDegrees(dbgPosWrapped));
        telemetry.addData("Field Target Deg", Math.toDegrees(dbgFieldTarget));
        telemetry.addData("Relative Aim Deg", Math.toDegrees(dbgRelativeAngle));
        telemetry.addData("Short Blocked",    dbgShortBlocked);
        telemetry.addData("Aim On Line Deg",  Math.toDegrees(dbgAimOnLine));
        telemetry.addData("Recovery Target",  Math.toDegrees(recoveryTarget));
        telemetry.addData("Target Deg",       Math.toDegrees(targetAngle));
        telemetry.addData("Error Deg",        Math.toDegrees(error));
        telemetry.addData("Raw Power",        power);
        telemetry.addData("Filtered Power",   filteredPower);
        telemetry.addData("Encoder Ticks",    encoderMotor.getCurrentPosition());
        telemetry.addData("CW Limit Deg",     Math.toDegrees(MAX_ANGLE));
        telemetry.addData("CCW Limit Deg",    Math.toDegrees(MIN_ANGLE));

        telemetry.update();
    }

    // =========================================
    // Recovery Helper
    // =========================================

    /**
     * Compute and latch a fixed recovery target in unwrapped encoder space.
     *
     * The target is the position the turret needs to reach so that the goal
     * becomes reachable on the long-way-around path. It is set once and held
     * fixed for the duration of recovery so the PID has a stable destination.
     *
     * @param pos           current unwrapped turret position (radians)
     * @param relativeAngle robot-relative aim angle in [-pi, pi]
     * @param goingCCW      true  -> unwind CCW (decreasing angle, toward MIN_ANGLE)
     *                      false -> unwind CW  (increasing angle, toward MAX_ANGLE)
     */
    private void enterRecovery(double pos, double relativeAngle, boolean goingCCW) {

        turretState   = TurretState.RECOVERING;
        recoveringCCW = goingCCW;
        controller.reset();

        // Shortest delta from current wrapped pos to relative aim
        double delta = wrapToPi(relativeAngle - wrapToPi(pos));

        // Force delta into the recovery direction (the long way around)
        if ( goingCCW && delta > 0) delta -= 2.0 * Math.PI;
        if (!goingCCW && delta < 0) delta += 2.0 * Math.PI;

        // Apply to unwrapped pos and clamp to legal range
        recoveryTarget = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, pos + delta));
    }

    // =========================================
    // Helpers
    // =========================================

    public double getAngle() {
        double ticks = encoderMotor.getCurrentPosition();
        return (ticks / ticksPerRadian) + turretZeroOffset;
    }

    /**
     * Wraps an angle in radians to (-pi, pi].
     */
    public static double wrapToPi(double radians) {

        double twoPi = 2.0 * Math.PI;

        radians %= twoPi;

        if (radians <= -Math.PI) {
            radians += twoPi;
        } else if (radians > Math.PI) {
            radians -= twoPi;
        }

        return radians;
    }
}