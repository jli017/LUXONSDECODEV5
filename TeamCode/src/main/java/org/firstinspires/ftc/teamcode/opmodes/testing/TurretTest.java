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

    public CRServo leftServo;
    public CRServo rightServo;
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

    private TurretState turretState = TurretState.NORMAL;

    private double recoveryTarget = 0;
    private boolean recoveringCCW = false;

    // =========================================
    // Runtime State
    // =========================================

    public double filteredAngle = 0;
    public double filteredPower = 0;
    public double targetAngle   = 0;

    // Debug
    private double dbgFieldTarget   = 0;
    private double dbgRelativeAngle = 0;
    private double dbgAimOnLine     = 0;
    private boolean dbgShortBlocked = false;
    private double dbgPosWrapped    = 0;

    // =========================================
    // Init
    // =========================================

    @Override
    public void init() {

        leftServo    = hardwareMap.get(CRServo.class, "turretLeft");
        rightServo   = hardwareMap.get(CRServo.class, "turretRight");
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

        filteredAngle =
                filteredAngle * (1.0 - filterGain)
                        + getAngle() * filterGain;

        double pos = filteredAngle;

        // =========================================
        // Field-centric target
        // =========================================

        double robotHeadingRad = Math.toRadians(ROBOT_HEADING);

        double dx = GOAL_X - ROBOT_X;
        double dy = GOAL_Y - ROBOT_Y;

        double fieldTargetAngle = Math.atan2(dy, dx);

        double relativeAngle =
                wrapToPi(fieldTargetAngle - robotHeadingRad);

        double posWrapped = wrapToPi(pos);

        double shortDelta =
                wrapToPi(relativeAngle - posWrapped);

        double longDelta =
                shortDelta >= 0
                        ? shortDelta - 2.0 * Math.PI
                        : shortDelta + 2.0 * Math.PI;

        double shortTarget = pos + shortDelta;
        double longTarget  = pos + longDelta;

        boolean shortFits =
                shortTarget >= MIN_ANGLE &&
                        shortTarget <= MAX_ANGLE;

        boolean longFits =
                longTarget >= MIN_ANGLE &&
                        longTarget <= MAX_ANGLE;

        double aimOnLine;
        boolean shortBlocked;

        if (shortFits) {

            aimOnLine = shortTarget;
            shortBlocked = false;

        } else if (longFits) {

            aimOnLine = longTarget;
            shortBlocked = true;

        } else {

            aimOnLine =
                    Math.max(MIN_ANGLE,
                            Math.min(MAX_ANGLE, shortTarget));

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

                boolean atCWStop =
                        pos >= MAX_ANGLE;

                boolean atCCWStop =
                        pos <= MIN_ANGLE;

                if (atCWStop && error > 0) {

                    enterRecovery(pos, relativeAngle, true);

                } else if (atCCWStop && error < 0) {

                    enterRecovery(pos, relativeAngle, false);
                }

                break;
            }

            case RECOVERING: {

                updateRecoveryTarget(pos, relativeAngle);

                targetAngle = recoveryTarget;

                if (Math.abs(targetAngle - pos)
                        < Math.toRadians(toleranceDeg + 1.0)) {

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

            power =
                    Math.max(-maxPower,
                            Math.min(maxPower, power));
        }

        filteredPower =
                filteredPower * (1.0 - powerFilterGain)
                        + power * powerFilterGain;

        if (Math.abs(filteredPower) < 0.01) {
            filteredPower = 0;
        }

        leftServo.setPower(filteredPower);
        rightServo.setPower(filteredPower);

        // =========================================
        // Telemetry
        // =========================================

        telemetry.addLine("===== ROBOT =====");
        telemetry.addData("Robot X", ROBOT_X);
        telemetry.addData("Robot Y", ROBOT_Y);
        telemetry.addData("Robot Heading", ROBOT_HEADING);

        telemetry.addLine();

        telemetry.addLine("===== GOAL =====");
        telemetry.addData("Goal X", GOAL_X);
        telemetry.addData("Goal Y", GOAL_Y);

        telemetry.addLine();

        telemetry.addLine("===== TURRET =====");
        telemetry.addData("State", turretState.name());
        telemetry.addData("Recovering CCW", recoveringCCW);
        telemetry.addData("Position Deg", Math.toDegrees(pos));
        telemetry.addData("Pos Wrapped Deg", Math.toDegrees(dbgPosWrapped));
        telemetry.addData("Field Target Deg", Math.toDegrees(dbgFieldTarget));
        telemetry.addData("Relative Aim Deg", Math.toDegrees(dbgRelativeAngle));
        telemetry.addData("Short Blocked", dbgShortBlocked);
        telemetry.addData("Aim On Line Deg", Math.toDegrees(dbgAimOnLine));
        telemetry.addData("Target Deg", Math.toDegrees(targetAngle));
        telemetry.addData("Error Deg", Math.toDegrees(error));
        telemetry.addData("Raw Power", power);
        telemetry.addData("Filtered Power", filteredPower);
        telemetry.addData("Encoder Ticks", encoderMotor.getCurrentPosition());
        telemetry.addData("CW Limit Deg", Math.toDegrees(MAX_ANGLE));
        telemetry.addData("CCW Limit Deg", Math.toDegrees(MIN_ANGLE));

        telemetry.update();
    }

    // =========================================
    // Recovery Helpers
    // =========================================

    private void enterRecovery(
            double pos,
            double relativeAngle,
            boolean goingCCW
    ) {

        turretState   = TurretState.RECOVERING;
        recoveringCCW = goingCCW;

        controller.reset();

        double posWrapped = wrapToPi(pos);

        double delta =
                wrapToPi(relativeAngle - posWrapped);

        // Force long-way path

        if (goingCCW) {

            if (delta > 0) {
                delta -= 2.0 * Math.PI;
            }

        } else {

            if (delta < 0) {
                delta += 2.0 * Math.PI;
            }
        }

        recoveryTarget = pos + delta;

        recoveryTarget =
                Math.max(MIN_ANGLE,
                        Math.min(MAX_ANGLE, recoveryTarget));
    }

    private void updateRecoveryTarget(
            double pos,
            double relativeAngle
    ) {

        double posWrapped = wrapToPi(pos);

        double delta =
                wrapToPi(relativeAngle - posWrapped);

        // Force same direction continuously

        if (recoveringCCW) {

            // force negative rotation
            if (delta > 0) {
                delta -= 2.0 * Math.PI;
            }

        } else {

            // force positive rotation
            if (delta < 0) {
                delta += 2.0 * Math.PI;
            }
        }

        double candidate = pos + delta;

        candidate =
                Math.max(MIN_ANGLE,
                        Math.min(MAX_ANGLE, candidate));

        // Never reverse direction during recovery

        if (recoveringCCW) {

            // CCW = decreasing angle
            if (candidate <= pos) {
                recoveryTarget = candidate;
            }

        } else {

            // CW = increasing angle
            if (candidate >= pos) {
                recoveryTarget = candidate;
            }
        }
    }

    // =========================================
    // Helpers
    // =========================================

    public double getAngle() {

        double ticks = encoderMotor.getCurrentPosition();

        return (ticks / ticksPerRadian)
                + turretZeroOffset;
    }

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