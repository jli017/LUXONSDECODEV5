package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.Lebruxon;
import org.firstinspires.ftc.teamcode.utils.Storage;

@Configurable
public class Drivetrain extends SubsystemBase {
    public Follower follower;
    public static double fast = 1;
    public static boolean turbo = false;
    public PIDFController headingController;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        follower.update();
    }

    public void drive(Gamepad gamepad1) {
        if(Storage.alliance == Lebruxon.Alliance.BLUE || Storage.alliance == Lebruxon.Alliance.BLUECLOSE || Storage.alliance == Lebruxon.Alliance.BLUESQ ) {
            if (turbo) {
                follower.setTeleOpDrive(
                        gamepad1.left_stick_y * 0.6,
                        gamepad1.left_stick_x * 0.6,
                        -gamepad1.right_stick_x * 0.8,
                        false
                );
            }
            else{
                follower.setTeleOpDrive(
                        gamepad1.left_stick_y * fast,
                        gamepad1.left_stick_x * fast,
                        -gamepad1.right_stick_x * 0.8,
                        false
                );
            }
        }
        else {
            if (turbo) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * 0.6,
                        -gamepad1.left_stick_x * 0.6,
                        -gamepad1.right_stick_x * 0.8,
                        false
                );
            }
            else{
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * fast,
                        -gamepad1.left_stick_x * fast,
                        -gamepad1.right_stick_x * 0.8,
                        false
                );
            }
        }

    }

    public void update(){
        follower.update();
        Storage.pose = follower.getPose();
    }
}
