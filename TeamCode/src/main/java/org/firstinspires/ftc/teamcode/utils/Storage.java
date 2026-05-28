package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;

public class    Storage {
    public static Pose pose = Lebruxon.BLUE_START_POSE;

    // Raw encoder ticks at the time the turret angle was last snapshotted.
    // Used to reconstruct absolute turret position after re-init (e.g. auto → teleop).
    // turretAngle holds the normalized [0, 2PI) angle at the snapshot moment;
    // turretEncoderSnapshot holds the raw tick count at that same moment so
    // Turret can compute a tick offset and apply it to getCurrentPosition().
    public static double turretAngle = 0.0;
    public static int turretEncoderSnapshot = 0;

    public static Lebruxon.Alliance alliance = Lebruxon.Alliance.BLUE;
}