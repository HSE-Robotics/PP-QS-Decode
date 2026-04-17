package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.geometry.Pose;

public class DataStorage {
    public static Pose currentPose = new Pose();
    public static Boolean Blue = false;
    public static Boolean FAR = false;
    public static Pose BlueIntake = new Pose(127,29,Math.toRadians(180));
    public static Pose BlueShoot = new Pose(84,35,Math.toRadians(123));
    public static Pose RedIntake = new Pose(-5,15,Math.toRadians(0));
    public static Pose RedIntakeFAR = new Pose(-2,13,Math.toRadians(0));
    public static Pose RedShoot = new Pose(56,23,Math.toRadians(57));
    public static Pose RedShootFAR = new Pose(57,17,Math.toRadians(57));
}
