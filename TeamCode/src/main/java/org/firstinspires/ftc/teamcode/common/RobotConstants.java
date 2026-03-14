package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.geometry.Pose;

public class RobotConstants {
    public enum Alliance {
        RED,BLUE
    }

    public static Alliance alliance = Alliance.RED;

    //goal poses
    public static Pose redGoal = new Pose(144 ,144);
    public static Pose blueGoal = new Pose(0, 144);

    //hood
    public static final String hood_name = "hood_servo";
    public static double hoodMax = 0.99, hoodMin = 0;

    //intake
    public static final String leftIntake = "front_intake";
    public static final String rightIntake = "right_intake";
    public static final boolean rightIntakeReversed = false;
    public static double intakeBoost = 0;

    //turret
    public static final String leftTurret = "lt_servo";
    public static final String rightTurret = "rt_servo";
    public static final double LTZERO = 0.0;
    public static final double RTZERO = 0.0;
    public static double turretServoRange = 355;

    //shooter
    public static final String leftShooter = "left_shooter";
    public static final String rightShooter=  "right_shooter";
    public static final boolean leftShooterReversed = true;
    public static double shooterP = 0.001, shooterI = 0, shooterD = 0, shooterF = 0.0004;



}
