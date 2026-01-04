package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class JaySampleSwerveDrivetrain implements Subsystem {
    public static final JaySampleSwerveDrivetrain INSTANCE = new JaySampleSwerveDrivetrain();
    private JaySampleSwerveDrivetrain() {}

    private final double ANALOG_VOLTAGE_COMPENSATION = 3.1865;
    public static final double cacheTolerance = 0.1;

    public IMU imu;
    public RevHubOrientationOnRobot hubOrient = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    public SwerveModule fl_Module, bl_Module, br_Module, fr_Module;
    public SwerveModule[] swerveModules;

    double[] wheelSpeeds, targetAngles, currentAngles, angleErrors, cachedAngles;

    @Override
    public void initialize(){
        fl_Module = new SwerveModule(new MotorEx("fl_motor").reversed(), "fl_rotation", true,
                "fl_absolute", 6.046, true, ANALOG_VOLTAGE_COMPENSATION, -1, 1);

        bl_Module = new SwerveModule(new MotorEx("bl_motor"), "bl_rotation", true,
                "bl_absolute", 4.695, true, ANALOG_VOLTAGE_COMPENSATION, -1, -1);

        br_Module = new SwerveModule(new MotorEx("br_motor"), "br_rotation", true,
                "br_absolute", 2.007, true, ANALOG_VOLTAGE_COMPENSATION, 1, -1);

        fr_Module = new SwerveModule(new MotorEx("fr_motor").reversed(), "fr_rotation", true,
                "fr_absolute", 1.351, true, ANALOG_VOLTAGE_COMPENSATION, 1, 1);

        swerveModules = new SwerveModule[]{fl_Module, bl_Module, br_Module, fr_Module};

        wheelSpeeds = new double[swerveModules.length];
        targetAngles = new double[swerveModules.length];

        imu = ActiveOpMode.hardwareMap().get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(hubOrient));
        imu.resetYaw();

        //motor flipping arrays
        currentAngles = new double[swerveModules.length];
        angleErrors = new double[swerveModules.length];
        cachedAngles = new double[swerveModules.length];
    }

    @Override
    public void periodic(){
        double rawLeftX = ActiveOpMode.gamepad1().left_stick_x,
                rawLeftY = -ActiveOpMode.gamepad1().left_stick_y,
                rawRightX = ActiveOpMode.gamepad1().right_stick_x,
                realRightX = rawRightX / Math.sqrt(2);
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double tempX = rawLeftX;
        double tempY = rawLeftY;
        rawLeftX = tempX * Math.cos(-imuHeading) - tempY * Math.sin(-imuHeading);
        rawLeftY = tempX * Math.sin(-imuHeading) + tempY * Math.cos(-imuHeading);

        for (int i = 0; i < swerveModules.length; i++) {
            double rotVectorX = realRightX * swerveModules[i].yOffset;
            double rotVectorY = -1 * realRightX * swerveModules[i].xOffset;

            double resultX = rawLeftX + rotVectorX;
            double resultY = rawLeftY + rotVectorY;

            // Compute final speed + angle
            wheelSpeeds[i] = Math.sqrt(resultX * resultX + resultY * resultY);
            targetAngles[i] = Math.atan2(resultY, resultX);

            currentAngles[i] = swerveModules[i].getPodHeading();

        }

        // === Normalize wheel speeds so none exceed 1.0 ===
        double max = Math.max(Math.max(wheelSpeeds[0], wheelSpeeds[1]),
                Math.max(wheelSpeeds[2], wheelSpeeds[3]));
        if (max > 1.0) {
            for (int i = 0; i < swerveModules.length; i++)
            {wheelSpeeds[i] /= max;}
        }

        boolean joystickIsIdle = (Math.abs(rawLeftX) <= cacheTolerance && Math.abs(rawLeftY) <= cacheTolerance && Math.abs(rawRightX) <= cacheTolerance);

        for (int i = 0; i < swerveModules.length; i++){
            angleErrors[i] = Math.abs(Math.atan2(Math.sin(targetAngles[i] - currentAngles[i]), Math.cos(targetAngles[i] - currentAngles[i])));
            if (angleErrors[i] > Math.PI/2){
                targetAngles[i] = (targetAngles[i] + Math.PI) % (2*Math.PI);
                wheelSpeeds[i] *= -1;
                angleErrors[i] = Math.abs(Math.atan2(Math.sin(targetAngles[i] - currentAngles[i]), Math.cos(targetAngles[i] - currentAngles[i])));
            }
            wheelSpeeds[i] *= Math.abs(Math.cos(angleErrors[i]));

            if (!joystickIsIdle){
                cachedAngles[i] = targetAngles[i];
            }
        }

        // Apply to each module
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setMotorPower(wheelSpeeds[i]);
            double commandedAngle = joystickIsIdle ? cachedAngles[i] : targetAngles[i];
            swerveModules[i].rotateTo(commandedAngle);
        }
    }
}