package org.firstinspires.ftc.teamcode.swerveDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@Disabled
public class DTTest implements Subsystem {
    public static final DTTest INSTANCE = new DTTest();
    private DTTest() {}

    private final double ANALOG_VOLTAGE_COMPENSATION = 3.22;
    public SwerveModule fl_Module, bl_Module, br_Module, fr_Module;
    public SwerveModule[] swerveModules;

    @Override
    public void initialize(){
        fl_Module = new SwerveModule(new MotorEx("fl_motor"), "fl_rotation", true,
                "fl_absolute", 6.046, true, ANALOG_VOLTAGE_COMPENSATION, 1, 1);

        bl_Module = new SwerveModule(new MotorEx("bl_motor"), "bl_rotation", true,
                "bl_absolute", 4.695, true, ANALOG_VOLTAGE_COMPENSATION, 1, 1);

        br_Module = new SwerveModule(new MotorEx("br_motor").reversed(), "br_rotation", true,
                "br_absolute", 2.007, true, ANALOG_VOLTAGE_COMPENSATION, 1, 1);

        fr_Module = new SwerveModule(new MotorEx("fr_motor").reversed(), "fr_rotation", true,
                "fr_absolute", 1.351, true, ANALOG_VOLTAGE_COMPENSATION, 1, 1);

        swerveModules = new SwerveModule[]{fl_Module, bl_Module, br_Module, fr_Module};
    }

    @Override
    public void periodic(){
        double rawLeftX = ActiveOpMode.gamepad1().left_stick_x,
                rawLeftY = ActiveOpMode.gamepad1().left_stick_y,
                rawRightX = ActiveOpMode.gamepad1().right_stick_x;

        double[][] vectorPositions = {
                {-1, 1}, //FL
                {-1, -1}, //BL
                {1, -1}, //BR
                {1, 1} //FR
        };
        double[] wheelSpeeds = new double[swerveModules.length];
        double[] rotationAngles = new double[swerveModules.length];

        for (int i = 0; i < 4; i++) {
            double wheelX = vectorPositions[i][0];
            double wheelY = vectorPositions[i][1];

            // Compute rotation vector (perpendicular to wheel position)
            double rotX = -wheelY * rawRightX;
            double rotY =  wheelX * rawRightX;

            // Combine translation + rotation
            double resultX = rawLeftX + rotX;
            double resultY = rawLeftY + rotY;

            // Compute final speed + angle
            wheelSpeeds[i] = Math.sqrt(resultX * resultX + resultY * resultY);
            rotationAngles[i] = Math.atan2(resultY, resultX);
        }

        // === Normalize wheel speeds so none exceed 1.0 ===
        double max = Math.max(Math.max(wheelSpeeds[0], wheelSpeeds[1]),
                Math.max(wheelSpeeds[2], wheelSpeeds[3]));
        if (max > 1.0) {
            for (int i = 0; i < 4; i++) wheelSpeeds[i] /= max;
        }

        // === Apply to each module ===
        SwerveModule[] modules = {fl_Module, fr_Module, bl_Module, br_Module};
        for (int i = 0; i < 4; i++) {
            swerveModules[i].rotateTo(rotationAngles[i]);
            swerveModules[i].setMotorPower(wheelSpeeds[i]);
        }

    }
}
