package org.firstinspires.ftc.teamcode.swerveDrive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class SwerveDrivetrainNew implements Subsystem {
    public static final SwerveDrivetrainNew INSTANCE = new SwerveDrivetrainNew();
    private SwerveDrivetrainNew() {}

    private final double ANALOG_VOLTAGE_COMPENSATION = 3.3;
    public static final double cacheTolerance = 0.1;

    public IMU imu;
    public RevHubOrientationOnRobot hubOrient = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.UP);
    public SwerveModule fl_Module, bl_Module, br_Module, fr_Module;
    public SwerveModule[] swerveModules;

    double[] wheelSpeeds, targetAngles, currentAngles, angleErrors, cachedAngles;

    @Override
    public void initialize(){
        fl_Module = new SwerveModule(new MotorEx("fl_motor").reversed(), "fl_rotation", true,
                "fl_absolute", 3.202, true, ANALOG_VOLTAGE_COMPENSATION, -1, 1);

        bl_Module = new SwerveModule(new MotorEx("bl_motor").reversed(), "bl_rotation", true,
                "bl_absolute", 1.613, true, ANALOG_VOLTAGE_COMPENSATION, -1, -1);

        br_Module = new SwerveModule(new MotorEx("br_motor"), "br_rotation", true,
                "br_absolute", 0.208, true, ANALOG_VOLTAGE_COMPENSATION, 1, -1);

        fr_Module = new SwerveModule(new MotorEx("fr_motor"), "fr_rotation", true,
                "fr_absolute", 1.957, true, ANALOG_VOLTAGE_COMPENSATION, 1, 1);

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
//        double fwd = -ActiveOpMode.gamepad1().left_stick_y;
//        double str =  ActiveOpMode.gamepad1().left_stick_x;
//        double rot =  ActiveOpMode.gamepad1().right_stick_x;

//        ActiveOpMode.telemetry().addData("fwd", fwd);
//        ActiveOpMode.telemetry().addData("str", str);
//        ActiveOpMode.telemetry().addData("rot", rot);
//        ActiveOpMode.telemetry().update();


//        ActiveOpMode.telemetry().addData("rawLeftX", rawLeftX);
//        ActiveOpMode.telemetry().addData("rawLeftY", rawLeftY);
        ActiveOpMode.telemetry().addData("rawRightX", rawRightX);
        ActiveOpMode.telemetry().addData("realRightX", realRightX);

        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double tempX = rawLeftX;
        double tempY = rawLeftY;
//        rawLeftX = tempX * Math.cos(-imuHeading) - tempY * Math.sin(-imuHeading);
//        rawLeftY = tempX * Math.sin(-imuHeading) + tempY * Math.cos(-imuHeading);
        rawLeftY = tempX * Math.cos(-imuHeading) - tempY * Math.sin(-imuHeading);
        rawLeftX = tempX * Math.sin(-imuHeading) + tempY * Math.cos(-imuHeading);

        ActiveOpMode.telemetry().addData("imuHeading(rad)", imuHeading);
        ActiveOpMode.telemetry().addData("robotX", rawLeftX);
        ActiveOpMode.telemetry().addData("robotY", rawLeftY);

//
//        ActiveOpMode.telemetry().addData("rawLeftX after", rawLeftX);
//        ActiveOpMode.telemetry().addData("rawLeftY after", rawLeftY);
//        ActiveOpMode.telemetry().update();


        for (int i = 0; i < swerveModules.length; i++) {
            double rotVectorY =-1* realRightX * swerveModules[i].yOffset;
            double rotVectorX = realRightX * swerveModules[i].xOffset;

            double resultX = rawLeftX + rotVectorX;
            double resultY = rawLeftY + rotVectorY;

            // Compute final speed + angle
            wheelSpeeds[i] = Math.sqrt(resultX * resultX + resultY * resultY);
            targetAngles[i] = Math.atan2(resultY, resultX);
//            targetAngles[i] = Math.atan2(resultY, resultX) - Math.PI/2;
//            targetAngles[i] = (targetAngles[i] + 2*Math.PI) % (2*Math.PI);

            currentAngles[i] = swerveModules[i].getPodHeading();
            ActiveOpMode.telemetry().addData("Module " + i + " rotVectorX", rotVectorX);
            ActiveOpMode.telemetry().addData("Module " + i + " rotVectorY", rotVectorY);
            ActiveOpMode.telemetry().addData("Module " + i + " resultX", resultX);
            ActiveOpMode.telemetry().addData("Module " + i + " resultY", resultY);
            ActiveOpMode.telemetry().addData("Module " + i + " targetAngle", targetAngles[i]);
            ActiveOpMode.telemetry().addData("Module " + i + " targetAngle(deg)", Math.toDegrees(targetAngles[i]));
            ActiveOpMode.telemetry().addData("Module " + i + " currentAngle", currentAngles[i]);
            ActiveOpMode.telemetry().addData("Module " + i + " currentAngle(deg)", Math.toDegrees(currentAngles[i]));
            ActiveOpMode.telemetry().addData("Module " + i + " angleError(deg)",
                    Math.toDegrees(Math.atan2(Math.sin(targetAngles[i] - currentAngles[i]),
                            Math.cos(targetAngles[i] - currentAngles[i]))));

        }
        ActiveOpMode.telemetry().update();


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