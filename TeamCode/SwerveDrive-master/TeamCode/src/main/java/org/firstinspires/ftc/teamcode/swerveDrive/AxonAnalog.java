package org.firstinspires.ftc.teamcode.swerveDrive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.ftc.ActiveOpMode;

public class AxonAnalog {
    private AnalogInput absoluteInput;
    private HardwareMap hwMap = ActiveOpMode.hardwareMap();
    private String name;
    private double offset;
    private double voltCompensation;
    private boolean reversed;
    private static final double TWO_PI = 2 * Math.PI;

    public AxonAnalog(String configName, double zeroOffset, double realMaxvolts, boolean isReversed){
        name = configName;
        offset = zeroOffset;
        voltCompensation = realMaxvolts;
        reversed = isReversed;
        absoluteInput = hwMap.get(AnalogInput.class, name);
    }

    private static double normalizeHeading(double inputHeading){
        return ((inputHeading % TWO_PI)+TWO_PI) % TWO_PI;
    }

    public double getHeading(){
        double rawVoltage = absoluteInput.getVoltage();
        double rawAngle;
        if (reversed){
            rawAngle = (voltCompensation - rawVoltage) / voltCompensation * TWO_PI;
        } else {
            rawAngle = rawVoltage / voltCompensation * TWO_PI;
        }
        double newAngle = rawAngle - offset;
        return normalizeHeading(newAngle);
    }
}
