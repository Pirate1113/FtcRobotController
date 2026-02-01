package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class ColorSensor implements Subsystem {
    public static final ColorSensor INSTANCE = new ColorSensor();

    private ColorSensor() {}

    // Spindexer sensors (one for each third)
    private NormalizedColorSensor colorLeft;
    private NormalizedColorSensor colorRight;
    private NormalizedColorSensor colorFloor;
    // Shooter sensor
    private NormalizedColorSensor colorShooter;

    private float gain = 2.0f;

    public void initialize() {
        colorLeft = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "colorleft");
        colorRight = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "colorright");
        colorFloor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "colorfloor");
        colorShooter = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "colorshooter");

        colorLeft.setGain(gain);
        colorRight.setGain(gain);
        colorFloor.setGain(gain);
        colorShooter.setGain(gain);
    }

    public enum DetectedColor {
        GREEN, PURPLE, NONE
    }

    public enum Sensor {
        LEFT, RIGHT, FLOOR, SHOOTER
    }

    private NormalizedColorSensor getSensor(Sensor sensor) {
        switch (sensor) {
            case LEFT: return colorLeft;
            case RIGHT: return colorRight;
            case FLOOR: return colorFloor;
            case SHOOTER: return colorShooter;
            default: return colorLeft;
        }
    }

    // ===== COLOR DETECTION =====

    private float[] rgbToHsv(NormalizedRGBA colors) {
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );
        return hsv;
    }

    public DetectedColor getDetectedColor(Sensor sensor) {
        float[] hsv = rgbToHsv(getSensor(sensor).getNormalizedColors());
        float hue = hsv[0];

        if (hue >= 260 && hue <= 300) {
            return DetectedColor.PURPLE;
        }
        if (hue >= 90 && hue <= 150) {
            return DetectedColor.GREEN;
        }
        return DetectedColor.NONE;
    }

    public float getHue(Sensor sensor) {
        return rgbToHsv(getSensor(sensor).getNormalizedColors())[0];
    }

    public NormalizedRGBA getRGBA(Sensor sensor) {
        return getSensor(sensor).getNormalizedColors();
    }

    // ===== SPINDEXER HELPERS =====

    public DetectedColor getLeftColor() {
        return getDetectedColor(Sensor.LEFT);
    }

    public DetectedColor getRightColor() {
        return getDetectedColor(Sensor.RIGHT);
    }

    public DetectedColor getFloorColor() {
        return getDetectedColor(Sensor.FLOOR);
    }

    public boolean hasLeftSample() {
        return getLeftColor() != DetectedColor.NONE;
    }

    public boolean hasRightSample() {
        return getRightColor() != DetectedColor.NONE;
    }

    public boolean hasFloorSample() {
        return getFloorColor() != DetectedColor.NONE;
    }

    public int getSampleCount() {
        int count = 0;
        if (hasLeftSample()) count++;
        if (hasRightSample()) count++;
        if (hasFloorSample()) count++;
        return count;
    }

    // ===== SHOOTER HELPERS =====

    public DetectedColor getShooterColor() {
        return getDetectedColor(Sensor.SHOOTER);
    }

    public boolean hasShooterSample() {
        return getShooterColor() != DetectedColor.NONE;
    }

    // ===== CONFIGURATION =====

    public void setGain(float newGain) {
        this.gain = newGain;
        colorLeft.setGain(gain);
        colorRight.setGain(gain);
        colorFloor.setGain(gain);
        colorShooter.setGain(gain);
    }

    public float getGain() {
        return gain;
    }
}