package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class ColorSensor implements Subsystem {
    public static final ColorSensor INSTANCE = new ColorSensor();

    private ColorSensor() {}

    private NormalizedColorSensor colorSensor;
    private float gain = 2.0f;

    public void initialize() {
        colorSensor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(gain);
    }

    // ===== COLOR DETECTION =====

    /** Get the current RGBA values from the sensor */
    public NormalizedRGBA getRGBA() {
        return colorSensor.getNormalizedColors();
    }

    /** Get red value (0-1) */
    public float getRed() {
        return colorSensor.getNormalizedColors().red;
    }

    /** Get green value (0-1) */
    public float getGreen() {
        return colorSensor.getNormalizedColors().green;
    }

    /** Get blue value (0-1) */
    public float getBlue() {
        return colorSensor.getNormalizedColors().blue;
    }

    /** Get alpha/intensity value (0-1) */
    public float getAlpha() {
        return colorSensor.getNormalizedColors().alpha;
    }

    // ===== COLOR IDENTIFICATION =====

    public enum DetectedColor {
        GREEN, PURPLE, NONE
    }

    private float[] hsv = new float[3];

    /** Update HSV values from the sensor */
    private void updateHSV() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        android.graphics.Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );
    }

    /** Detect which game element color is present */
    public DetectedColor getDetectedColor() {
        updateHSV();
        float hue = hsv[0];

        // Purple detection (hue 260-300)
        if (hue >= 260 && hue <= 300) {
            return DetectedColor.PURPLE;
        }
        // Green detection (hue 90-150)
        if (hue >= 90 && hue <= 150) {
            return DetectedColor.GREEN;
        }

        return DetectedColor.NONE;
    }

    /** Get current hue value (0-360) */
    public float getHue() {
        updateHSV();
        return hsv[0];
    }

    /** Get current saturation value (0-1) */
    public float getSaturation() {
        updateHSV();
        return hsv[1];
    }

    /** Get current value/brightness (0-1) */
    public float getValue() {
        updateHSV();
        return hsv[2];
    }

    /** Check if a specific color is detected */
    public boolean isColor(DetectedColor target) {
        return getDetectedColor() == target;
    }

    /** Check if any game element color is detected */
    public boolean hasGameElement() {
        return getDetectedColor() != DetectedColor.NONE;
    }

    // ===== CONFIGURATION =====

    /** Set the sensor gain (higher = more sensitive) */
    public void setGain(float newGain) {
        this.gain = newGain;
        colorSensor.setGain(gain);
    }

    public float getGain() {
        return gain;
    }
}
