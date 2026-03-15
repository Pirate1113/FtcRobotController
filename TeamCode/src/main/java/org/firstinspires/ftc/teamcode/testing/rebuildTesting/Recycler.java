package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Recycler implements Subsystem {

    public static final Recycler INSTANCE = new Recycler();
    Recycler() {}

    private Servo gate;
    private ColorSensor colorSensor;

    private final double GATE_OPEN = 0.8;
    private final double GATE_CLOSED = 0.2;

    public enum ColorChoice {
        GREEN,
        PURPLE
    }

    private ColorChoice selectedColor = ColorChoice.GREEN;

    @Override
    public void initialize() {
        // Initialize hardware
        gate = ActiveOpMode.hardwareMap().get(Servo.class, "gate");
        colorSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, "color");

        // Start with gate closed
        closeGate();
    }

    @Override
    public void periodic() {
        // Continuously check color sensor and control gate
        if (selectedColor == ColorChoice.GREEN) {
            if (isGreen()) {
                closeGate(); // keep gate closed IF green is detected
            } else {
                openGate();  // open gate for purplepurple
            }
        } else { // PURPLE selected
            if (isPurple()) {
                closeGate(); // keep gate closed if purple is detected
            } else {
                openGate();  // open gate for green
            }
        }
    }

    // gate control methods
    public void openGate() {
        gate.setPosition(GATE_OPEN);
    }

    public void closeGate() {
        gate.setPosition(GATE_CLOSED);
    }

    // gamepad selection
    public void selectGreen() {
        selectedColor = ColorChoice.GREEN;
    }

    public void selectPurple() {
        selectedColor = ColorChoice.PURPLE;
    }

    public ColorChoice getSelectedColor() {
        return selectedColor;
    }

    // Color detection methods
    public boolean isGreen() {
        return colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue();
    }

    public boolean isPurple() {
        return colorSensor.red() > colorSensor.green() && colorSensor.blue() > colorSensor.green();
    }
}


