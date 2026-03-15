package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class RecyclerCommand implements Subsystem {

    public static final RecyclerCommand INSTANCE = new RecyclerCommand();
    private RecyclerCommand() {}

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
        gate = ActiveOpMode.hardwareMap().get(Servo.class, "gate");
        colorSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, "color");

        closeGate();
    }

    @Override
    public void periodic() {
        // Continuously check the sensor and open/close gate
        if (selectedColor == ColorChoice.GREEN) {
            // Keep gate closed if green detected, open if not
            if (isGreen()) {
                closeGate();
            } else {
                openGate();
            }
        } else { // assuming selectedColor == PURPLE
            if (isPurple()) {
                closeGate();
            } else {
                openGate();
            }
        }
    }

    public void openGate() {
        gate.setPosition(GATE_OPEN);
    }

    public void closeGate() {
        gate.setPosition(GATE_CLOSED);
    }

    public void selectGreen() {
        selectedColor = ColorChoice.GREEN;
    }

    public void selectPurple() {
        selectedColor = ColorChoice.PURPLE;
    }

    public ColorChoice getSelectedColor() {
        return selectedColor;
    }

    // checking the color okok
    public boolean isGreen() {
        return colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue();
    }

    public boolean isPurple() {
        return colorSensor.red() > colorSensor.green() && colorSensor.blue() > colorSensor.green();
    }
}