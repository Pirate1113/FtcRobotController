package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

public class Recycler implements Subsystem {

    public static final Recycler INSTANCE = new Recycler();

    private Recycler() {
    }

    private ServoEx gateServo;
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
        gateServo = new ServoEx(RobotConstants.gate);
        colorSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, "color");

        // Start with gate closed
        gateServo.getServo().setPosition(GATE_CLOSED);
    }

    @Override
    public void periodic() {
        // Only act on confident color reads; hold current gate state in the dead zone
        if (selectedColor == ColorChoice.GREEN) {
            if (isGreen()) {
                openGate.schedule();  // open gate to let green through
            } else if (isPurple()) {
                closeGate.schedule(); // close gate to block purple
            }
            // dead zone: neither detected — hold current state
        } else { // PURPLE selected
            if (isPurple()) {
                openGate.schedule();  // open gate to let purple through
            } else if (isGreen()) {
                closeGate.schedule(); // close gate to block green
            }
            // dead zone: neither detected — hold current state
        }
    }

    // gate control methods
    public Command openGate = new InstantCommand(() -> {
        gateServo.getServo().setPosition(GATE_OPEN);
    });

    public Command closeGate = new InstantCommand(() -> {
        gateServo.getServo().setPosition(GATE_CLOSED);
    });

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

    public double getGatePosition() {
        return gateServo.getServo().getPosition();
    }

    // Raw sensor values
    public int rawRed()   { return colorSensor.red(); }
    public int rawGreen() { return colorSensor.green(); }
    public int rawBlue()  { return colorSensor.blue(); }

    // Color detection methods
    public boolean isGreen() {
        return colorSensor.green() > colorSensor.blue();
    }

    public boolean isPurple() {
        return colorSensor.blue() > colorSensor.green();
    }
}