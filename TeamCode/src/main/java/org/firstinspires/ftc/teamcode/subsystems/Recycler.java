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

    // Baseline sensor readings when nothing is in the recycler
    private static final int EMPTY_RED   = 42;
    private static final int EMPTY_GREEN = 70;
    private static final int EMPTY_BLUE  = 58;
    private static final int EMPTY_ERROR = 10;   // ±4 on each channel

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
        boolean wantGreen = selectedColor == ColorChoice.GREEN;
        if ((wantGreen && isGreen()) || (!wantGreen && isPurple())) {
            openGate.schedule();
        } else {
            closeGate.schedule(); // undesired color OR no ball → hold closed
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
    public boolean isEmpty() {
        return Math.abs(colorSensor.red()   - EMPTY_RED)   <= EMPTY_ERROR
            && Math.abs(colorSensor.green() - EMPTY_GREEN) <= EMPTY_ERROR
            && Math.abs(colorSensor.blue()  - EMPTY_BLUE)  <= EMPTY_ERROR;
    }

    public boolean isGreen() {
        return !isEmpty() && colorSensor.green() > colorSensor.blue() + 10;
    }

    public boolean isPurple() {
        return !isEmpty() && colorSensor.blue() > colorSensor.green() + 10;
    }
}