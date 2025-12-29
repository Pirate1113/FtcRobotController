package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Spindexer {

    // constants
    private static final int PRESENCE_ALPHA_THRESHOLD = 5; // tune for sensor/lighting

    // SERVO POSITIONS - tune on robot idk actual PUT ACTUAL
    private static final double SLOT0_POS = 0.10;  // Slot 0 at intake
    private static final double SLOT1_POS = 0.43;  // Slot 1 at intake (~120)
    private static final double SLOT2_POS = 0.76;  // Slot 2 at intake (~240)

    private static final double EJECT0_POS = 0.26; // 60° past slot0 (eject position)
    private static final double EJECT1_POS = 0.60; // 60° past slot1
    private static final double EJECT2_POS = 0.93; // 60° past slot2

    private static final double EJECTOR_RETRACT_POS = 0.0;  // tune on robot
    private static final double EJECTOR_FLIP_POS = 0.6;     // tune on robot

    // ===== TYPES =====
    public enum Ball { EMPTY, PURPLE, GREEN }

    // ===== HARDWARE =====
    private final Servo servoLeft;
    private final Servo servoRight;
    private final Servo ejectorServo;
    private final ColorSensor intakeColor;

    // ===== STATE =====
    private int intakeIndex = 0;  // which slot (0,1,2) is at intake
    private final Ball[] slots = { Ball.EMPTY, Ball.EMPTY, Ball.EMPTY };

    public Spindexer(Servo servoLeft, Servo servoRight, Servo ejectorServo, ColorSensor intakeColor) {
        this.servoLeft = servoLeft;
        this.servoRight = servoRight;
        this.ejectorServo = ejectorServo;
        this.intakeColor = intakeColor;

        // Initialize to slot 0 at intake, ejector el finger retracted
        goToSlot(0);
        if (ejectorServo != null) {
            ejectorServo.setPosition(EJECTOR_RETRACT_POS);
        }
    }

    //

    /** Read color at intake, store it, rotate to next slot (+120) */
    public void intakeOne(Telemetry telemetry) {
        Ball color = readColorAtIntake();
        slots[intakeIndex] = color;

        // Move to next slot
        intakeIndex = mod3(intakeIndex + 1);
        goToSlot(intakeIndex);
    }

    /** Eject all GREEN first, then all PURPLE */
    public void ejectAllGreenThenPurple(Telemetry telemetry) {
        Integer greenIdx = findFirst(Ball.GREEN);
        if (greenIdx != null) {
            ejectSlot(greenIdx, telemetry);
        }
        // Eject all PURPLEs
        for (int i = 0; i < 3; i++) {
            if (slots[i] == Ball.PURPLE) {
                ejectSlot(i, telemetry);
            }
        }
    }

    /** Re-zero: declare current position as slot 0 at intake */
    public void rezeroHere(int newSlotIndex) {
        intakeIndex = newSlotIndex;
        goToSlot(intakeIndex);
    }

    public Ball[] getSlots() { return slots.clone(); }
    public int getIntakeIndex() { return intakeIndex; }

    // =

    private Integer findFirst(Ball color) {
        for (int i = 0; i < 3; i++) {
            if (slots[i] == color) {
                return i;
            }
        }
        return null;
    }


    private void ejectSlot(int slotIndex, Telemetry telemetry) {
        // Rotate so this slot is at its eject position (60° past intake stop)
        goToEjectForSlot(slotIndex);

        // Actuate ejector flipper
        if (ejectorServo != null) {
            ejectorServo.setPosition(EJECTOR_FLIP_POS);
            // Short delay would go here in real OpMode (non-blocking)
            ejectorServo.setPosition(EJECTOR_RETRACT_POS);
        }

        // Mark slot as empty
        slots[slotIndex] = Ball.EMPTY;

        // Advance to next intake stop
        intakeIndex = mod3(intakeIndex + 1);
        goToSlot(intakeIndex);
    }

    private int mod3(int x) {
        return ((x % 3) + 3) % 3;
    }

    private void goToSlot(int slot) {
        double pos = getSlotPosition(slot);
        setServos(pos);
        intakeIndex = slot;
    }

    private void goToEjectForSlot(int slot) {
        double pos = getEjectPosition(slot);
        setServos(pos);
    }

    private double getSlotPosition(int slot) {
        switch (slot) {
            case 0: return SLOT0_POS;
            case 1: return SLOT1_POS;
            case 2: return SLOT2_POS;
            default: return SLOT0_POS;
        }
    }

    private double getEjectPosition(int slot) {
        switch (slot) {
            case 0: return EJECT0_POS;
            case 1: return EJECT1_POS;
            case 2: return EJECT2_POS;
            default: return EJECT0_POS;
        }
    }

    public void init() {
        intakeIndex = 0;
        goToSlot(0);

        if (ejectorServo != null) {
            ejectorServo.setPosition(EJECTOR_RETRACT_POS);
        }
    }

    private void setServos(double position) {
        servoLeft.setPosition(position);
        servoRight.setPosition(position);  // Change to (1.0 - position) if mirrored
    }

    // ===== color
    private boolean ballPresentAtIntake() {
        int alpha = intakeColor.alpha();
        return alpha > PRESENCE_ALPHA_THRESHOLD;
    }

    private Ball readColorAtIntake() {
        if (!ballPresentAtIntake()) return Ball.EMPTY;

        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();

        boolean isPurple = (b > g + 5) && (r > g + 5);
        boolean isGreen  = (g > r + 5) && (g > b + 5);

        if (isGreen && !isPurple) return Ball.GREEN;
        if (isPurple && !isGreen) return Ball.PURPLE;

        if (g >= r && g >= b) return Ball.GREEN;
        return Ball.PURPLE;
    }
}