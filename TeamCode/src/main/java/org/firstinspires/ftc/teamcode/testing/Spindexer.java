package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


    public class Spindexer {

        // constants

        private final Servo ejectorServo;
        private static final double RETRACT_POS = 0.0;   // tune on robot
        private static final double FLIP_POS    = 0.6;   // tune on robot

        private static final double CPR_MOTOR = 750;        // counts per motor shaft revolution on encoder idk the number
        private static final double GEAR_RATIO = 1.0;         // plate revs per motor rev denominator (1.0 => 1:1) change if needed
        private static final double CPR_PLATE = CPR_MOTOR / GEAR_RATIO;

        private static final double TICKS_PER_SLOT = CPR_PLATE / 3.0; // 120°
        private static final double TICKS_PER_60   = CPR_PLATE / 6.0; // 60°

        private static final PIDFCoefficients POS_PIDF = new PIDFCoefficients(
                8.0,   // kP
                0.0,   // kI
                0.6,   // kD
                0.0    // kF
        );

        private static final int TARGET_TOL = 2;    // tick tolerance for RUN_TO_POSITION
        private static final double MAX_POWER = 0.6;

        private static final int PRESENCE_ALPHA_THRESHOLD = 5; // tune for your sensor/lighting

        // ===== TYPES =====
        public enum Ball { EMPTY, PURPLE, GREEN }

        // ===== HARDWARE =====
        private final DcMotorEx motor;
        private final ColorSensor intakeColor;

        // ===== STATE =====
        private int zeroCount = 0;      // encoder tick value for slot 0 at intake
        private double accum = 0.0;     // cumulative (floating) encoder target; drift-free rounding
        private int targetCounts = 0;   // last set RUN_TO_POSITION target
        private int intakeIndex = 0;    // which absolute slot is at intake stop

        private final Ball[] slots = { Ball.EMPTY, Ball.EMPTY, Ball.EMPTY };

        private Integer findFirst(Ball color) {
            for (int i = 0; i < 3; i++) {
                if (slots[i] == color) {
                    return i;
                }
            }
            return null;
        }

        public Spindexer(DcMotorEx motor, ColorSensor intakeColor, Servo ejectorServo) {
            this.motor = motor;
            this.intakeColor = intakeColor;
            this.ejectorServo = ejectorServo;

            // Motor config
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            try {
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, POS_PIDF);
            } catch (Exception ignored) {}
            motor.setTargetPositionTolerance(TARGET_TOL);

            zeroCount = motor.getCurrentPosition();
            accum = 0.0;
            targetCounts = motor.getCurrentPosition();
            motor.setTargetPosition(targetCounts);
            motor.setPower(MAX_POWER);

            // Start flipper out of the way
            if (ejectorServo != null) {
                ejectorServo.setPosition(RETRACT_POS);
            }
        }



        /** Read color at intake, store it in the current intake slot, then rotate +120° without ejecting. */
        public void intakeOne(Telemetry telemetry) {
            Ball color = readColorAtIntake();
            slots[intakeIndex] = color;

            // Move +120° as two half-steps; do not clear any slot
            move120NoEject(telemetry);

            // Next slot is now at intake
            intakeIndex = mod3(intakeIndex + 1);
        }

        /** Eject all balls with GREEN first, then all PURPLE. */
        public void ejectAllGreenThenPurple(Telemetry telemetry) {
            Integer greenIdx = findFirst(Ball.GREEN);
            if (greenIdx != null) {
                ejectSlot(greenIdx, telemetry);
            }
            // Eject both PURPLEs (if present)
            for (int i = 0; i < 3; i++) {
                if (slots[i] == Ball.PURPLE) {
                    ejectSlot(i, telemetry);
                }
            }
        }

        public void rezeroHere() {
            zeroCount = motor.getCurrentPosition();
            accum = 0.0;
            setTarget(motor.getCurrentPosition());
            intakeIndex = 0; // define current intake stop as slot 0
        }

        public Ball[] getSlots() { return slots.clone(); }
        public int getIntakeIndex() { return intakeIndex; }
        public int getEncoder() { return motor.getCurrentPosition(); }
        public int getTarget() { return targetCounts; }

        // ===== INTERNAL — Ejection helpers =====

        /** Eject a specific absolute slot (0..2): align, +60° to eject (clear), +60° to finish; update intakeIndex. */
        private void ejectSlot(int slotIndex, Telemetry telemetry) {
            // Align so that after a +60° half-step, 'slotIndex' will be at eject.
            // Eject slot at +60° is (intakeIndex + 1) % 3 from the current intake stop.
            // Therefore, we need intakeIndex == slotIndex - 1 (mod 3).
            int deltaSlots = mod3(slotIndex - 1 - intakeIndex);
            for (int k = 0; k < deltaSlots; k++) {
                move120NoEject(telemetry); // align in 120° chunks, do NOT clear while aligning
                intakeIndex = mod3(intakeIndex + 1);
            }

            // Now +60° puts 'slotIndex' at eject
            int mid = stepForward60();
            goTo(mid, telemetry);


// === actuate ejector el finger ===
            if (ejectorServo != null) {
                // swing through the slot
                ejectorServo.setPosition(FLIP_POS);

                ejectorServo.setPosition(RETRACT_POS);
            }


            slots[slotIndex] = Ball.EMPTY;

            slots[slotIndex] = Ball.EMPTY;


            int nextStop = stepForward60();
            goTo(nextStop, telemetry);


            intakeIndex = mod3(intakeIndex + 1);
        }

        // ===== INTERNAL Movement =====

        private int mod3(int x) { return ((x % 3) + 3) % 3; }

        private int stepForward60() {
            accum += TICKS_PER_60;                      // +60°
            return (int)Math.rint(zeroCount + accum);   // cumulative rounding to minimize drift
        }

        /** Move +120° as two half-steps */
        private void move120NoEject(Telemetry telemetry) {
            int mid = stepForward60();
            goTo(mid, telemetry);
            int stop = stepForward60();
            goTo(stop, telemetry);
        }

        private void setTarget(int t) {
            targetCounts = t;
            motor.setTargetPosition(targetCounts);
            if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            motor.setPower(MAX_POWER);
        }

        private void goTo(int t, Telemetry telemetry) {
            setTarget(t);
            while (opModeIsActive() && motor.isBusy()) {
                motor.setPower(MAX_POWER);
                // Optional live telemetry while moving
                if (telemetry != null) {
                    telemetry.addData("enc", motor.getCurrentPosition());
                    telemetry.addData("target", targetCounts);
                    telemetry.addData("intakeIndex", intakeIndex);
                    telemetry.addData("slots", "%s | %s | %s", slots[0], slots[1], slots[2]);
                    telemetry.update();
                }
            }
        }

        // ===== INTERNAL — Sensors / Color =====

        private boolean ballPresentAtIntake() {
            int alpha = intakeColor.alpha();
            return alpha > PRESENCE_ALPHA_THRESHOLD;
        }

        private Ball readColorAtIntake() {
            // If nothing sensed, return EMPTY to avoid corrupting inventory
            if (!ballPresentAtIntake()) return Ball.EMPTY;

            int r = intakeColor.red();
            int g = intakeColor.green();
            int b = intakeColor.blue();

            boolean isPurple = (b > g + 5) && (r > g + 5);
            boolean isGreen  = (g > r + 5) && (g > b + 5);

            if (isGreen && !isPurple) return Ball.GREEN;
            if (isPurple && !isGreen) return Ball.PURPLE;

            // Fallback: choose dominant channel (simple, tune as needed)
            if (g >= r && g >= b) return Ball.GREEN;
            return Ball.PURPLE;
        }

        // ===== UTIL =====

        private boolean opModeIsActive() {
            
            return true;
        }
    }

