package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Shooter TeleOp", group="TeleOp")
public class Shootertest extends OpMode {

    // Hardware
    private DcMotor shooter1;
    private DcMotor shooter2;
    private Servo ejectorServo;
    private Servo feedServo;

    // Shooter speed control
    private double shooterSpeed = 0.0;
    private final double SPEED_INCREMENT = 0.05;
    private final double MAX_SPEED = 1.0;
    private final double MIN_SPEED = 0.0;

    // Servo positions
    private final double EJECTOR_RETRACTED = 0.5;
    private final double EJECTOR_EXTENDED = 0;
    private final double FEED_RETRACTED = 0.5;
    private final double FEED_EXTENDED = 0.2;

    // Button debouncing
    private boolean dpadUpPreviousState = false;
    private boolean dpadDownPreviousState = false;
    private boolean ejectorButtonPreviousState = false;
    private boolean feedButtonPreviousState = false;

    @Override
    public void init() {
        // Initialize hardware
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        ejectorServo = hardwareMap.get(Servo.class, "ejectorServo");
        feedServo = hardwareMap.get(Servo.class, "feedServo");

        // Set motor directions (adjust if needed)
        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize servos to retracted position
        ejectorServo.setPosition(EJECTOR_RETRACTED);
        feedServo.setPosition(FEED_RETRACTED);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Shooter Speed", "%.2f", shooterSpeed);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Speed control with D-pad (with debouncing)
        boolean dpadUpCurrentState = gamepad1.dpad_up;
        boolean dpadDownCurrentState = gamepad1.dpad_down;

        // Increase speed with D-pad Up
        if (dpadUpCurrentState && !dpadUpPreviousState) {
            shooterSpeed += SPEED_INCREMENT;
            if (shooterSpeed > MAX_SPEED) {
                shooterSpeed = MAX_SPEED;
            }
        }

        // Decrease speed with D-pad Down
        if (dpadDownCurrentState && !dpadDownPreviousState) {
            shooterSpeed -= SPEED_INCREMENT;
            if (shooterSpeed < MIN_SPEED) {
                shooterSpeed = MIN_SPEED;
            }
        }

        // Update previous states
        dpadUpPreviousState = dpadUpCurrentState;
        dpadDownPreviousState = dpadDownCurrentState;

        // Set shooter motor powers
        shooter1.setPower(shooterSpeed);
        shooter2.setPower(shooterSpeed);

        // Ejector servo (right bumper)
        boolean ejectorButtonCurrentState = gamepad1.right_bumper;

        if (ejectorButtonCurrentState && !ejectorButtonPreviousState) {
            // Extend ejector servo
            ejectorServo.setPosition(EJECTOR_EXTENDED);
        } else if (!ejectorButtonCurrentState && ejectorButtonPreviousState) {
            // Retract ejector servo when button released
            ejectorServo.setPosition(EJECTOR_RETRACTED);
        }

        ejectorButtonPreviousState = ejectorButtonCurrentState;

        // Feed servo (left bumper)
        boolean feedButtonCurrentState = gamepad1.left_bumper;

        if (feedButtonCurrentState && !feedButtonPreviousState) {
            // Extend feed servo
            feedServo.setPosition(FEED_EXTENDED);
        } else if (!feedButtonCurrentState && feedButtonPreviousState) {
            // Retract feed servo when button released
            feedServo.setPosition(FEED_RETRACTED);
        }

        feedButtonPreviousState = feedButtonCurrentState;

        // Telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Shooter Speed", "%.2f (%.0f%%)", shooterSpeed, shooterSpeed * 100);
        telemetry.addData("Shooter 1 Power", "%.2f", shooter1.getPower());
        telemetry.addData("Shooter 2 Power", "%.2f", shooter2.getPower());
        telemetry.addData("Ejector Position", "%.2f", ejectorServo.getPosition());
        telemetry.addData("Feed Position", "%.2f", feedServo.getPosition());
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("D-pad Up", "Increase shooter speed");
        telemetry.addData("D-pad Down", "Decrease shooter speed");
        telemetry.addData("Right Bumper", "Ejector");
        telemetry.addData("Left Bumper", "Feed");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop motors and retract servos when op mode ends
        shooter1.setPower(0);
        shooter2.setPower(0);
        ejectorServo.setPosition(EJECTOR_RETRACTED);
        feedServo.setPosition(FEED_RETRACTED);
    }
}