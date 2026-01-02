package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Spindexer Forward/Backward", group="Test")
public class SpindexerTest extends LinearOpMode {

    private Servo servoLeft;
    private Servo servoRight;

    // Servo speed / step for manual control (0–1 scale)
    private static final double SERVO_SPEED = 0.01;

    @Override
    public void runOpMode() {

        // Initialize hardware
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        // Set initial positions
        servoLeft.setPosition(0.5);
        servoRight.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {

            double leftPos = servoLeft.getPosition();
            double rightPos = servoRight.getPosition();

            // Forward control: right bumper
            if (gamepad1.right_bumper) {
                leftPos  = Math.min(1.0, leftPos + SERVO_SPEED);
                rightPos = Math.min(1.0, rightPos + SERVO_SPEED);
            }

            // Backward control: left bumper
            if (gamepad1.left_bumper) {
                leftPos  = Math.max(0.0, leftPos - SERVO_SPEED);
                rightPos = Math.max(0.0, rightPos - SERVO_SPEED);
            }

            // Update servo positions
            servoLeft.setPosition(leftPos);
            servoRight.setPosition(rightPos);

            // Telemetry
            telemetry.addData("Left Servo", "%.2f", leftPos);
            telemetry.addData("Right Servo", "%.2f", rightPos);
            telemetry.update();
        }
    }
}