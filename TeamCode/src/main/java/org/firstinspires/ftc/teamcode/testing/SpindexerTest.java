package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Spindexer Forward/Backward", group="Test")
public class SpindexerTest extends LinearOpMode {

    private CRServo servoLeft;
    private CRServo servoRight;

    // Servo speed / step for manual control (0–1 scale)
    private static final double SERVO_SPEED = 0.005;

    @Override
    public void runOpMode() {

        // Initialize hardware
        servoLeft = hardwareMap.get(CRServo.class, "spindexerleft");
        servoRight = hardwareMap.get(CRServo.class, "spindexerright");

        // Set initial positions
        servoLeft.setPower(0);
        servoRight.setPower(0);

        waitForStart();

        while (opModeIsActive()) {

            double leftPow = servoLeft.getPower();
            double rightPow = servoRight.getPower();

            // Forward control: right bumper
            if (gamepad1.right_bumper) {
                leftPow = Math.min(1.0, leftPow + SERVO_SPEED);
                rightPow = Math.min(1.0, rightPow + SERVO_SPEED);
            }

            // Backward control: left bumper
            if (gamepad1.left_bumper) {
                leftPow = Math.max(-1.0, leftPow - SERVO_SPEED);
                rightPow = Math.max(-1.0, rightPow - SERVO_SPEED);
            }

            // Update power
            servoLeft.setPower(leftPow);
            servoRight.setPower(rightPow);

            // Telemetry
            telemetry.addData("Left Servo", "%.2f", leftPow);
            telemetry.addData("Right Servo", "%.2f", rightPow);
            telemetry.update();
        }
    }
}