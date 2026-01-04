package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Spindexer Forward/Backward", group="Test")
public class SpindexerTest extends LinearOpMode {

    private CRServo servoLeft;
    private CRServo servoRight;

    // Servo speed / step for manual control (0–1 scale)
    private static final double SERVO_SPEED = 0.005;



    DcMotorEx intakeleft;
    DcMotorEx intakeright;

    double powerLeft = 0.0;
    double powerRight = 0.0;

    boolean previousDpadUp;
    boolean previousDpadDown;
    boolean previousX;
    boolean previousB;


    @Override
    public void runOpMode() {

        // Initialize hardware
        servoLeft = hardwareMap.get(CRServo.class, "spindexerleft");
        servoRight = hardwareMap.get(CRServo.class, "spindexerright");
        intakeleft = hardwareMap.get(DcMotorEx.class, "intakeleft");
        intakeright = hardwareMap.get(DcMotorEx.class, "intakeright");

        intakeleft.setDirection(DcMotor.Direction.REVERSE);
        intakeleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeright.setDirection(DcMotor.Direction.FORWARD);
        intakeright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set initial positions
        servoLeft.setPower(0);
        servoRight.setPower(0);

        waitForStart();

        while (opModeIsActive()) {

            double leftPow = servoLeft.getPower();
            double rightPow = servoRight.getPower();
            //spindexer control
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

            // intake control
            if (gamepad1.dpad_up && !previousDpadUp) {
                powerLeft += 0.1;
            } else if (gamepad1.dpad_down && !previousDpadDown) {
                powerLeft -= 0.1;
            }
            // Right intake control with X (square) and B (circle)
            if (gamepad1.x && !previousX) {
                powerRight += 0.1;
            } else if (gamepad1.b && !previousB) {
                powerRight -= 0.1;
            }

            // Set motor powers
            intakeleft.setPower(powerLeft);
            intakeright.setPower(powerRight);

            // Update previous button states
            previousDpadUp = gamepad1.dpad_up;
            previousDpadDown = gamepad1.dpad_down;
            previousX = gamepad1.x;
            previousB = gamepad1.b;

            // Telemetry for debugging

            // Update power
            servoLeft.setPower(leftPow);
            servoRight.setPower(rightPow);

            // Telemetry
            telemetry.addData("Left Servo", "%.2f", leftPow);
            telemetry.addData("Right Servo", "%.2f", rightPow);
            telemetry.addData("Left Intake Power", powerLeft);
            telemetry.addData("Right Intake Power", powerRight);
            telemetry.addData("previousX", previousX);
            telemetry.addData("previousB", previousB);
            telemetry.addData("previousDpadUp", previousDpadUp);
            telemetry.addData("previousDpadDown", previousDpadDown);
            telemetry.update();
        }

    }
}