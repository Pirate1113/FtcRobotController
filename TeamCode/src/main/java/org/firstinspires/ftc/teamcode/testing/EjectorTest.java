package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class EjectorTest extends OpMode {

    private Servo ejector;
    private boolean isForward = false;
    private boolean previousRightBumper = false;

    // Servo positions
    private static final double FORWARD_POSITION = 1.0;
    private static final double BACKWARD_POSITION = 0.0;

    @Override
    public void init() {
        ejector = hardwareMap.get(Servo.class, "ejectorServe");
        ejector.setPosition(BACKWARD_POSITION);
    }

    @Override
    public void loop() {
        // Toggle ejector position when right bumper is pressed
        if (gamepad1.right_bumper && !previousRightBumper) {
            isForward = !isForward;
            if (isForward) {
                ejector.setPosition(FORWARD_POSITION);
            } else {
                ejector.setPosition(BACKWARD_POSITION);
            }
        }

        // Update previous button state
        previousRightBumper = gamepad1.right_bumper;

        // Telemetry
        telemetry.addData("Ejector Position", ejector.getPosition());
        telemetry.addData("State", isForward ? "Forward" : "Backward");
        telemetry.update();
    }
}
