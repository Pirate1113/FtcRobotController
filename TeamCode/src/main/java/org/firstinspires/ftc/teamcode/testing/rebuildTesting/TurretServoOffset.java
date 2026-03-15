package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

@TeleOp(name="Axon Turret Full Sweep")
public class TurretServoOffset extends OpMode {

    Servo rt;
    Servo lt;

    // Range limits as requested
    private final double MIN_POS = 0.00;
    private final double MAX_POS = 1.0;

    private double ltPos = 0.5;
    private double sweepSpeed = 0.4; // Percent of range per second (0.4 = 40% per sec)
    private int direction = 1;       // 1 for increasing, -1 for decreasing

    private boolean autoSweep = false;
    private boolean lastA = false;

    @Override
    public void init() {
        rt = hardwareMap.get(Servo.class, "rt_servo");
        lt = hardwareMap.get(Servo.class, "lt_servo");

        // Start at center for safety
        rt.setPosition(0.5);
        lt.setPosition(0.5);
    }

    @Override
    public void loop() {
        // Toggle Auto-Sweep
        if (gamepad1.a && !lastA) {
            autoSweep = !autoSweep;
        }
        lastA = gamepad1.a;

        if (autoSweep) {
            // Linear constant-speed sweep logic
            // We use 0.01 (10ms loop estimate) or better yet, a delta time
            double deltaTime = 0.02; // Roughly 50Hz loop speed

            ltPos += direction * sweepSpeed * deltaTime;

            // Check boundaries and flip direction
            if (ltPos >= MAX_POS) {
                ltPos = MAX_POS;
                direction = -1;
            } else if (ltPos <= MIN_POS) {
                ltPos = MIN_POS;
                direction = 1;
            }
        } else {
            // Manual adjustment via stick for testing specific spots
            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                ltPos -= gamepad1.left_stick_y * 0.01;
            }
        }

        // Final safety clamp
        ltPos = Range.clip(ltPos, MIN_POS, MAX_POS);

        rt.setPosition(ltPos);
        lt.setPosition(ltPos);

        telemetry.addData("Mode", autoSweep ? "SWEEPING" : "MANUAL");
        telemetry.addData("Position", "%.3f", ltPos);
        telemetry.addData("Target Range", "[%.2f - %.2f]", MIN_POS, MAX_POS);
        telemetry.update();
    }
}