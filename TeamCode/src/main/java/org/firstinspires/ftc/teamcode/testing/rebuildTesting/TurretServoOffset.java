package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

import dev.nextftc.hardware.impl.ServoEx;

@TeleOp(name="Axon Turret Calibration")
public class TurretServoOffset extends LinearOpMode {

    private double leftPos = 0.5;
    private double rightPos = 0.5;
    private boolean tuningLeft = true;

    private boolean lastUp = false;
    private boolean lastDown = false;

    @Override
    public void runOpMode() {
        // Keeping your 0.02 tolerance for the wrapper
        ServoEx leftTurretEx = new ServoEx(RobotConstants.leftTurret, 0.02);
        ServoEx rightTurretEx = new ServoEx(RobotConstants.rightTurret, 0.02);

        // Bypass the wrapper by using the internal RobotCore servos directly
        // This ensures the 0.001 increments actually MOVE the motor immediately
        Servo leftInternal = leftTurretEx.getServo();
        Servo rightInternal = rightTurretEx.getServo();

        telemetry.addLine("=== AXON 1:1 CALIBRATION ===");
        telemetry.addLine("A -> Tune Left | B -> Tune Right");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) tuningLeft = true;
            if (gamepad1.b) tuningLeft = false;

            // Increment logic (0.001 is now viable because we bypass the cache)
            if (gamepad1.dpad_up && !lastUp) {
                if (tuningLeft) leftPos += 0.03; else rightPos += 0.03;
            }
            if (gamepad1.dpad_down && !lastDown) {
                if (tuningLeft) leftPos -= 0.03; else rightPos -= 0.03;
            }

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            leftPos = Range.clip(leftPos, 0.0, 1.0);
            rightPos = Range.clip(rightPos, 0.0, 1.0);

            // Using internal servos to ensure every 0.001 change is sent to the Hub
            if (tuningLeft) {
                ((PwmControl) rightInternal).setPwmDisable();
                ((PwmControl) leftInternal).setPwmEnable();
                leftInternal.setPosition(leftPos);
            } else {
                ((PwmControl) leftInternal).setPwmDisable();
                ((PwmControl) rightInternal).setPwmEnable();
                rightInternal.setPosition(rightPos);
            }

            double currentDeg = (tuningLeft ? leftPos : rightPos) * RobotConstants.turretServoRange;
            double centerDiff = currentDeg - 177.5;

            telemetry.addData("Tuning", tuningLeft ? "LEFT" : "RIGHT");
            telemetry.addData("Current Degrees", "%.2f°", currentDeg);
            telemetry.addData("Sync Offset (Delta)", "%.2f°", centerDiff);
            telemetry.addLine("\nEvery D-pad click now moves the servo exactly 0.001 in pos");
            telemetry.update();
        }
    }
}