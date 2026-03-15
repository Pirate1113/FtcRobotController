package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

@TeleOp(name="Ramp NOFTC")
public class RampNOFTC extends OpMode {

    Servo r_servo;
    // Range limits as requested
    private final double MIN_POS = 0;
    private final double MAX_POS = 1;
    private boolean lastA = false;

    @Override
    public void init() {

        r_servo = hardwareMap.get(Servo.class, RobotConstants.ramp);
        r_servo.setPosition(0);
    }

    @Override
    public void loop() {
        // Toggle Auto-Sweep
        if (gamepad1.a && !lastA) {
            r_servo.setPosition(1);
        } else if(gamepad1.a && lastA) {
            r_servo.setPosition(0);
        }
        lastA = gamepad1.a;


        telemetry.addData("Position", "%.3f", r_servo.getPosition());
        telemetry.update();
    }
}