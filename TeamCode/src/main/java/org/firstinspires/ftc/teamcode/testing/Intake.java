package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
@Config
public class Intake extends OpMode {

    DcMotorEx intakeleft;
    DcMotorEx intakeright;

    double powerLeft = 0.0;
    double powerRight = 0.0;

    boolean previousDpadUp;
    boolean previousDpadDown;
    boolean previousX;
    boolean previousB;


    @Override
    public void init() {
        intakeleft = hardwareMap.get(DcMotorEx.class, "intakeleft");
        intakeright = hardwareMap.get(DcMotorEx.class, "intakeright");

        intakeleft.setDirection(DcMotor.Direction.REVERSE);
        intakeleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeright.setDirection(DcMotor.Direction.FORWARD);
        intakeright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void loop() {
        // Left intake control with dpad
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
        telemetry.addData("Left Intake Power", powerLeft);
        telemetry.addData("Right Intake Power", powerRight);
        telemetry.update();

    }
}