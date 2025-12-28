package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
@Config
public class intake extends OpMode {

    DcMotorEx intakeleft;

    double power = 0.0;

    boolean previousDpadUp;
    boolean previousDpadDown;


    @Override
    public void init() {
        intakeleft = hardwareMap.get(DcMotorEx.class, "intakeleft");

        intakeleft.setDirection(DcMotor.Direction.REVERSE);
        intakeleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void loop() {
        if (gamepad1.dpad_up && !previousDpadUp) {
            power += 0.1;
        } else if (gamepad1.dpad_down && !previousDpadDown) {
            power -= 0.1;
        }

        intakeleft.setPower(power);

    }
}