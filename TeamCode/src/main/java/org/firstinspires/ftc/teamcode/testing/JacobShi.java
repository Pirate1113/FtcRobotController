package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
@Config
public class JacobShi extends OpMode {

    DcMotorEx shooter;

    double power = 0.0;

    boolean previousDpadUp;
    boolean previousDpadDown;

    double RPM;

    FtcDashboard dashboard;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "drive");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();
    }

    public void loop() {
        if (gamepad1.dpad_up && !previousDpadUp) {
            power += 0.1;
        } else if (gamepad1.dpad_down && !previousDpadDown) {
            power -= 0.1;
        }

        shooter.setPower(power);

        RPM = shooter.getVelocity()*60/28;

        telemetry.addData("RPM", RPM);
        telemetry.update();

        telemetry = dashboard.getTelemetry();
    }
}
