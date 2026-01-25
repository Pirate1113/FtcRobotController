package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
@Config
public class JacobShi extends OpMode {

    DcMotorEx shooter1;
    DcMotorEx shooter2;

    double power = 0.0;

    boolean previousDpadUp;
    boolean previousDpadDown;

    double RPM;

    FtcDashboard dashboard;

    @Override
    public void init() {
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");

        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter2.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();
    }

    public void loop() {
//        if (gamepad1.dpad_up && !previousDpadUp) {
//            power += 0.1;
//        } else if (gamepad1.dpad_down && !previousDpadDown) {
//            power -= 0.1;
//        }

        shooter1.setPower(-1.0);
        shooter2.setPower(-1.0);

        RPM = shooter1.getVelocity()*60/28;

        telemetry.addData("RPM", RPM);
        telemetry.update();
        
    }
}
