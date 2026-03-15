package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

@Config
@TeleOp(name="Axon Turret Calibration")
public class TurretServoOffset extends OpMode {

    public static double turretZero = 0.5;
    public static double turretServoRange = 360.0;
    public static double ENCODER_OFFSET_DEG = 187.75;

    ServoImplEx rt, lt;
    AnalogInput ai;
    AbsoluteAnalogEncoder enc;

    public double offset = 0;
    private double currentBaseAngle = 0;

    private boolean isLoose = false;
    private boolean lastB = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rt = (ServoImplEx) hardwareMap.get(Servo.class, "rt_servo");
        lt = (ServoImplEx) hardwareMap.get(Servo.class, "lt_servo");
        ai = hardwareMap.get(AnalogInput.class, "t_absolute");

        enc = new AbsoluteAnalogEncoder(ai, 3.3).zero(Math.toRadians(ENCODER_OFFSET_DEG));

        toAngle(0);
    }

    public void toAngle(double angle) {
        // 1:1 Math
        double totalAngle = angle + offset;

        // Calculate the raw servo command (could be 1.5, -0.2, etc.)
        double rawTarget = turretZero + (totalAngle / turretServoRange);

        // MODULO WRAP: This keeps the hardware command between 0 and 1
        // but keeps the movement direction consistent.
        double wrappedTarget = rawTarget % 1.0;
        if (wrappedTarget < 0) wrappedTarget += 1.0;

        // Apply to hardware
        rt.setPosition(wrappedTarget);
        lt.setPosition(wrappedTarget);

        telemetry.addData("Wrapped Servo Pos", "%.3f", wrappedTarget);
    }

    @Override
    public void loop() {
        if (gamepad1.b && !lastB) isLoose = !isLoose;
        lastB = gamepad1.b;

        if (isLoose) {
            rt.setPwmDisable();
            lt.setPwmDisable();
        } else {
            if (!rt.isPwmEnabled()) {
                rt.setPwmEnable();
                lt.setPwmEnable();
            }

            // D-pad now increments the offset infinitely.
            // Up will ALWAYS increase the angle, Down will ALWAYS decrease it.
            if (gamepad1.dpad_up) offset += 2.0;
            if (gamepad1.dpad_down) offset -= 2.0;

            if (gamepad1.x) offset = 0;

            toAngle(currentBaseAngle);
        }

        telemetry.addData("Encoder Deg", "%.2f°", Math.toDegrees(enc.getCurrentPosition()));
        telemetry.addData("Total Target Angle", "%.2f°", (currentBaseAngle + offset));
        telemetry.update();
    }
}