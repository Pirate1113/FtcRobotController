package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

@Config
@TeleOp(name="Axon Turret Calibration")
public class TurretServoOffset extends OpMode {

    public static double SERVO_MIN = 0.05;
    public static double SERVO_MAX = 0.95;

    // The physical limits of your turret's travel in Degrees
    public static double ENC_MIN_DEG = 10.0;
    public static double ENC_MAX_DEG = 350.0;

    // YOUR OFFSET
    public static double ENCODER_OFFSET_DEG = 187.75;
    public static boolean INVERT_ENCODER = false;

    ServoImplEx rt, lt;
    AnalogInput ai;
    AbsoluteAnalogEncoder enc;

    private double currentTargetServo = 0.5;
    private boolean isLoose = false;
    private boolean lastB = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rt = (ServoImplEx) hardwareMap.get(Servo.class, "rt_servo");
        lt = (ServoImplEx) hardwareMap.get(Servo.class, "lt_servo");
        ai = hardwareMap.get(AnalogInput.class, "t_absolute");

        // Zeroing with your 187.75 offset
        enc = new AbsoluteAnalogEncoder(ai, 3.3).zero(Math.toRadians(ENCODER_OFFSET_DEG));
        enc.setInverted(INVERT_ENCODER);

        rt.setPosition(0.5);
        lt.setPosition(0.5);
    }

    @Override
    public void loop() {
        if (gamepad1.b && !lastB) isLoose = !isLoose;
        lastB = gamepad1.b;

        // Force a fresh read
        double currentDeg = Math.toDegrees(enc.getCurrentPosition());

        if (isLoose) {
            rt.setPwmDisable();
            lt.setPwmDisable();
        } else {
            if (!rt.isPwmEnabled()) {
                rt.setPwmEnable();
                lt.setPwmEnable();
            }

            // D-pad movement
            if (gamepad1.dpad_up) currentTargetServo += 0.005;
            if (gamepad1.dpad_down) currentTargetServo -= 0.005;

            // WHIP LOGIC
            // If we hit the Max degree limit, whip the servo back to the Min position
            if (currentDeg >= ENC_MAX_DEG) {
                currentTargetServo = SERVO_MIN;
            }
            // If we hit the Min degree limit, whip the servo forward to the Max position
            else if (currentDeg <= ENC_MIN_DEG) {
                currentTargetServo = SERVO_MAX;
            }

            currentTargetServo = Range.clip(currentTargetServo, 0, 1);

            rt.setPosition(currentTargetServo);
            lt.setPosition(currentTargetServo);
        }

        telemetry.addData("Encoder Deg", "%.2f°", currentDeg);
        telemetry.addData("Servo Command", "%.3f", currentTargetServo);
        telemetry.addData("Raw Voltage", "%.3fV", ai.getVoltage());
        telemetry.update();
    }
}