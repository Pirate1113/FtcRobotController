package org.firstinspires.ftc.teamcode.common.swerve;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

public class SpindexerVoltageTester extends LinearOpMode {

    private AnalogInput spindexerEncoder;
    private Servo servoLeft;
    private Servo servoRight;

    @Override
    public void runOpMode() {

        AnalogInput EncoderLeft = hardwareMap.get(AnalogInput.class, "spindexerEncoderLeft");
        AnalogInput EncoderRight = hardwareMap.get(AnalogInput.class, "spindexerEncoderRight");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        waitForStart();

        while (opModeIsActive()) {

            double voltageLeft = EncoderLeft.getVoltage();
            double voltageRight = EncoderRight.getVoltage();

            telemetry.addData("Spindexer Voltage Left", "%.3f", voltageLeft);
            telemetry.addData("Spindexer Voltage Right", "%.3f", voltageRight);
            telemetry.addData("Servo Pos", servoLeft.getPosition());
            telemetry.update();
        }
    }
}
