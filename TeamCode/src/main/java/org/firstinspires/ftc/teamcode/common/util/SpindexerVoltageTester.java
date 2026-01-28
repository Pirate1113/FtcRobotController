package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "OffsetsTesting")
public class SpindexerVoltageTester extends LinearOpMode {

    private AnalogInput spindexerEncoder;
    private CRServo servoLeft;
    private CRServo servoRight;

    @Override
    public void runOpMode() {

        AnalogInput EncoderLeft = hardwareMap.get(AnalogInput.class, "analogLeft");
        AnalogInput EncoderRight = hardwareMap.get(AnalogInput.class, "analogRight");
        servoLeft = hardwareMap.get(CRServo.class, "spindexerleft");
        servoRight = hardwareMap.get(CRServo.class, "spindexerright");

        waitForStart();

        while (opModeIsActive()) {

            double voltageLeft = EncoderLeft.getVoltage();
            double voltageRight = EncoderRight.getVoltage();

            telemetry.addData("Spindexer Voltage Left", "%.3f", voltageLeft/3.3);
            telemetry.addData("Spindexer Voltage Right", "%.3f", voltageRight/3.3);
            telemetry.update();
        }
    }
}
