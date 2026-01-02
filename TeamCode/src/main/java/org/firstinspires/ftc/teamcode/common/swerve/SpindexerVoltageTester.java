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

        spindexerEncoder = hardwareMap.get(AnalogInput.class, "spindexerEncoder");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        waitForStart();

        while (opModeIsActive()) {

            double voltage = spindexerEncoder.getVoltage();

            telemetry.addData("Spindexer Voltage", "%.3f", voltage);
            telemetry.addData("Servo Pos", servoLeft.getPosition());
            telemetry.update();
        }
    }
}
