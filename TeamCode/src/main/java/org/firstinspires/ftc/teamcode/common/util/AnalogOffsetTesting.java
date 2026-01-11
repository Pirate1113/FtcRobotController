package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import java.util.List;

@TeleOp (name = "offsetTEstingReal")
public class AnalogOffsetTesting extends OpMode {
    private static final double TAU = 2*Math.PI;

    private static final double SWERVE_MAX_VOLTS = 3.3;
    private static final double SPINDEXER_MAX_VOLTS = 3.3;

//    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

    CRServoImplEx fRServo;

    AnalogInput fR;
    AnalogInput bR;
    AnalogInput bL;
    AnalogInput fL;

    AnalogInput SpindexerL;
    AnalogInput SpindexerR;

    @Override
    public void init() {
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }

        fRServo = hardwareMap.get(CRServoImplEx.class, "fr_rotation");

        fR = hardwareMap.get(AnalogInput.class, "fr_absolute");
        bR = hardwareMap.get(AnalogInput.class, "br_absolute");
        bL = hardwareMap.get(AnalogInput.class, "bl_absolute");
        fL = hardwareMap.get(AnalogInput.class, "fl_absolute");

//        SpindexerL = hardwareMap.get(AnalogInput.class, "servoLeft");
//        SpindexerR = hardwareMap.get(AnalogInput.class, "servoRight");

//        fRServo.setPower(1);
//        fRServo.setPower(0);
        fRServo.setPwmEnable();
    }

    @Override
    public void loop() {
//        for (LynxModule hub : allHubs) {
//            hub.clearBulkCache();
//        }

        telemetry.addData("fR", doTheThing(fR.getVoltage()));
        telemetry.addData("bR", doTheThing(bR.getVoltage()));
        telemetry.addData("bL", doTheThing(bL.getVoltage()));
        telemetry.addData("fL", doTheThing(fL.getVoltage()));

//        telemetry.addData("Spindexer Right", doTheThing(SpindexerR.getVoltage()));
//        telemetry.addData("Spindexer Left", doTheThing(SpindexerL.getVoltage()));

        telemetry.update();
    }

    public double doTheThing(double voltzIn) {
        return (voltzIn/3.3) * TAU;
        /*
        3.3 FOR NOW; MIGHT HAVE TO CHANGE LATER BECAUSE SWERVE AND
        SPINDEXER PLUGGED INTO DIFF POWER INJECTORS
        *
        */

    }

}
