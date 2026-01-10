package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class HoodAngle {

    private final Servo hood;
    private final DcMotorEx flywheel;

    public HoodAngle(HardwareMap hw) {
        hood = hw.get(Servo.class, "hoodServo");
        flywheel = hw.get(DcMotorEx.class, "shooter1");

        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /** Convert pitch to hood position */
    public void setHoodFromPitch(double pitchDeg) {
        double pos = 0.45 + pitchDeg * 1/360;  // need to tune 0.45 (initial hood angle), and the 0.002 prob should be tuned too
        pos = Math.max(0.0, Math.min(1.0, pos));
        hood.setPosition(pos);
    }

    /** Convert distance to RPM */
    public void setFlywheelFromDistance(double distance) {
        double rpm = 2200 + distance * 67; // wut wut wut 67
        flywheel.setVelocity(rpm);
    }

    public void stop() {
        flywheel.setPower(0);
    }
}
