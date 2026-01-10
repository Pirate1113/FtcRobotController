package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveToShootingAngle {

    private final DcMotorEx left, right;

    public SwerveToShootingAngle(HardwareMap hw) {
        left  = hw.get(DcMotorEx.class, "leftDrive");
        right = hw.get(DcMotorEx.class, "rightDrive");

        right.setDirection(DcMotorEx.Direction.REVERSE);
    }

    /** Turn robot using yaw offset */
    public void turnToTarget(double yawDeg) {
        double kP = 0.02; // we have to tune this
        double turnPower = yawDeg * kP;

        turnPower = Math.max(-0.5, Math.min(0.5, turnPower));

        left.setPower(turnPower);
        right.setPower(-turnPower);
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }
}