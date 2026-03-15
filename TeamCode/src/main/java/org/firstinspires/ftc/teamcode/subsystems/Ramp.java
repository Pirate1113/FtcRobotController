package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.common.RobotConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Ramp implements Subsystem {

    public static final Ramp INSTANCE = new Ramp();

    private ServoEx rampServo;
    private Ramp() {
    }

    @Override
    public void initialize() {
        rampServo = new ServoEx(RobotConstants.ramp, 0.02);
    }
    public Command front = new InstantCommand(() -> {
        rampServo.getServo().setPosition(1);
    });
    public Command back = new InstantCommand(() -> {
        rampServo.getServo().setPosition(0);
    });
    public double getRamp() {
        return rampServo.getPosition();
    }
    public void periodic() {

    }
}