package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Palm implements Subsystem {
    public static final Palm INSTANCE = new Palm();
    private double pos = 0;
    private Palm() {
    }

    private ServoEx palm;


    public void initialize() {
        palm = new ServoEx("feedServo");
    }

    public void periodic() {
    }

    public final Command off = new InstantCommand(() -> {
        pos = 0;
        palm.setPosition(0);
    }).requires(this);
    public final Command on = new InstantCommand(() -> {
        pos = 0.3;
        palm.setPosition(0.3);
    }).requires(this);
    public double getPos() {
        return pos;
    }
}