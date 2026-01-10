package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Palm implements Subsystem {
    public static final Palm INSTANCE = new Palm();

    private Palm() {
    }

    private ServoEx palm;


    public void initialize() {
        palm = new ServoEx("feedServo");
    }

    public void periodic() {
    }

    public final Command off = new SetPosition(palm, 0).requires(this).named("PalmOff");
    public final Command on = new SetPosition(palm, 0.1).requires(this).named("PalmOn");

}