package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.common.RobotConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Gate implements Subsystem {

    public static final Gate INSTANCE = new Gate();

    private ServoEx gateServo;
    private Gate() {
    }

    @Override
    public void initialize() {
        gateServo = new ServoEx(RobotConstants.gate, 0.02);
    }
    public Command open = new InstantCommand(() -> {
        gateServo.getServo().setPosition(1);
    });
    public Command close = new InstantCommand(() -> {
        gateServo.getServo().setPosition(0);
    });
    public Command flick = new InstantCommand(() -> {
        gateServo.getServo().setPosition(2);
    });
    public double getGate() {
        return gateServo.getPosition();
    }
    public void periodic() {

    }
}