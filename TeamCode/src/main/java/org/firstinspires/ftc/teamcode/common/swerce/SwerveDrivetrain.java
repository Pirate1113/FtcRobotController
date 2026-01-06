package org.firstinspires.ftc.teamcode.common.swerce;

import androidx.annotation.NonNull;

import java.util.Set;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.driving.FieldCentric;

public class SwerveDrivetrain implements Subsystem {
    public static final SwerveDrivetrain INSTANCE = new SwerveDrivetrain();
    private SwerveDrivetrain() {}

    public static double[][] PIDKVal = {
            {0.6, 0 ,0}, //fR
            {0.6, 0 ,0}, //bR
            {0.6, 0 ,0}, //bL
            {0.6, 0 ,0}  //fL
    };

    @NonNull
    @Override
    public Command getDefaultCommand() {
        return Subsystem.super.getDefaultCommand();
    }

    @NonNull
    @Override
    public Set<Subsystem> getSubsystems() {
        return Subsystem.super.getSubsystems();
    }

    @Override
    public void initialize() {
        Subsystem.super.initialize();
    }

    @Override
    public void periodic() {
        Subsystem.super.periodic();
    }

}
