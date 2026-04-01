package org.firstinspires.ftc.teamcode.swerveDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;

import dev.nextftc.control.ControlSystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Disabled
public class SubsystemTest implements Subsystem {

    public static final SubsystemTest INSTANCE = new SubsystemTest();
    private SubsystemTest() { }

    public SwerveModule testModule;

    @Override
    public void initialize(){
        //testModule = new SwerveModule(ActiveOpMode.hardwareMap(), "fl_motor", false,
        //        "fl_rotation", true, "fl_absolute", 0, true, 3.1865);
    }

}
