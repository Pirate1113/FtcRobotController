package org.firstinspires.ftc.teamcode.common.swerce;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.driving.Drivetrain;
import dev.nextftc.hardware.driving.FieldCentric;

public class SwerveDrivetrain implements Subsystem {

    public static double[][] PIDKVal = {
            {0.6, 0 ,0}, //fR
            {0.6, 0 ,0}, //bR
            {0.6, 0 ,0}, //bL
            {0.6, 0 ,0}  //fL
    };


}
