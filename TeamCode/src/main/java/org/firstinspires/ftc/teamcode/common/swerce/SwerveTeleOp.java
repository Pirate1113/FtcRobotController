package org.firstinspires.ftc.teamcode.common.swerce;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Swerve TeleOp")
public class SwerveTeleOp extends NextFTCOpMode {

    public SwerveTeleOp() {
        addComponents(
                new SubsystemComponent(SwerveDrivetrain.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {}


    @Override
    public void onUpdate(){
        telemetry.update();
    }

}
