package org.firstinspires.ftc.teamcode.common.swerce;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Swerve TeleOp")
public class SwerveTeleOp extends NextFTCOpMode {

    Telemetry dashboardTelemetry;
    FtcDashboard dashboard;


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
