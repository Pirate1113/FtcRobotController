package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;

import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.testing.LimelightAngle;

@Autonomous(name = "Forward + Limelight Aim")
public class Auto extends NextFTCOpMode {

    private LimelightAngle limelight;

    public Auto () {
        addComponents(
                new SubsystemComponent(SwerveDrivetrain.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onInit() {

        limelight = new LimelightAngle(
                hardwareMap,
                "limelight",
                10,
                60
        );

        limelight.pipelineSwitch(0);
    }

    @Override
    public void onStartButtonPressed() {

        try {

            // move forward a short distance
            SwerveDrivetrain.INSTANCE.autoDrive(0.4);
            Thread.sleep(1200);
            SwerveDrivetrain.INSTANCE.stop();

            // aim using limelight?? this needs work?
            if (limelight.hasTarget()) {

                double yaw = limelight.getYaw();

                // turn turret to angle
                Turret.INSTANCE.setBaseAngle(yaw);

            }

        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}