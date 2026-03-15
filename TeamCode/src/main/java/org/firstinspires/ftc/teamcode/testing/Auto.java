package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;

import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.testing.LimelightAngle;

@Autonomous(name = "Forward + Limelight Aim")
public class Auto extends NextFTCOpMode {

    private LimelightAngle limelight;
    private ElapsedTime timer = new ElapsedTime();

    private enum State {
        DRIVE_FORWARD,
        AIM,
        DONE
    }

    private State state = State.DRIVE_FORWARD;

    public Auto() {
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
        timer.reset();
        state = State.DRIVE_FORWARD;
    }

    @Override
    public void onUpdate() {

        switch (state) {

            case DRIVE_FORWARD:

                SwerveDrivetrain.INSTANCE.autoDrive(0.4);

                if (timer.seconds() > 1.2) {
                    SwerveDrivetrain.INSTANCE.stop();
                    timer.reset();
                    state = State.AIM;
                }

                break;

            case AIM:

                if (limelight.hasTarget()) {

                    double yaw = limelight.getYaw();

                    // turret subsystem handles offsets internally
                    Turret.INSTANCE.setBaseAngle(yaw);
                }

                // aim for ~1 second
                if (timer.seconds() > 1.0) {
                    state = State.DONE;
                }

                break;

            case DONE:
                // robot finished
                break;
        }
    }
}