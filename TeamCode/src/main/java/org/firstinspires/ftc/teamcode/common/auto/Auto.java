package org.firstinspires.ftc.teamcode.common.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Auto")
public class Auto extends NextFTCOpMode {

    public Auto() {
        addComponents(new SubsystemComponent(SwerveDrivetrain.INSTANCE));
    }

    @Override
    public void onStartButtonPressed() {
        autoSequence().schedule();
    }

    // All coordinates are ABSOLUTE from the starting position (odo is reset on init).
    // X = strafe (positive = right), Y = forward (positive = forward), heading in radians.
    public Command autoSequence() {
        return new SequentialGroup(
                new PoseDriveCommand(0,  1, 0,            1.0), // forward 24 in
                new PoseDriveCommand(-2, 1, 0,            0.8), // strafe right 12 in
                new PoseDriveCommand(-2, 1, Math.PI / 2,  0.5), // rotate 90°
                new PoseDriveCommand(-2,  0, Math.PI / 2,  1.0)  // backward 24 in
        );
    }

    // Drives to an absolute field position (measured from odometry reset at init).
    public static class PoseDriveCommand extends Command {
        private final double targetX, targetY, targetHeading;
        private final double power;

        private static final double POS_TOLERANCE     = 0.5;  // inches
        private static final double HEADING_TOLERANCE = 0.02; // radians

        private boolean finished = false;

        public PoseDriveCommand(double targetX, double targetY, double targetHeading, double power) {
            this.targetX       = targetX;
            this.targetY       = targetY;
            this.targetHeading = targetHeading;
            this.power         = power;
        }

        @Override
        public void start() {
            finished = false;
            SwerveDrivetrain.INSTANCE.setTargetPose(targetX, targetY, targetHeading, power);
        }

        @Override
        public void update() {
            if (SwerveDrivetrain.INSTANCE.isAtTargetPose(POS_TOLERANCE, HEADING_TOLERANCE)) {
                SwerveDrivetrain.INSTANCE.stop();
                finished = true;
            }
        }

        @Override
        public boolean isDone() {
            return finished;
        }
    }
}
