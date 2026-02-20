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
        SwerveDrivetrain.INSTANCE.setAuto();
        autoSequence().schedule();
    }


    public Command autoSequence() {
        return new SequentialGroup(
                new PoseDriveCommand(0, 24, 0, 1.0),    // move forward 24 in
                new PoseDriveCommand(12, 0, 0, 0.8),    // strafe right 12 in
                new PoseDriveCommand(0, 0, Math.PI/2, 0.5), // rotate 90 degrees clockwise
                new PoseDriveCommand(0, -24, 0, 1.0)    // move backward 24 in
        ); // patterns random
    }

    /**
     * relative to starting position
     */
    public static class PoseDriveCommand extends Command {
        private final double dx, dy, dHeading;
        private final double power;
        private static final double TOLERANCE = 0.5; // ininin inches
        private static final double HEADING_TOL = 0.02; // ininin radians
        private boolean finished = false;

        public PoseDriveCommand(double dx, double dy, double dHeading, double power) {
            this.dx = dx;
            this.dy = dy;
            this.dHeading = dHeading;
            this.power = power;
        }

        @Override
        public void start() {
            SwerveDrivetrain.INSTANCE.setTargetPose(dx, dy, dHeading, power);
        }

        @Override
        public void update() {
            if (SwerveDrivetrain.INSTANCE.isAtTargetPose(TOLERANCE, HEADING_TOL)) {
                SwerveDrivetrain.INSTANCE.stop(); // stopping when reaches targetpose
                finished = true;
            }
        }

        @Override
        public boolean isDone() {
            return finished;
        }
    }
}