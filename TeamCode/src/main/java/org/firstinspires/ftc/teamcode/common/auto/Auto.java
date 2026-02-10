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

    /**
     * example auto sequence
     */
    public Command autoSequence() {
        return new SequentialGroup(
                new PoseDriveCommand(0, 24, 0, 1.0),    // Move forward 24 in
                new PoseDriveCommand(12, 0, 0, 0.8),    // Strafe right 12 in
                new PoseDriveCommand(0, 0, Math.PI/2, 0.5), // Rotate 90 deg CW
                new PoseDriveCommand(0, -24, 0, 1.0)    // Move backward 24 in
        );
    }

    /**
     * relative to starting position
     */
    public static class PoseDriveCommand extends Command {
        private final double dx, dy, dHeading;
        private final double power;
        private static final double TOLERANCE = 0.5; // in inches
        private static final double HEADING_TOL = 0.02; // in radians
        private boolean finished = false;

        public PoseDriveCommand(double dx, double dy, double dHeading, double power) {
            this.dx = dx;
            this.dy = dy;
            this.dHeading = dHeading;
            this.power = power;
        }

        @Override
        public void start() {
            // called once at command start
            SwerveDrivetrain.INSTANCE.setTargetPose(dx, dy, dHeading, power);
        }

        @Override
        public void update() {
            // ts called repeatedly
            if (SwerveDrivetrain.INSTANCE.isAtTargetPose(TOLERANCE, HEADING_TOL)) {
                SwerveDrivetrain.INSTANCE.stop(); // stops when target reached
                finished = true;
            }
        }

        @Override
        public boolean isDone() {
            return finished;
        }

        }
    }
