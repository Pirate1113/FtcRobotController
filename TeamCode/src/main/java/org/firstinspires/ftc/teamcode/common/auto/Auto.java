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
     * example auto
     */

    // supposed to be square
    public Command autoSequence() {
        double sideLength = 24; // in inches
        double power = 0.8;

        return new SequentialGroup(
                new PoseDriveCommand(0, sideLength, 0, power),
                new PoseDriveCommand(sideLength, 0, 0, power),
                new PoseDriveCommand(0, -sideLength, 0, power),
                new PoseDriveCommand(-sideLength, 0, 0, power)
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
