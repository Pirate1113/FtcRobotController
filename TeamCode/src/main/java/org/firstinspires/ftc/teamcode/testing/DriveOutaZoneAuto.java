package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;

@Autonomous(name = "drive out of the zone")
public class DriveOutaZoneAuto extends LinearOpMode {
    private SwerveDrivetrain drivetrain = SwerveDrivetrain.INSTANCE;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drivetrain subsystem
        drivetrain.initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        timer.reset();

        // Drive forward for 1.0 seconds
        while (opModeIsActive() && timer.seconds() < 0.5) {
            drivetrain.autoDrive(0.4); // Tune this power as needed

            telemetry.addData("Auto Status", "Driving Forward");
            telemetry.addData("Time", timer.seconds());
            telemetry.update();
        }

        // Must stop the motors after the loop or they will keep spinning!
        drivetrain.stop();

        telemetry.addData("Status", "Complete");
        telemetry.update();
    }
}

