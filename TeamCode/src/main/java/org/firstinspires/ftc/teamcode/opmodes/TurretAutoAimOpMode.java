package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.testing.LimelightAngle;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "Turret Auto Aim", group = "Main")
public class TurretAutoAimOpMode extends NextFTCOpMode {

    private LimelightAngle limelight;
    private final double Kp = 0.5; // Proportional gain for aiming

    public TurretAutoAimOpMode() {
        addComponents(new SubsystemComponent(Turret.INSTANCE));
    }

    @Override
    public void onInit() {
        // Parameters: HardwareMap, name, shooterHeight, tagHeight
        limelight = new LimelightAngle(hardwareMap, "limelight", 14.0, 29.5);
    }

    @Override
    public void onUpdate() {
        if (limelight.hasTarget()) {
            double tx = limelight.getYaw();
            
            // Adjust the turret base angle based on Limelight horizontal offset (tx)
            double currentTarget = Turret.INSTANCE.getTargetAngle();
            Turret.INSTANCE.setBaseAngle(currentTarget + (tx * Kp));
            
            telemetry.addData("Status", "Target Locked");
            telemetry.addData("Limelight tx", tx);
        } else {
            telemetry.addData("Status", "No Target Found");
        }

        telemetry.addData("Turret Target Angle", Turret.INSTANCE.getTargetAngle());
        telemetry.addData("Turret Encoder Angle", Turret.INSTANCE.getEncoderAngle());
    }
}
