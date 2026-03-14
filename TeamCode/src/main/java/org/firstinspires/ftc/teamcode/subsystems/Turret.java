package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.common.RobotConstants;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Turret implements Subsystem {

    public static final Turret INSTANCE = new Turret();

    private ServoEx leftServo;
    private ServoEx rightServo;

    private final double range;

    // Track the desired angle state
    private double targetAngle = 0.0;
    private boolean isDead = false;

    private Turret() {
        this.range = RobotConstants.turretServoRange;
    }

    @Override
    public void initialize() {
        this.leftServo = new ServoEx(RobotConstants.leftTurret, 0.02);
        this.rightServo = new ServoEx(RobotConstants.rightTurret, 0.02);

        // Start centered
        this.targetAngle = 0.0;
        this.isDead = false;
    }

    /**
     * Call this from your OpMode or other subsystems to update the target.
     * Angle: -177.5 to 177.5
     */
    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
        this.isDead = false; // Re-enable if a new angle is requested
    }

    public void setDead() {
        this.isDead = true;
    }

    @Override
    public void periodic() {
        // If we want the turret to be limp, disable PWM and skip movement
        if (isDead) {
            ((PwmControl) leftServo.getServo()).setPwmDisable();
            ((PwmControl) rightServo.getServo()).setPwmDisable();
            return;
        }

        // 1. Calculate the 0.0 - 1.0 position
        // Midpoint is 177.5 (0.5 position)
        double targetPos = (targetAngle + 177.5) / range;

        // 2. Clip to protect physical 355-degree limits
        targetPos = Range.clip(targetPos, 0.0, 1.0);

        // 3. Ensure PWM is enabled
        ((PwmControl) leftServo.getServo()).setPwmEnable();
        ((PwmControl) rightServo.getServo()).setPwmEnable();

        // 4. Update hardware
        leftServo.setPosition(targetPos);
        rightServo.setPosition(targetPos);
    }
}