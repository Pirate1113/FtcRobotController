package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    private Turret() {
    }

    private ServoEx leftServo;
    private ServoEx rightServo;
    private AbsoluteAnalogEncoder encoder;

    private double offset = 0;
    private double baseAngle = 0;
    private boolean isLoose = false;

    public static double turretZero = 0.5;
    public static double turretServoRange = 360.0;
    public static double ENCODER_OFFSET_DEG = 187.75;

    @Override
    public void initialize() {
        leftServo = new ServoEx(RobotConstants.leftTurret);
        rightServo = new ServoEx(RobotConstants.rightTurret);
        
        // Using raw hardware for encoder as AbsoluteAnalogEncoder takes AnalogInput
        AnalogInput ai = ActiveOpMode.hardwareMap().get(AnalogInput.class, "t_absolute");
        encoder = new AbsoluteAnalogEncoder(ai, 3.3).zero(Math.toRadians(ENCODER_OFFSET_DEG));
    }

    @Override
    public void periodic() {
        if (isLoose) {
            leftServo.getServo().getController().pwmDisable();
            rightServo.getServo().getController().pwmDisable();
        } else {
            // Re-enable if it was disabled
            // Note: ServoEx doesn't directly expose pwmEnable easily, so using raw servo
            // This assumes the controller is the same for both
            leftServo.getServo().getController().pwmEnable();
            
            double totalAngle = baseAngle + offset;
            double rawTarget = turretZero + (totalAngle / turretServoRange);
            double wrappedTarget = rawTarget % 1.0;
            if (wrappedTarget < 0) wrappedTarget += 1.0;

            leftServo.setPosition(wrappedTarget);
            rightServo.setPosition(wrappedTarget);
        }
    }

    public void setBaseAngle(double angle) {
        this.baseAngle = angle;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void addOffset(double delta) {
        this.offset += delta;
    }

    public void setLoose(boolean loose) {
        this.isLoose = loose;
    }

    public double getEncoderAngle() {
        return Math.toDegrees(encoder.getCurrentPosition());
    }

    public double getTargetAngle() {
        return baseAngle + offset;
    }
}
