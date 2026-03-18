package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    private Turret() {
    }

    private Pose goalPose;
    private ServoEx leftServo;
    private ServoEx rightServo;
    private AbsoluteAnalogEncoder encoder;

    public static double turretZero = 0.5;
    public static double turretServoRange = 355;
    public static double ENCODER_OFFSET_DEG = 187.75;

    private double offset = 0;
    private double baseAngle = 0;
    private boolean isLoose = false;
    private boolean autoAimEnabled = false;

    @Override
    public void initialize() {
        leftServo = new ServoEx(RobotConstants.leftTurret);
        rightServo = new ServoEx(RobotConstants.rightTurret);
        
        AnalogInput ai = ActiveOpMode.hardwareMap().get(AnalogInput.class, "t_absolute");
        encoder = new AbsoluteAnalogEncoder(ai, 3.3).zero(Math.toRadians(ENCODER_OFFSET_DEG));

        if (RobotConstants.alliance == RobotConstants.Alliance.RED) {
            goalPose = RobotConstants.redGoal;
        } else {
            goalPose = RobotConstants.blueGoal;
        }
    }

    @Override
    public void periodic() {
        ServoImplEx leftRaw = (ServoImplEx) leftServo.getServo();
        ServoImplEx rightRaw = (ServoImplEx) rightServo.getServo();

        if (isLoose) {
            leftRaw.setPwmDisable();
            rightRaw.setPwmDisable();
        } else {
            if (!leftRaw.isPwmEnabled()) {
                leftRaw.setPwmEnable();
                rightRaw.setPwmEnable();
            }

            double angleToTarget = baseAngle + offset;
            double rawTarget = turretZero + (angleToTarget / turretServoRange);

            double wrappedTarget = rawTarget % 1.0;
            if (wrappedTarget < 0) wrappedTarget += 1.0;

            leftServo.setPosition(wrappedTarget);
            rightServo.setPosition(wrappedTarget);
        }
    }

    public final Command track = new LambdaCommand("Turret Track Goal")
            .setUpdate(() -> {
                turretTrack(SwerveDrivetrain.INSTANCE.getPose());
            })
            .setIsDone(() -> false)
            .requires(this);

    public void turretTrack(Pose robotPose) {
        if (goalPose == null) return;

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        double fieldAngle = Math.atan2(dy, dx);

        // Flip sign because turret + is clockwise
        double turretAngleRad = robotPose.getHeading() - fieldAngle;

        double turretAngleDeg = Math.toDegrees(turretAngleRad);

        // Normalize to [-180, 180]
        turretAngleDeg = ((turretAngleDeg + 180) % 360) - 180;

        setBaseAngle(turretAngleDeg);
    }

    public void setGoalPose(Pose goalPose) {
        this.goalPose = goalPose;
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

    public boolean isLoose() {
        return isLoose;
    }

    public double getEncoderAngle() {
        return Math.toDegrees(encoder.getCurrentPosition());
    }

    public double getTargetAngle() {
        return baseAngle + offset;
    }
}
