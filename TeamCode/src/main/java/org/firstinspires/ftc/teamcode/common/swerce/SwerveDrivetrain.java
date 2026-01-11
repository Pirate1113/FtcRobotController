package org.firstinspires.ftc.teamcode.common.swerce;

import static java.lang.Math.hypot;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.common.swerce.SwerveModule;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class SwerveDrivetrain implements Subsystem {
    public static final SwerveDrivetrain INSTANCE = new SwerveDrivetrain();
    private SwerveDrivetrain() {}

    public GoBildaPinpointDriver odo;

    private double heading;

    Pose rawPose; //just use these as vectors
    Pose rotPose;

    public SwerveModule fR, bR, bL, fL;
    public SwerveModule[] swerveModules;

    public double[] wheelSpeeds = new double [4];
    public static final double MAX_SPEED = 6000; //TODO find
    public double[] angles = new double[4];
    public double[] cacheAngles = new double[4];

    private final double TW = 13.36;
    private  final double WB = 13.36;
    private final double R = hypot(TW/2, WB/2);

    private double startingAngle = 0;
    private static final double CACHE_TOLERANCE = 0.05;

    public static double[][] PIDKVal = {
            {0.6, 0, 0, 0.02}, // fR
            {0.6, 0, 0, 0.02}, // bR
            {0.6, 0, 0, 0.02}, // bL
            {0.6, 0, 0, 0.02}  // fL
    };

    @NonNull
    @Override
    public void initialize() {
        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-2.50688543, -6.70373543, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        fR = new SwerveModule("frontRight",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "fr_motor"),
                new com.seattlesolvers.solverslib.hardware.motors.CRServo(ActiveOpMode.hardwareMap(), "fr_rotation"),
                new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap(), "fr_absolute")
                        .zero(4.99),
                true, PIDKVal[0]);

        bR = new SwerveModule("backRight",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "br_motor"),
                new com.seattlesolvers.solverslib.hardware.motors.CRServo(ActiveOpMode.hardwareMap(), "br_rotation"),
                new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap(),"br_absolute")
                        .zero(6.14),
                false, PIDKVal[1]);

        bL = new SwerveModule("backLeft",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "bl_motor"),
                new com.seattlesolvers.solverslib.hardware.motors.CRServo(ActiveOpMode.hardwareMap(), "bl_rotation"),
                new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap(), "bl_absolute")
                        .zero(1.47),
                true, PIDKVal[2]);

        fL = new SwerveModule("frontLeft",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "fl_motor"),
                new com.seattlesolvers.solverslib.hardware.motors.CRServo(ActiveOpMode.hardwareMap(), "fl_rotation"),
                new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap(), "fl_absolute")
                        .zero(6.12),
                false, PIDKVal[3]);

        swerveModules = new SwerveModule[] {fR, bR, bL, fL};
    }

    @Override
    public void periodic() {
        for(SwerveModule m : swerveModules){
            m.read();
        }

        double rawLeftX = ActiveOpMode.gamepad1().left_stick_x,
                rawLeftY = -ActiveOpMode.gamepad1().left_stick_y,
                rawRightX = ActiveOpMode.gamepad1().right_stick_x,
                realRightX = rawRightX / Math.sqrt(2);

        heading = odo.getHeading(AngleUnit.RADIANS);

        rawPose = new Pose(rawLeftX, rawLeftY, realRightX);
        rotPose = rawPose.rotate(-heading, false);

        double fwd = rotPose.getY();
        double str = rotPose.getX();
        double rcw = rotPose.getHeading();

        double a = str - rcw * (WB / R);
        double b = str + rcw * (WB / R);
        double c = fwd - rcw * (TW / R);
        double d = fwd + rcw * (TW / R);

        wheelSpeeds = new double[]{
                Math.hypot(b, c), // frontRight
                Math.hypot(a, c), // backRight
                Math.hypot(a, d), // backLeft
                Math.hypot(b, d)  // frontLeft
        };

        angles = new double[]{
                Math.atan2(b, c), // frontRight
                Math.atan2(a, c), // backRight
                Math.atan2(a, d), // backLeft
                Math.atan2(b, d)  // frontLeft
        };

        double max = Math.max(Math.max(wheelSpeeds[0], wheelSpeeds[1]), Math.max(wheelSpeeds[2], wheelSpeeds[3]));
        if (max > 1.0) {
            for (int i = 0; i < 4; i++) wheelSpeeds[i] /= max;
        }

        boolean joystickIsIdle = (Math.abs(rawLeftX) <= CACHE_TOLERANCE && Math.abs(rawLeftY) <= CACHE_TOLERANCE && Math.abs(rawRightX) <= CACHE_TOLERANCE);


        for(int i = 0; i<swerveModules.length; i++){
            double targetAngle = angles[i];
            if (joystickIsIdle) {
                targetAngle = cacheAngles[i];
            } else {
                cacheAngles[i] = angles[i];
            }
            swerveModules[i].set(targetAngle);
            double speed = joystickIsIdle ? 0 : wheelSpeeds[i] * MAX_SPEED;
            swerveModules[i].write(speed);

            swerveModules[i].getTelemetry(ActiveOpMode.telemetry());
        }



    }
}