package org.firstinspires.ftc.teamcode.common.swerce;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDriver;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class SwerveDrivetrain implements Subsystem {
    public static final SwerveDrivetrain INSTANCE = new SwerveDrivetrain();
    private SwerveDrivetrain() {}

    private enum ControlMode { TELEOP, AUTO }
    private ControlMode controlMode = ControlMode.TELEOP;

    Telemetry dashboardTelemetry;
    FtcDashboard dashboard;

    public GoBildaPinpointDriver odo;

    private double heading;

    Pose rawPose;
    Pose rotPose;

    public SwerveModule fR, bR, bL, fL;
    public SwerveModule[] swerveModules;

    public double[] wheelSpeeds = new double[4];
    public static final double MAX_SPEED = 6000;
    public double[] angles = new double[4];
    public double[] cacheAngles = new double[4];

    private final double TW = 13.36;
    private final double WB = 13.36;
    private final double R = hypot(TW, WB);

    private static final double CACHE_TOLERANCE = 0.05;

    public static double[][] PIDKVal = {
            {0.6, 0, 0.02, 0},
            {0.6, 0, 0.02, 0},
            {0.6, 0, 0.02, 0},
            {0.6, 0, 0.02, 0}
    };

    private double targetX, targetY, targetHeading;
    private double targetPower;

    @NonNull
    @Override
    public void initialize() {
        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-2.50688543, -6.70373543, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        fR = new SwerveModule("frontRight",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "fr_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "fr_rotation"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "fr_absolute"),
                4.22, false, true, PIDKVal[0]);

        bR = new SwerveModule("backRight",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "br_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "br_rotation"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "br_absolute"),
                6.01, false, true, PIDKVal[1]);

        bL = new SwerveModule("backLeft",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "bl_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "bl_rotation"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "bl_absolute"),
                1.47, false, true, PIDKVal[2]);

        fL = new SwerveModule("frontLeft",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "fl_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "fl_rotation"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "fl_absolute"),
                6.12, false, true, PIDKVal[3]);

        swerveModules = new SwerveModule[]{fL, fR, bR, bL};
    }

    @Override
    public void periodic() {

        for (SwerveModule m : swerveModules) {
            m.read();
        }

        odo.update();

        if (controlMode == ControlMode.AUTO) {
            moveToTargetPose();
            return;
        }

        double rawLeftX = -ActiveOpMode.gamepad1().left_stick_x;
        double rawLeftY = -ActiveOpMode.gamepad1().left_stick_y;
        double rawRightX = -ActiveOpMode.gamepad1().right_stick_x;
        double realRightX = rawRightX / Math.sqrt(2);

        heading = odo.getHeading(AngleUnit.RADIANS);



        rawPose = new Pose(odo.getPosX(DistanceUnit.INCH), odo.getPosY(DistanceUnit.INCH), odo.getHeading(AngleUnit.RADIANS));
        rotPose = rawPose.rotate(heading, false);

        double x = rotPose.getX();
        double y = rotPose.getY();
        double head = rotPose.getHeading();


        double a = x - head * (WB / R);
        double b = x + head * (WB / R);
        double c = y - head * (TW / R);
        double d = y + head * (TW / R);

        wheelSpeeds = new double[]{
                hypot(b, c),
                hypot(b, d),
                hypot(a, d),
                hypot(a, c)
        };

        angles = new double[]{
                atan2(b, c),
                atan2(b, d),
                atan2(a, d),
                atan2(a, c)
        };

        double max = Math.max(
                Math.max(wheelSpeeds[0], wheelSpeeds[1]),
                Math.max(wheelSpeeds[2], wheelSpeeds[3])
        );

        if (max > 1.0) {
            for (int i = 0; i < 4; i++) wheelSpeeds[i] /= max;
        }

        boolean joystickIsIdle =
                Math.abs(rawLeftX) <= CACHE_TOLERANCE &&
                        Math.abs(rawLeftY) <= CACHE_TOLERANCE &&
                        Math.abs(rawRightX) <= CACHE_TOLERANCE;

        for (int i = 0; i < swerveModules.length; i++) {
            if (!joystickIsIdle) {
                cacheAngles[i] = angles[i];
            }
            swerveModules[i].rotateTo(cacheAngles[i]);
            swerveModules[i].write(wheelSpeeds[i] * MAX_SPEED);
            swerveModules[i].getTelemetry(ActiveOpMode.telemetry());
        }
    }

    public void setTargetPose(double dx, double dy, double dHeading, double power) {
        targetX = dx;
        targetY = dy;
        targetHeading = dHeading;
        targetPower = power;
        controlMode = ControlMode.AUTO;
    }

    public boolean isAtTargetPose(double posTolerance, double headingTolerance) {
        double currentX = odo.getPosX(DistanceUnit.INCH);
        double currentY = odo.getPosY(DistanceUnit.INCH);
        double currentHeading = odo.getHeading(AngleUnit.RADIANS);

        double xError = Math.abs(targetX - currentX);
        double yError = Math.abs(targetY - currentY);
        double headingError = Math.abs(targetHeading - currentHeading);

        return xError <= posTolerance &&
                yError <= posTolerance &&
                headingError <= headingTolerance;
    }

    private void moveToTargetPose() {
        double currentX = odo.getPosX(DistanceUnit.INCH);
        double currentY = odo.getPosY(DistanceUnit.INCH);
        double currentHeading = odo.getHeading(AngleUnit.RADIANS);

        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double dh = targetHeading - currentHeading;

        Pose errorPose = new Pose(dx, dy, dh)
                .rotate(currentHeading, false);

        double x = errorPose.getX();
        double y = errorPose.getY();
        double head = errorPose.getHeading();

        double a = x - head * (WB / R);
        double b = x + head * (WB / R);
        double c = y - head * (TW / R);
        double d = y + head * (TW / R);

        wheelSpeeds = new double[]{
                hypot(b, c),
                hypot(b, d),
                hypot(a, d),
                hypot(a, c)
        };

        angles = new double[]{
                atan2(b, c),
                atan2(b, d),
                atan2(a, d),
                atan2(a, c)
        };

        double max = Math.max(
                Math.max(wheelSpeeds[0], wheelSpeeds[1]),
                Math.max(wheelSpeeds[2], wheelSpeeds[3])
        );

        if (max > 1.0) {
            for (int i = 0; i < 4; i++) wheelSpeeds[i] /= max;
        }

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].rotateTo(angles[i]);
            swerveModules[i].write(wheelSpeeds[i] * targetPower * MAX_SPEED);
        }
    }

    public void stop() {
        for (SwerveModule m : swerveModules) {
            m.write(0);
        }
        controlMode = ControlMode.TELEOP;
    }
}