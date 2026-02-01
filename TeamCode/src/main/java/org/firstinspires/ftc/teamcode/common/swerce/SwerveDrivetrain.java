package org.firstinspires.ftc.teamcode.common.swerce;

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
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDriver;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class SwerveDrivetrain implements Subsystem {
    public static final SwerveDrivetrain INSTANCE = new SwerveDrivetrain();
    private SwerveDrivetrain() {}

    Telemetry dashboardTelemetry;
    FtcDashboard dashboard;

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
    private final double R = hypot(TW, WB);


    private double startingAngle = 0;
    private static final double CACHE_TOLERANCE = 0.05;

    public static double[][] PIDKVal = {
            {0.6, 0 ,0.02, 0}, // fL
            {0.6, 0 ,0.02, 0}, // fR
            {0.6, 0 ,0.02, 0}, // bR
            {0.6, 0 ,0.02, 0}  // bL
    };

//    public static PIDCoefficients HEADING_PID_COEFFS = new PIDCoefficients(1.8, 0, 0.1);
//
//    ControlSystem headingPID = ControlSystem.builder().angular(AngleType.RADIANS,
//            feedback -> {feedback.posPid(HEADING_PID_COEFFS);}
//            )
//            .build();

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

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

//        for (SwerveModule m : swerveModules) {
//            m.rotateTo(startingAngle);
//        }
    }


    @Override
    public void periodic() {
        for(SwerveModule m : swerveModules){
            m.read();
        }

        double rawLeftX = -ActiveOpMode.gamepad1().left_stick_x,
                rawLeftY = -ActiveOpMode.gamepad1().left_stick_y,
                rawRightX = -ActiveOpMode.gamepad1().right_stick_x,
                realRightX = rawRightX / Math.sqrt(2);

        heading = odo.getHeading(AngleUnit.RADIANS);
        odo.update();

        rawPose = new Pose(rawLeftX, rawLeftY, realRightX);
        rotPose = rawPose.rotate(heading, false);

        double x = rotPose.getX(), y = rotPose.getY(), head = rotPose.getHeading();

        double a = x - head * (WB / R),
                b = x + head * (WB / R),
                c = y - head * (TW / R),
                d = y + head * (TW / R);

        wheelSpeeds = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
        angles = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};

        double max = Math.max(Math.max(wheelSpeeds[0], wheelSpeeds[1]), Math.max(wheelSpeeds[2], wheelSpeeds[3]));
        if (max > 1.0) {
            for (int i = 0; i < 4; i++) wheelSpeeds[i] /= max;
        }

        boolean joystickIsIdle = (Math.abs(rawLeftX) <= CACHE_TOLERANCE && Math.abs(rawLeftY) <= CACHE_TOLERANCE && Math.abs(rawRightX) <= CACHE_TOLERANCE);


        for(int i = 0; i<swerveModules.length; i++){
            if (!joystickIsIdle){
                cacheAngles[i] = angles[i];
            }
            swerveModules[i].rotateTo(cacheAngles[i]);
            swerveModules[i].write(wheelSpeeds[i]*MAX_SPEED);
            swerveModules[i].getTelemetry(ActiveOpMode.telemetry());
        }



    }

    public void autoDrive(double power) {
        double currentHeading = odo.getHeading(AngleUnit.RADIANS);
        for (SwerveModule m : swerveModules){
            m.read();
        }
//        double headingVeloicty = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
//
//        headingPID.setGoal(new KineticState(0, 0));
//
//        double headingPower = headingPID.calculate(new KineticState(currentHeading, headingVeloicty));

        Pose drivePose = new Pose(power, 0, 0);

        double x = drivePose.getX(), y = drivePose.getY(), head = drivePose.getHeading();

        double a = x - head * (WB / R),
                b = x + head * (WB / R),
                c = y - head * (TW / R),
                d = y + head * (TW / R);

        wheelSpeeds = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
        angles = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};

        double max = Math.max(Math.max(wheelSpeeds[0], wheelSpeeds[1]), Math.max(wheelSpeeds[2], wheelSpeeds[3]));
        if (max > 1.0) {
            for (int i = 0; i < 4; i++) wheelSpeeds[i] /= max;
        }

        for(int i = 0; i<swerveModules.length; i++){
            swerveModules[i].rotateTo(angles[i]);
            swerveModules[i].write(wheelSpeeds[i]*MAX_SPEED);
            swerveModules[i].getTelemetry(dashboardTelemetry);
        }

    }

    public void stop() {
        for (SwerveModule m : swerveModules) {
            m.write(0);
        }
    }
}