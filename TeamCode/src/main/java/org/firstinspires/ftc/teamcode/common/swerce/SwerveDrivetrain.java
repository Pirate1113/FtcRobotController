package org.firstinspires.ftc.teamcode.common.swerce;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.common.swerce.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.ftc.ActiveOpMode;

public class SwerveDrivetrain implements Subsystem {
    public static final SwerveDrivetrain INSTANCE = new SwerveDrivetrain();
    private SwerveDrivetrain(){}
    public GoBildaPinpointDriver odo;

    private static final boolean USE_WHEEL_FEEDFORWARD = true;
    public SwerveModule frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public SwerveModule[] modules;

    private final double TW = 13.36;
    private  final double WB = 13.36;
    private final double R = hypot(TW/2, WB/2);
    public static double frontLeftOffset = 6.12, frontRightOffset = 4.22, backLeftOffset = 1.47, backRightOffset = 6.01;

    public static boolean maintainHeading = false;

    double[] ws = new double[4];
    double[] wa = new double[4];
    double max = 0.0;

    public final double minPow = 0.1;
    public static double imuOffset = 0.0;

    private boolean locked = false;


    @Override
    public void periodic() {

        for (SwerveModule module : modules) module.read();


        double rawLeftX = ActiveOpMode.gamepad1().left_stick_x,
                rawLeftY = -ActiveOpMode.gamepad1().left_stick_y,
                rawRightX = -ActiveOpMode.gamepad1().right_stick_x,
                realRightX = rawRightX / Math.sqrt(2);

        double heading = odo.getHeading(AngleUnit.RADIANS);

        Pose rawPose = new Pose(rawLeftY, rawLeftX, realRightX);
        Pose rotPose = rawPose.rotate(-heading, false);

        double x = rotPose.getX(), y = rotPose.getY(), head = rotPose.getHeading();

        double a = x - head * (WB / R),
                b = x + head * (WB / R),
                c = y - head * (TW / R),
                d = y + head * (TW / R);

        if (locked) {
            ws = new double[]{0, 0, 0, 0};
            wa = new double[]{Math.PI / 4, -Math.PI / 4, Math.PI / 4, -Math.PI / 4};
        } else {
            ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
            if (!maintainHeading) wa = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};
        }
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            m.setMotorPower(Math.abs(ws[i]) + ((USE_WHEEL_FEEDFORWARD) ? minPow * Math.signum(ws[i]) : 0));
            m.setTargetRotation(Angle.Companion.wrapAngle0To2Pi(wa[i]));
            m.update();
        }

        String telemetry = frontLeftModule.getTelemetry("leftFrontModule") + "\n" +
                backLeftModule.getTelemetry("leftRearModule") + "\n" +
                frontRightModule.getTelemetry("rightFrontModule") + "\n" +
                backRightModule.getTelemetry("rightRearModule") + "\n";

        ActiveOpMode.telemetry().addData("telemetry",0);
    }

    @Override
    public void initialize() {

        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-2.50688543, -6.70373543, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        AbsoluteAnalogEncoder flenc = new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap().get(AnalogInput.class, "fl_absolute"), 3.3).zero(frontLeftOffset).setInverted(true);
        AbsoluteAnalogEncoder blenc = new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap().get(AnalogInput.class, "bl_absolute"), 3.3).zero(backLeftOffset).setInverted(true);
        AbsoluteAnalogEncoder frenc = new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap().get(AnalogInput.class, "fr_absolute"), 3.3).zero(frontRightOffset).setInverted(true);
        AbsoluteAnalogEncoder brenc = new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap().get(AnalogInput.class, "br_absolute"), 3.3).zero(backRightOffset).setInverted(true);
        frontLeftModule = new SwerveModule(ActiveOpMode.hardwareMap().get(DcMotorEx.class, "fl_motor"), ActiveOpMode.hardwareMap().get(CRServo.class, "fl_rotation"), flenc);
        backLeftModule = new SwerveModule(ActiveOpMode.hardwareMap().get(DcMotorEx.class, "bl_motor"), ActiveOpMode.hardwareMap().get(CRServo.class, " bl_rotation"), blenc);
        frontRightModule = new SwerveModule(ActiveOpMode.hardwareMap().get(DcMotorEx.class, "fr_motor"), ActiveOpMode.hardwareMap().get(CRServo.class, " fr_rotation"), frenc);
        backRightModule = new SwerveModule(ActiveOpMode.hardwareMap().get(DcMotorEx.class, "br_motor"), ActiveOpMode.hardwareMap().get(CRServo.class, " br_rotation"), brenc);

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}