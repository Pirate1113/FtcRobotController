package org.firstinspires.ftc.teamcode.common.swerce;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDriver;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class SwerveDrivetrain implements Subsystem {

    public static final SwerveDrivetrain INSTANCE = new SwerveDrivetrain();
    private SwerveDrivetrain() {}

    public GoBildaPinpointDriver odo;

    public SwerveModule frontLeft, frontRight, backRight, backLeft;
    public SwerveModule[] modules;

    private final double TW = 13.36;
    private final double WB = 13.36;
    private final double R = hypot(TW / 2, WB / 2);

    public static double frontLeftOffset = 2;
    public static double frontRightOffset = 0;
    public static double backLeftOffset = 0;
    public static double backRightOffset = -0.055;

    private final double minPow = 0.1;

    @Override
    public void initialize() {

        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-2.5069, -6.7037, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        AbsoluteAnalogEncoder flEnc =
                new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap().get(AnalogInput.class, "fl_rotation"), 3.3)
                        .zero(frontLeftOffset).setInverted(true);

        AbsoluteAnalogEncoder blEnc =
                new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap().get(AnalogInput.class, "bl_absolute"), 3.3)
                        .zero(backLeftOffset).setInverted(true);

        AbsoluteAnalogEncoder frEnc =
                new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap().get(AnalogInput.class, "fr_absolute"), 3.3)
                        .zero(frontRightOffset).setInverted(true);

        AbsoluteAnalogEncoder brEnc =
                new AbsoluteAnalogEncoder(ActiveOpMode.hardwareMap().get(AnalogInput.class, "br_absolute"), 3.3)
                        .zero(backRightOffset).setInverted(true);

        frontLeft = new SwerveModule(
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "fl_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "fl_rotation"),
                flEnc
        );

        backLeft = new SwerveModule(
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "bl_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "bl_rotation"),
                blEnc
        );

        frontRight = new SwerveModule(
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "fr_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "fr_rotation"),
                frEnc
        );

        backRight = new SwerveModule(
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "br_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "br_rotation"),
                brEnc
        );

        modules = new SwerveModule[]{frontLeft, frontRight, backRight, backLeft};

        for (SwerveModule m : modules) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void periodic() {

        for (SwerveModule m : modules) {
            m.read();
            m.update();
        }

        double lx = ActiveOpMode.gamepad1().left_stick_x;
        double ly = -ActiveOpMode.gamepad1().left_stick_y;
        double rx = ActiveOpMode.gamepad1().right_stick_x;

        double heading = odo.getHeading(AngleUnit.RADIANS);

        Pose input = new Pose(ly, lx, rx);
        Pose field = input.rotate(-heading, false);

        double x = field.getX();
        double y = field.getY();
        double rot = field.getHeading();

        double a = x - rot * (WB / R);
        double b = x + rot * (WB / R);
        double c = y - rot * (TW / R);
        double d = y + rot * (TW / R);

        double[] ws = {
                hypot(b, c),
                hypot(b, d),
                hypot(a, d),
                hypot(a, c)
        };

        double[] wa = {
                atan2(c, b),
                atan2(d, b),
                atan2(d, a),
                atan2(c, a)
        };

        for (int i = 0; i < 4; i++) {
            modules[i].setTargetRotation(wa[i]);
            modules[i].setMotorPower(ws[i] + minPow * Math.signum(ws[i]));
        }
    }
}