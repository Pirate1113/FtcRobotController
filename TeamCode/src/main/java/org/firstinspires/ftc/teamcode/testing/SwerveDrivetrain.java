package org.firstinspires.ftc.teamcode.testing;

import static java.lang.Math.hypot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDriver;

import java.util.Set;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class SwerveDrivetrain implements Subsystem {
    public static final SwerveDrivetrain INSTANCE = new SwerveDrivetrain();
    private SwerveDrivetrain() {}

    private final double VOLTAGE_COMP=3.3; //TODO: check later
    private static final double CACHE_TOL = 0.05;

    public GoBildaPinpointDriver odo;
    double oldTime = 0;

    public SwerveModule fR, bR, bL, fL;
    public SwerveModule[] swerveModules;

    private static final double
            FR_OFFSET = 1,
            BR_OFFSET = 1,
            BL_OFFSET = 1,
            FL_OFFSET = 1;

    private double wheelSpeeds[] = new double [4];
    private double targetAngles[] = new double [4];
    private double cachedaAnlges[] = new double [4];
    private double startingAngle = 0;

    private final double TW = 8.86;
    private final double WB = 8.86   ;
    private final double R = hypot(TW/2, WB/2);

    public static double[][] PIDKVal = {
            {0.6, 0 ,0}, //fR
            {0.6, 0 ,0}, //bR
            {0.6, 0 ,0}, //bL
            {0.6, 0 ,0}  //fL
    };

    @NonNull
    @Override
    public Command getDefaultCommand() {
        return Subsystem.super.getDefaultCommand();
    }

    @NonNull
    @Override
    public Set<Subsystem> getSubsystems() {
        return Subsystem.super.getSubsystems();
    }

    @Override
    public void initialize() {
        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-2.50688543, -6.70373543, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        fR = new SwerveModule("frontRight",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "frontRight"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "sfrontRight"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "efrontRight"),
                FR_OFFSET,
                false,
                PIDKVal[0]
        );
        bR = new SwerveModule("backRight",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "backRight"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "sbackRight"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "ebackRight"),
                BR_OFFSET,
                true,
                PIDKVal[1]
        );
        bL = new SwerveModule("backLeft",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "backLeft"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "sbackLeft"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "ebackLeft"),
                FR_OFFSET,
                false,
                PIDKVal[2]
        );
        fL = new SwerveModule("frontLeft",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "frontLeft"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "sfrontLeft"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "efrontLeft"),
                FR_OFFSET,
                true,
                PIDKVal[3]
        );
        swerveModules = new SwerveModule[] {fR, bR, bL, fL};

        for (SwerveModule m : swerveModules){
            m.rotateTo(startingAngle);
        }
    }

    @Override
    public void periodic() {
        Subsystem.super.periodic();
    }


}
