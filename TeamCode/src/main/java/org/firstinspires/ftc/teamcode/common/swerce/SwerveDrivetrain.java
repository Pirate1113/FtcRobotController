package org.firstinspires.ftc.teamcode.common.swerce;

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

    public GoBildaPinpointDriver odo;

    public SwerveModule fR, bR, bL, fL;
    public SwerveModule[] swerveModules;

    public double[] wheelSpeeds = new double [4];
    public double[] angles = new double[4];
    public double[] cacheAngles = new double[4];
    

    private double startingAngle = 0;

    public static double[][] PIDKVal = {
            {0.6, 0 ,0}, // fR
            {0.6, 0 ,0}, // bR
            {0.6, 0 ,0}, // bL
            {0.6, 0 ,0}  // fL
    };

    @NonNull
    @Override
    public Command getDefaultCommand() { return Subsystem.super.getDefaultCommand(); }

    @NonNull
    @Override
    public Set<Subsystem> getSubsystems() { return Subsystem.super.getSubsystems(); }

    @Override
    public void initialize() {
        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-2.50688543, -6.70373543, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        fR = new SwerveModule("frontRight",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "frontright_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "frontright_rotation"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "frontright_encoder"),
                1, false, PIDKVal[0]);

        bR = new SwerveModule("backRight",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "backright_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "backright_rotation"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "backright_encoder"),
                1, true, PIDKVal[1]);

        bL = new SwerveModule("backLeft",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "backleft_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "backleft_rotation"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "backleft_encoder"),
                1, false, PIDKVal[2]);

        fL = new SwerveModule("frontLeft",
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "frontleft_motor"),
                ActiveOpMode.hardwareMap().get(CRServo.class, "frontleft_rotation"),
                ActiveOpMode.hardwareMap().get(AnalogInput.class, "frontleft_encoder"),
                1, true, PIDKVal[3]);

        swerveModules = new SwerveModule[] {fR, bR, bL, fL};

        for (SwerveModule m : swerveModules) {
            m.rotateTo(startingAngle);
        }
    }

    @Override
    public void periodic() {


    }
}