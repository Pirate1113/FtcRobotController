package org.firstinspires.ftc.teamcode.common.swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;

@Configurable
public class SwerveModule{

    private DcMotorEx drive;
    private CRServo axon;
    private AbsoluteAnalogEncoder encoder;

    public boolean wheelFlipped = false;
    private double target = 0.0;
    private double position = 0.0;

    private double error;

    private PIDController rotController;
    private PIDCoefficients PIDCoeffs;



    public SwerveModule(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e, double encoderOffset, double[] PIDKVal){
        this.drive = m;
        this.axon = s;
        this.encoder = e;

        MotorConfigurationType motorConfigurationType = m.getMotorType().clone();
        this.drive.setMotorType(motorConfigurationType);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.encoder.zero(encoderOffset);

        PIDCoeffs = new PIDCoefficients(PIDKVal[0], PIDKVal[1], PIDKVal[2]);
        rotController = new PIDController(PIDCoeffs);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void read(){

    }

    public void update(){
        
    }

    public void setTarget(double t){
        this.target = normalizeRadians(t);
    }

    public double getTargetRotation() {
        return normalizeRadians(target - Math.PI);
    }

    public double getModuleRotation() {
        return normalizeRadians(position - Math.PI);
    }




}
