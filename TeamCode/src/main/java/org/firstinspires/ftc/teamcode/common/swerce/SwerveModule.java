package org.firstinspires.ftc.teamcode.common.swerce;

import org.firstinspires.ftc.teamcode.common.util.AngleFunxuns;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;

import com.qualcomm.robotcore.

@Configurable
public class SwerveModule{

    boolean MOTOR_FLIPPING = true;

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
        position = encoder.getCurrentPosition();
    }

    public void update(){
        double sTarget = getTargetRotation(), current = getModuleRotation();

        double error = AngleFunxuns.wrapAngle0to2pi(sTarget - current);


        if(MOTOR_FLIPPING && Math.abs(error) > Math.PI/2) {
            sTarget = AngleFunxuns.wrapAngle0to2pi(sTarget-Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        error = AngleFunxuns.wrapAngle0to2pi(sTarget - current);

        double power = Range.clip(rotController.calculate())

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
