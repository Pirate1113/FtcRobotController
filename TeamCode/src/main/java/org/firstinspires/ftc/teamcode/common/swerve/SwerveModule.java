package org.firstinspires.ftc.teamcode.common.swerve;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import static org.firstinspires.ftc.teamcode.common.util.AngleFunxuns.wrapAngle0to2pi;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.builder.ControlSystemBuilder;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;

@Configurable
public class SwerveModule{

    boolean MOTOR_FLIPPING = true;

    private DcMotorEx drive;
    private double lastMotorPower;

    private FeedbackCRServoEx axon;
    private double uncoffset;

    public boolean wheelFlipped = false;
    private double target = 0.0;
    private double position = 0.0;
    private double velocity = 0.0;

    private double power;


    private PIDController rotController;
    private PIDCoefficients PIDCoeffs;

    private ControlSystem pid;

    public SwerveModule(DcMotorEx m, CRServo s, AnalogInput e, double encoderOffset, double[] PIDKVal){
        this.drive = m;
        this.axon = new FeedbackCRServoEx(0.01, () -> e, () -> s);
        this.uncoffset = encoderOffset;

        MotorConfigurationType motorConfigurationType = m.getMotorType().clone();
        this.drive.setMotorType(motorConfigurationType);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDCoeffs = new PIDCoefficients(PIDKVal[0], PIDKVal[1], PIDKVal[2]);
        rotController = new PIDController(PIDCoeffs);

        pid = new ControlSystemBuilder().angular(AngleType.RADIANS, feedback -> feedback.posPid(PIDCoeffs)).build();

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void read(){
        position = axon.getCurrentPosition();
        velocity = axon.getVelocity();
    }

    public void update(){
        double sTarget = getTargetRotation(), current = getModuleRotation();

        double error = wrapAngle0to2pi(sTarget - current);


        if(MOTOR_FLIPPING && Math.abs(error) > Math.PI/2) {
            sTarget = wrapAngle0to2pi(sTarget-Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        error = wrapAngle0to2pi(sTarget - current);

        pid.setGoal(new KineticState(target));
        power = pid.calculate(new KineticState(getModuleRotation(), velocity));

    }

    public void write(){
        axon.setPower(power);
    }

    public void setMotorPower(double power) {
        if (wheelFlipped) power *= -1;
        lastMotorPower = power;
        drive.setPower(power);
    }

    public void setTarget(double t){
        this.target = wrapAngle0to2pi(t);
    }

    public double getTargetRotation() {
        return wrapAngle0to2pi(target - Math.PI);
    }

    public double getModuleRotation() {
        return position - uncoffset;
    }




}
