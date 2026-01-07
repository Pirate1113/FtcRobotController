//package org.firstinspires.ftc.teamcode.common.swerce;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
//
//
//import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
//import static org.firstinspires.ftc.teamcode.common.util.AngleFunxuns.wrapAngle0to2pi;
//
//
//import dev.nextftc.control.ControlSystem;
//import dev.nextftc.control.KineticState;
//import dev.nextftc.control.builder.ControlSystemBuilder;
//import dev.nextftc.control.feedback.AngleType;
//import dev.nextftc.control.feedback.PIDCoefficients;
//import dev.nextftc.control.feedback.PIDController;
//import dev.nextftc.hardware.impl.CRServoEx;
//import dev.nextftc.hardware.impl.FeedbackCRServoEx;
//import dev.nextftc.hardware.impl.MotorEx;
//
//@Configurable
//public class SwerveModule{
//
//    boolean WHEEL_FLIPPING = true;
//
//    private DcMotorEx drive;
//    private double lastMotorPower;
//
//    private FeedbackCRServoEx axon;
//    private double uncoffset;
//
//    public boolean wheelFlipped = false;
//    private double target = 0.0;
//    private KineticState vTarget;
//    private double position = 0.0;
//    private double velocity = 0.0;
//    private KineticState current;
//    private double error;
//    private double lastError;
//
//
//    private double power;
//
//
//    private PIDController rotController;
//    private PIDCoefficients PIDCoeffs;
//
//    private ControlSystem pid;
//
//    public SwerveModule(DcMotorEx m, CRServo s, AnalogInput e, double encoderOffset, double[] PIDVals){
//        this.drive = m;
//        this.
//        this.uncoffset = encoderOffset;
//
//        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        pid = new ControlSystemBuilder().angular(AngleType.RADIANS, feedback -> feedback.posPid(PIDCoeffs)).build();
//
//        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        vTarget = new KineticState(0,0);
//
//    }
//
//    public void read(){
//        position = axon.getCurrentPosition();
//    }
//
//    public void update(){
//        vTarget = new KineticState(target);
//        current = new KineticState(getModuleRotation(), velocity);
//
//        error = wrapAngle0to2pi(vTarget.minus(current).getPosition());
//
//        if(WHEEL_FLIPPING && Math.abs(error) > Math.PI/2) {
//            vTarget = new KineticState(wrapAngle0to2pi(vTarget.getPosition() - Math.PI));
//            wheelFlipped = true;
//        } else {
//            wheelFlipped = false;
//        }
//
//        error = wrapAngle0to2pi(vTarget.minus(current).getPosition());
//
//        pid.setGoal(new KineticState(target));
//        power = pid.calculate(new KineticState(getModuleRotation(), velocity));
//    }
//
//    public void write(){
//        axon.setPower(power);
//    }
//
//    public void setMotorPower(double power) {
//        if (wheelFlipped) power *= -1;
//        lastMotorPower = power;
//        drive.setPower(power);
//    }
//
//    public void setTarget(double t){
//        this.target = wrapAngle0to2pi(t);
//    }
//
//    public double getTargetRotation() {
//        return wrapAngle0to2pi(target - Math.PI);
//    }
//
//    public double getModuleRotation() {
//        return position - uncoffset;
//    }
//
//
//}
//package org.firstinspires.ftc.teamcode.common.swerce;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
//
//
//import com.bylazar.configurables.annotations.Configurable;
//import dev.nextftc.control.ControlSystem;
//import dev.nextftc.control.KineticState;
//import dev.nextftc.control.feedback.AngleType;
//import dev.nextftc.control.feedback.PIDCoefficients;
//import dev.nextftc.hardware.impl.CRServoEx;
//import dev.nextftc.hardware.impl.FeedbackCRServoEx;
//import dev.nextftc.hardware.impl.MotorEx;
//
//@Configurable
//public class SwerveModule{
//
//    final double WHEEL_RADIUS = 1;
//    final double GEAR_RATIO = 1;
//    final double TICKS_PER_REVOLUTION = 1;
//
//    double current;
//    double lastCurrent;
//    double target;
//    double period;
//    double velocity;
//
//    double power;
//
//    public static PIDCoefficients pidValues = new PIDCoefficients(0.5, 0, 0);
//    public ControlSystem pid = ControlSystem.builder()
//            .angular(AngleType.RADIANS, feedback -> feedback.posPid(pidValues))
//            .build();
//
//    private MotorEx drive;
//    private CRServoEx axon;
//    private FeedbackCRServoEx ax3on;
//    private AbsoluteAnalogEncoder enc;
//
//    public double xOffset;
//    public double yOffset;
//
//    public SwerveModule(MotorEx m, CRServoEx s, AnalogInput e, double eOffset, boolean reversed){
//        drive = m;
//        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        axon = s;
//        if(reversed) ax3on.getServo().setDirection( );
//
//        enc = new AbsoluteAnalogEncoder(e, 3.3).zero(eOffset).setInverted(reversed);
//
//
//
//        axon.setPower(1);
//        axon.setPower(0);
//    }
//
//    public double read(){
//        current = enc.getCurrentPosition();
//    }
//
//    public void rotateTo(double target){
//        pid.setGoal(new KineticState(target));
//
//        if (Math.abs(period) > 1E-6) {
//            velocity = (current - lastCurrent) / period;
//        } else {
//            velocity = 0;
//        }
//
//        KineticState sCurrent = new KineticState(current, velocity);
//
//        power = pid.calculate(sCurrent);
//
//    }
//
//    public void write(double power){
//        axon.setPower(power);
//        drive.setPower(power);
//    }
//
//
//    public void setTarget(double t){
//        this.target = wrapAngle0to2pi(t);
//    }
//
//    public double getTargetRotation() {
//        return wrapAngle0to2pi(target - Math.PI);
//    }
//
//    public double getModuleRotation() {
//        return position - uncoffset;
//    }
//
//
//}
package org.firstinspires.ftc.teamcode.common.swerce;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.core.units.Angle;


@Config
public class SwerveModule{

    String name;

    final double WHEEL_RADIUS = 1;
    final double GEAR_RATIO = 1;
    final double TICKS_PER_REVOLUTION = 1;
    final boolean WHEEL_FLIPPING = true;

    double wheelVel; //in ticks/sec

    double current;
    double lastCurrent;
    double target;
    double lastTarget;
    double error;
    double period;
    double velocity;

    double power;
    boolean wheelFlipped;

    PIDCoefficients pidValues;
    double K_STATIC;
    public ControlSystem pid;

    private DcMotorEx drive;
    private CRServoEx axon;
    private AbsoluteAnalogEncoder enc;


    /** Construcotr is:
     @param n name
     @param m motor
     @param s servo
     @param e AnalogInput
     @param eOffset  offset
     @param reversed is it reveresed?
     @param PIDK an array of the PIDK values
     */
    public SwerveModule(String n, DcMotorEx m, CRServo s, AnalogInput e, double eOffset, boolean reversed, double[] PIDK){
        this.name = n;

        this.drive = m;
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.axon = new CRServoEx(s, 0.03);

        this.enc = new AbsoluteAnalogEncoder(e, 3.3).zero(eOffset);

        this.pidValues = new PIDCoefficients(PIDK[0], PIDK[1], PIDK[2]);
        this.K_STATIC = PIDK[3];

        this.pid = ControlSystem.builder()
                .angular(AngleType.RADIANS, feedback -> feedback.posPid(pidValues))
                .build();

        if(reversed) {
            this.axon.getServo().setDirection(DcMotorSimple.Direction.REVERSE);
            this.enc.setInverted(true);
        }

        axon.setPower(1);
        axon.setPower(0);
    }

    public void read(){
        Angle.Companion.wrapAnglePiToPi(current = enc.getCurrentPosition());
    } // this comes out [-pi, pi)

    public void rotateTo(double tar){
        this.target = Angle.Companion.wrapAnglePiToPi(tar);

        this.error = Angle.Companion.wrapAnglePiToPi(target-current);

        if (WHEEL_FLIPPING && Math.abs(error) > Math.PI / 2) {
            target = Angle.Companion.wrapAnglePiToPi(target - Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        pid.setGoal(new KineticState(target));

        //calculating velocity for use in D term
        if (Math.abs(period) > 1E-6) {
            velocity = (current - lastCurrent) / period;
        } else {
            velocity = 0;
        }

        KineticState sCurrent = new KineticState(current, velocity);

        power = pid.calculate(sCurrent) + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power);
    }

    public void write(){
        axon.setPower(power);
        if (wheelFlipped) wheelVel *= -1;
        drive.setVelocity(wheelVel);
    }

    public void getTelemetry(Telemetry telemetry){
        telemetry.addData("", name);
        telemetry.addData("Target", target);
        telemetry.addData("Current", current);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.addData("Rot vel", velocity);
        telemetry.addData("Wheel Flipped", wheelFlipped);
        telemetry.addData("Drive power", wheelVel);
    }

}
