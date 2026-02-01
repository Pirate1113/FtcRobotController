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
import dev.nextftc.hardware.impl.FeedbackCRServoEx;


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
    private FeedbackCRServoEx ax3on;
    private AbsoluteAnalogEncoder enc;
    private double lastTimeStamp;

    /** Construcotr is:
     @param n name
     @param m motor
     @param s servo
     @param e AnalogInput
     @param eOffset  offset
     @param servoReverse is servo reveresed?
     @param analogReverse is the enc reversed>
     @param PIDK an array of the PIDK values
     */
    public SwerveModule(String n, DcMotorEx m, CRServo s, AnalogInput e, double eOffset, boolean servoReverse, boolean analogReverse, double[] PIDK){
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

        if(servoReverse) this.axon.getServo().setDirection(CRServo.Direction.REVERSE);
        if(analogReverse) this.enc.setInverted(true);

        axon.setPower(1);
        axon.setPower(0);

        ax3on = new FeedbackCRServoEx(0.05, ()->e, ()->s);
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
        error = Angle.Companion.wrapAnglePiToPi(target - current);

        pid.setGoal(new KineticState(target));

//        double currentTimeStamp = (double) System.nanoTime() / 1E9;
//        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
//        period = currentTimeStamp - lastTimeStamp;
//        lastTimeStamp = currentTimeStamp;
//
//        //calculating velocity for use in D term
//        if (Math.abs(period) > 1E-6) {
//            velocity = (current - lastCurrent) / period;
//        } else {
//            velocity = 0;
//        }

        velocity = ax3on.getVelocity();

        KineticState sCurrent = new KineticState(current, velocity);

        power = pid.calculate(sCurrent) + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power);
    }

    public void write(double wV){
        this.wheelVel = wV;
        axon.setPower(power);
        if (wheelFlipped) wheelVel *= -1;
        drive.setVelocity(Math.pow(Math.cos(Math.abs(error)), 8)*wheelVel);
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