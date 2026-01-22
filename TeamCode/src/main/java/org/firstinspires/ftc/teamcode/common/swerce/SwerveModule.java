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

    public void write(double wheelVel){
        axon.setPower(power);
        if (wheelFlipped) wheelVel *= -1;
        drive.setVelocity( Math.cos(error) * wheelVel);
        //cosine comp to not move when wheel still tunring
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