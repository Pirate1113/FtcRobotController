package org.firstinspires.ftc.teamcode.common.swerce;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SwerveModule {
    String name;

    final double WHEEL_RADIUS = 1;
    final double GEAR_RATIO = 1;
    final double TICKS_PER_REVOLUTION = 1;
    final boolean WHEEL_FLIPPING = true;

    double wheelVel; //in ticks/sec
    private static final double MAX_VEL = 6000;

    double current;
    double target;
    Double lastTarget;
    double error;
    double power;

    boolean wheelFlipped;

    PIDFController pidf;

    double K_STATIC;

    private DcMotorEx drive;
    private CRServo axon;
    private AbsoluteAnalogEncoder enc;

    /** Construcotr is:
     @param n name
     @param m motor
     @param s servo
     @param e AnalogInput
     @param reversed is it reveresed?
     @param PIDK an array of the PIDK values
     */
    public SwerveModule(String n, DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e, boolean reversed, double[] PIDK){
        this.name = n;

        this.drive = m;
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.axon = s;

        this.enc = e;

        this.pidf = new PIDFController(PIDK[0], PIDK[1], PIDK[2], 0);
        this.K_STATIC = PIDK[3];

        if(reversed) {
            this.axon.setInverted(true);
            this.enc.setReversed(true);
        } else {
            this.axon.setInverted(false);
            this.enc.setReversed(false);
        }

    }

    public SwerveModule setPIDF(PIDFCoefficients coefficients) {
        this.pidf = new PIDFController(coefficients);
        return this;
    }

    public SwerveModule setAbsoluteEncoder(AbsoluteAnalogEncoder encoder) {
        this.enc = encoder;
        return this;
    }

    public AbsoluteAnalogEncoder getAbsoluteEncoder() {
        return this.enc;
    }

    public void read() {
        current = enc.getCurrentPosition();
    }

    public void set(double tar) {
        // Safety check
        if (pidf == null) {
            throw new IllegalStateException(
                    "OptimizedPositionalControl requires a PIDF"
            );
        }

        AngleUnit unit = enc.getAngleUnit();

        // Normalize target to [0, 2π)
        double target = MathUtils.normalizeAngle(tar, true, unit);

        // Current position (already absolute, but normalize defensively)
        double current = MathUtils.normalizeAngle(
                enc.getCurrentPosition(), true, unit
        );

        // Shortest-path error in [-π, π)
        double error = MathUtils.normalizeAngle(
                target - current, false, unit
        );

        // Reset PID if target changed
        if (lastTarget == null ||
                Math.abs(MathUtils.normalizeAngle(
                        target - lastTarget, false, unit
                )) > 1e-6) {

            pidf.reset();
            lastTarget = target;
        }

        // PID drives error to 0
        double power = pidf.calculate(error, 0);

        // Direct write with calculated power

//        power += Range.clip((Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power), -1,1);
        axon.set(power);
    }

    public void write(double wheelSpeed){
        if(wheelFlipped) wheelSpeed*=-1;
        wheelSpeed *= Math.cos(error);
        drive.setVelocity(wheelSpeed);
    }

    public void getTelemetry(Telemetry telemetry){
        telemetry.addData("", name);
        telemetry.addData("Target", target);
        telemetry.addData("Current", current);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.addData("Wheel Flipped", wheelFlipped);
        telemetry.addData("Drive power", wheelVel);
    }

}