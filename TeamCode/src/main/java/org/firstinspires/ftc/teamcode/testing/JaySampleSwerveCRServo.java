package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * An extended wrapper class for CRServos with more features
 * such as integration with absolute analog encoders for Axon servos
 * and their absolute encoders and power caching to reduce loop times.
 * Taken from SolversLib, fixed specifically for Swerve Mode
 *
 * @author Jay
 */

//TS MADE BY JAY KARANI FOR SAMPLING
public class JaySampleSwerveCRServo extends CRServo {

    private AbsoluteAnalogEncoder absoluteEncoder;
    private PIDFController pidf;

    /**
     * Used to detect target changes so PID can be reset
     */
    private Double lastTarget = null;


    /*
       Constructors
        */

    public SwerveCRServo(
            HardwareMap hwMap,
            String id,
            String encoderID,
            double analogRange,
            AngleUnit angleUnit

    ) {
        super(hwMap, id);
        this.absoluteEncoder = new AbsoluteAnalogEncoder(
                hwMap, encoderID, analogRange, angleUnit
        );

    }

    public SwerveCRServo(
            HardwareMap hwMap,
            String id,
            AbsoluteAnalogEncoder absoluteEncoder

    ) {
        super(hwMap, id);
        this.absoluteEncoder = absoluteEncoder;

    }

    /*
       Configuration Methods
        */

    public SwerveCRServo setPIDF(PIDFCoefficients coefficients) {
        this.pidf = new PIDFController(coefficients);
        return this;
    }

    public SwerveCRServo setAbsoluteEncoder(AbsoluteAnalogEncoder encoder) {
        this.absoluteEncoder = encoder;
        return this;
    }

    public AbsoluteAnalogEncoder getAbsoluteEncoder() {
        return absoluteEncoder;
    }

    /*
       Control Logic
        */

    @Override
    public void set(double output) {

        // Safety check
        if (pidf == null) {
            throw new IllegalStateException(
                    "OptimizedPositionalControl requires a PIDF"
            );
        }

        AngleUnit unit = absoluteEncoder.getAngleUnit();

        // Normalize target to [0, 2π)
        double target = MathUtils.normalizeAngle(output, true, unit);

        // Current position (already absolute, but normalize defensively)
        double current = MathUtils.normalizeAngle(
                absoluteEncoder.getCurrentPosition(), true, unit
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
        crServo.setPower(power);
    }

    /*
       Hardware helper methods
        */

    public SwerveCRServo setPwm(PwmControl.PwmRange pwmRange) {
        getController().setServoPwmRange(crServo.getPortNumber(), pwmRange);
        return this;
    }

    public ServoControllerEx getController() {
        return (ServoControllerEx) crServo.getController();
    }

    public com.qualcomm.robotcore.hardware.CRServo getServo() {
        return crServo;
    }

    @Override
    public String getDeviceType() {
        return "Extended Swerve " + super.getDeviceType();
    }
}