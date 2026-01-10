package org.firstinspires.ftc.teamcode.swerveDrive;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class SwerveModule{

    final double WHEEL_RADIUS = 1;
    final double GEAR_RATIO = 1;
    final double TICKS_PER_REVOLUTION = 1;

    public static PIDCoefficients pidValues = new PIDCoefficients(0.5, 0, 0);
    public ControlSystem pid = ControlSystem.builder()
            .angular(AngleType.RADIANS, feedback -> feedback.posPid(pidValues))
            .build();

    private MotorEx driveMotor;
    private CRServoEx rotationServo;
    private AxonAnalog absoluteAnalog;

    public double xOffset;
    public double yOffset;

    public SwerveModule(MotorEx drivingMotor, String servoName,
                        boolean servoReversed, String analogName,
                        double analogOffset, boolean reverseAnalog, double analogMaxVolt,
                        double podXOffset, double podYOffset){

        driveMotor = drivingMotor.brakeMode();

        absoluteAnalog = new AxonAnalog(analogName, analogOffset, analogMaxVolt, reverseAnalog);

        rotationServo = new CRServoEx(servoName);
        if (servoReversed){
            rotationServo.getServo().setDirection(CRServo.Direction.REVERSE);
        }
        rotationServo.setPower(1);
        rotationServo.setPower(0);

        xOffset = podXOffset;
        yOffset = podYOffset;
    }

    public double getPodHeading(){
        return absoluteAnalog.getHeading();
    }

    public void rotateTo(double target){
        pid.setGoal(new KineticState(target));
        double power = pid.calculate(new KineticState(absoluteAnalog.getHeading()));
        rotationServo.setPower(power);
    }

    public void setMotorPower(double power){
        driveMotor.setPower(power);
    }
}

