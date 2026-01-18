package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();

    private Flywheel() {
    }

    private MotorEx shooter1;
    private MotorEx shooter2;

    public double distance;
    public double power = 0, power1 =0, power2 =0;

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();

    public void initialize() {
        shooter1 = new MotorEx("shooter1");
        shooter2 = new MotorEx("shooter2");
    }

    public void periodic() {
//        power1 = controller.calculate(shooter1.getState());
//        power2 = controller.calculate(shooter2.getState());
//        shooter1.setPower(power1);
//        shooter2.setPower(power2);
    }

    public void setDistance(double set) {
        distance = set;
    }

    public final Command off = new InstantCommand(() -> {
        power = 0;
        shooter1.setPower(power);
        shooter2.setPower(power);

    });

//    controller, 0.0,0).requires(this).named("FlywheelOff");
    public final Command runPID = new RunToVelocity(controller, 2200 + distance * 67,50).requires(this).named("FlywheelOn");
    public final Command stopPID = new RunToVelocity(controller, 0,0).requires(this).named("FlywheelOn");
    public final Command stopPID1 = new InstantCommand(() -> {
      new RunToVelocity(controller, 0, 0).schedule();
    });
    public final Command RunPID1 = new InstantCommand(() -> {
        new RunToVelocity(controller, 2200+distance*67, 50).schedule();
    });
//    RunToVelocity(controller, 0,0).requires(this).named("FlywheelOn");
    public final Command setFlywheel = new InstantCommand(() -> {
        power = 1;
        shooter1.setPower(-power);
        shooter2.setPower(power);

    });
    public double getRPM() {
        return -shooter1.getVelocity()*60/28;
    }
    public final Command backFlywheel = new InstantCommand(() -> {
        power =-0.2;
        shooter1.setPower(-power);
        shooter2.setPower(power);
    });
    public double getPower() {
        return power;
    }
    public double getGoal() {
        return power1;
    }
}
//        flywheel.setVelocity(rpm);
//    }

//
//    public Command moveLeft = new InstantCommand(() -> {
//        powerLeft = 0.8;
//        powerRight = -0.8;
//        servoLeft.getServo().setPower(powerLeft);
//        servoRight.getServo().setPower(-powerRight);
//    });
//package org.firstinspires.ftc.teamcode.testing;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//public class HoodAngle {
//
//    private final Servo hood;
//    private final DcMotorEx flywheel;
//
//    public HoodAngle(HardwareMap hw) {
//        hood = hw.get(Servo.class, "hoodServo");
//        flywheel = hw.get(DcMotorEx.class, "shooter1");
//
//        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//    }
//
//    /** Convert pitch to hood position */
//    public void setHoodFromPitch(double pitchDeg) {
//        double pos = 0.45 + pitchDeg * 1/360;  // need to tune 0.45 (initial hood angle), and the 0.002 prob should be tuned too
//        pos = Math.max(0.0, Math.min(1.0, pos));
//        hood.setPosition(pos);
//    }
//
//    /** Convert distance to RPM */
//    public void setFlywheelFromDistance(double distance) {
//        double rpm = 2200 + distance * 67; // wut wut wut 67
//        flywheel.setVelocity(rpm);
//    }
//
//    public void stop() {
//        flywheel.setPower(0);
//    }
//}
//package org.firstinspires.ftc.teamcode.subsystems;
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.utility.InstantCommand;
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.hardware.impl.CRServoEx;
//import dev.nextftc.hardware.powerable.SetPower;
//
//public class Spindexer implements Subsystem {
//    public static final Spindexer INSTANCE = new Spindexer();
//    private static final double POWER_INCREMENT = 0.2;
//
//    private Spindexer() {}
//
//    private CRServoEx servoLeft;
//    private CRServoEx servoRight;
//
//    private double powerLeft = 0.0;
//    private double powerRight = 0.0;
//
//    public void initialize() {
//        servoLeft = new CRServoEx("spindexerleft");
//        servoRight = new CRServoEx("spindexerright");
//    }
//
//    // ===== SYNCHRONIZED CONTROL =====
//
//    /** Run both servos at full power to intake samples */
//    public Command intake() {
//        powerLeft = 1.0;
//        powerRight = 1.0;
//        return new SetPower(servoLeft, 1.0)
//                .and(new SetPower(servoRight, 1.0))
//                .named("Intake");
//    }
//
//    /** Run both servos. in reverse to eject samples */
//    public Command outtake() {
//        powerLeft = -1.0;
//        powerRight = -1.0;
//        return new SetPower(servoLeft, -1.0)
//                .and(new SetPower(servoRight, -1.0))
//                .named("Outtake");
//    }
//
//
//    /** Set both servos to a specific power */
//    public Command setPower(double power) {
//        powerLeft = power;
//        powerRight = power;
//        return new SetPower(servoLeft, power)
//                .and(new SetPower(servoRight, power))
//                .named("Set Spindexer Power");
//    }
//
//    // ===== INDEPENDENT SERVO CONTROL =====
//
//    /** Increase left servo power by increment */
//
//    public Command moveLeft = new InstantCommand(() -> {
//        powerLeft = 0.8;
//        powerRight = -0.8;
//        servoLeft.getServo().setPower(powerLeft);
//        servoRight.getServo().setPower(-powerRight);
//    });
//    public Command moveRight = new InstantCommand(() -> {
//        powerRight = 0.8;
//        powerLeft = -0.8;
//        servoLeft.getServo().setPower(powerLeft);
//        servoRight.getServo().setPower(-powerRight);
//    });
//    public Command stop = new InstantCommand(() -> {
//        powerLeft = 0;
//        powerRight = 0;
//        servoLeft.getServo().setPower(powerLeft);
//        servoRight.getServo().setPower(-powerRight);
//    });
//
//    /** Set left servo to specific power */
//    public Command setLeftPower(double power) {
//        powerLeft = Math.max(-1.0, Math.min(1.0, power));
//        return new SetPower(servoLeft, powerLeft).named("Set Left Power");
//    }
//
//    /** Set right servo to specific power */
//    public Command setRightPower(double power) {
//        powerRight = Math.max(-1.0, Math.min(1.0, power));
//        return new SetPower(servoRight, powerRight).named("Set Right Power");
//    }
//
//    // ===== GETTERS FOR TELEMETRY =====
//
//    public double getLeftPower() {
//        return powerLeft;
//    }
//
//    public double getRightPower() {
//        return powerRight;
//    }
//}