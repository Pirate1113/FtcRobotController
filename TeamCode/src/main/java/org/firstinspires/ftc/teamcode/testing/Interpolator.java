package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Config
@TeleOp(name="Interpolating test")
public class Interpolator extends LinearOpMode {
    private static final double LLHeight = 14;
    private  static final double tagHeight = 29.5;

    private MotorEx shooter1;
    private MotorEx shooter2;

    private ServoEx hood;

    private ServoEx palm;

    public static double P = 0.02;
    public static double I = 0.00;
    public static double D = 0.1;

    public static double targetRPM;
    double currentRPM;

    double distance;

    boolean dpadUpPrev = false;
    boolean dpadDownPrev = false;

    boolean palmOn = false;
    boolean aPrev = false;

    public static double servoCurrent = 0.0;
    public static double increment = 0.05;

    LimelightAngle limelight;

    private final ControlSystem vpid = ControlSystem.builder()
            .velPid(P,I,D)
            .build();


    @Override
    public void runOpMode() {
        limelight = new LimelightAngle(hardwareMap, "limelight", LLHeight, tagHeight);

        limelight.pipelineSwitch(0);

        shooter1 = new MotorEx("shooter1", 0.05);
        shooter2 = new MotorEx("shooter2", 0.05);
        shooter1.setDirection(-1);
        shooter2.setDirection(1);

        hood = new ServoEx("hoodServo");

        palm = new ServoEx("feedServo");

        waitForStart();

        while (opModeIsActive()) {
            distance = limelight.getDistanceInches();

            double stick = -gamepad1.right_stick_y; // up = positive
            if (Math.abs(stick) < 0.05) stick = 0;
            servoCurrent += stick * increment;
            servoCurrent = Range.clip(servoCurrent, 0.0, 1.0);
            hood.setPosition(servoCurrent);


            if (gamepad1.dpad_up && !dpadUpPrev) {
                targetRPM += 100;
            }
            if (gamepad1.dpad_down && !dpadDownPrev) {
                targetRPM -= 100;
            }

            targetRPM = Range.clip(targetRPM, 0, 6000);

            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;

            if (gamepad1.a && !aPrev) {
                palmOn = !palmOn;

                if (palmOn) {
                    palm.setPosition(0);
                } else {
                    palm.setPosition(0.3);
                }
            }

            aPrev = gamepad1.a;

            currentRPM = shooter1.getVelocity();

            vpid.setGoal(new KineticState(0, targetRPM));

            double power = vpid.calculate(new KineticState(
                    shooter1.getCurrentPosition(),
                    shooter1.getVelocity()
                )
            );

            shooter1.setPower(power);
            shooter2.setPower(power);

            telemetry.addData("shooter1 current RPM: ", currentRPM);
            telemetry.addData("target RPM: ", targetRPM);
            telemetry.addData("hood servo current: ", servoCurrent);
            telemetry.addData("distance: ", distance);

            telemetry.update();
        }
    }
}
