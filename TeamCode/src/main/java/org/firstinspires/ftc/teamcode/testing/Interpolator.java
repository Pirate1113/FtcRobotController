package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.hardware.impl.ServoEx;

@Config
@TeleOp(name="Interpolating test")
public class Interpolator extends LinearOpMode {
    private static final double LLHeight = 14;
    private  static final double tagHeight = 29.5;

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private ServoEx hood;

    private ServoEx palm;

    public static double P = 0.02;
    public static double I = 0.00;
    public static double D = 0.1;

    public static double targetVel;
    double currentVel;

    double distance;

    boolean dpadUpPrev = false;
    boolean dpadDownPrev = false;

    boolean palmOn = false;
    boolean aPrev = false;

    public static double servoCurrent = 0.0;
    public static double increment = 0.01;

    LimelightAngle limelight;

    private final ControlSystem vpid = ControlSystem.builder()
            .velPid(P,I,D)
            .build();


    @Override
    public void runOpMode() {
        limelight = new LimelightAngle(hardwareMap, "limelight", LLHeight, tagHeight);

        limelight.pipelineSwitch(0);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

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
                targetVel += 100;
            }
            if (gamepad1.dpad_down && !dpadDownPrev) {
                targetVel -= 100;
            }

            targetVel = Range.clip(targetVel, 0, 6000);

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

            vpid.setGoal(new KineticState(0, targetVel));

            double power = vpid.calculate(new KineticState(
                    shooter1.getCurrentPosition(),
                    shooter1.getVelocity()
                )
            );

//            shooter1.setPower(power);
//            shooter2.setPower(power);
            shooter1.setVelocity(-targetVel);
            shooter2.setVelocity(-targetVel);

            currentVel = shooter1.getVelocity();

            telemetry.addData("shooter1 current RPM: ", currentVel*60/28);
            telemetry.addData("target RPM: ", targetVel*60/28);
            telemetry.addData("hood servo current: ", servoCurrent);
            telemetry.addData("distance: ", distance);
            telemetry.addData(
                    "real? shooter1 RPM",
                    Math.abs(shooter1.getVelocity()) * 60.0 / 28.0
            );
            telemetry.addData(
                    "real? target RPM",
                    targetVel * 60.0 / 28.0
            );
            telemetry.update();
        }
    }
}
