package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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



    @Override
    public void runOpMode() {
        limelight = new LimelightAngle(hardwareMap, "limelight", LLHeight, tagHeight);

        limelight.pipelineSwitch(0);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hood = new ServoEx("hoodServo");

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

            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;


            aPrev = gamepad1.a;

//            vpid.setGoal(new KineticState(0, targetVel));
//
//            double power = vpid.calculate(new KineticState(
//                    shooter1.getCurrentPosition(),
//                    shooter1.getVelocity()
//                )
//            );

//            shooter1.setPower(power);
//            shooter2.setPower(power);
            shooter1.setVelocity(-targetVel);
            shooter2.setVelocity(-targetVel);

            currentVel = -shooter1.getVelocity();

            telemetry.addData("Shooter current RPM: ", currentVel*60/28);
            telemetry.addData("target RPM: ", targetVel*60/28);
            telemetry.addData("Hood servo current: ", servoCurrent);
            telemetry.addData("distance: ", distance);
            telemetry.update();
        }
    }
}
