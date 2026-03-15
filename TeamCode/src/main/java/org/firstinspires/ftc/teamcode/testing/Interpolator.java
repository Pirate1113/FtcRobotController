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
    private static final double LLHeight = 13.5;
    private  static final double tagHeight = 29.5;

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private ServoEx hood;



    public static double targetVel;
    double currentVel;

    // Velocity PIDF — tune via FTC Dashboard. F is most important for a flywheel.
    // Start with F only (P=I=D=0), increase F until motor approaches target, then add P.
    public static double PIDF_P = 5.0;
    public static double PIDF_I = 0.0;
    public static double PIDF_D = 0.0;
    public static double PIDF_F = 12.0;

    double distance;

    boolean dpadUpPrev = false;
    boolean dpadDownPrev = false;

    boolean aPrev = false;

    public static double servoCurrent = 0.0;
    public static double increment = 0.01;

    LimelightAngle limelight;



    @Override
    public void runOpMode() {
        limelight = new LimelightAngle(hardwareMap, "limelight", LLHeight, tagHeight);

        limelight.pipelineSwitch(0);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocityPIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F);
        shooter2.setVelocityPIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F);

        hood = new ServoEx("hood_servo");
        waitForStart();
        while (opModeIsActive()) {
            distance = limelight.getDistanceInches();
            double stick = -gamepad1.right_stick_y; // up = positive
            if (Math.abs(stick) < 0.05) stick = 0;
            servoCurrent += stick * increment;
            servoCurrent = Range.clip(servoCurrent, 0.0, 1.0);
//            servoCurrent = -36.79717*Math.pow(distance,-1.08794)+0.494168;
//            servoCurrent = Range.clip(servoCurrent, 0.0, 1.0);
            hood.setPosition(1-servoCurrent);



//            targetVel = (10.88255*distance+2691.0285)*28/60;
//            targetVel = Range.clip(targetVel, 0, 6000);


            if (gamepad1.dpad_up && !dpadUpPrev) {
                targetVel += 100;
            }
            if (gamepad1.dpad_down && !dpadDownPrev) {
                targetVel -= 100;
            }

            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;
//
//
//            aPrev = gamepad1.a;
            shooter1.setVelocity(-targetVel);
            shooter2.setVelocity(-targetVel);

            currentVel = -shooter1.getVelocity();

            telemetry.addLine("--- Controls ---");
            telemetry.addLine("D-Pad Up/Down : shooter speed +/-100 ticks/s");
            telemetry.addLine("Right Stick Y : hood servo position");
            telemetry.addLine("--- State ---");
            telemetry.addData("Shooter RPM (actual)", currentVel * 60 / 28);
            telemetry.addData("Shooter RPM (target)", targetVel * 60 / 28);
            telemetry.addData("Hood servo position", servoCurrent);
            telemetry.addData("Distance (in)", distance);
            telemetry.addLine("--- Debug ---");
            telemetry.addData("shooter1 raw vel (ticks/s)", shooter1.getVelocity());
            telemetry.addData("shooter2 raw vel (ticks/s)", shooter2.getVelocity());
            telemetry.addData("shooter1 pos (ticks)", shooter1.getCurrentPosition());
            telemetry.addData("shooter2 pos (ticks)", shooter2.getCurrentPosition());
            telemetry.addData("shooter1 power", shooter1.getPower());
            telemetry.addData("shooter2 power", shooter2.getPower());
            telemetry.addData("distance debug",  limelight.getDistanceInfo());
            telemetry.update();
        }
    }
}
