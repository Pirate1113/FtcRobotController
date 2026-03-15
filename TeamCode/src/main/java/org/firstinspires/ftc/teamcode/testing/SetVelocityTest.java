package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="SetVelocity Test")
public class SetVelocityTest extends LinearOpMode {

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    public static double targetVel = 500;
    public static final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");


        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);


        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            /*if (gamepad1.dpad_up && !dpadUpPrev) targetVel += 100;
            if (gamepad1.dpad_down && !dpadDownPrev) targetVel -= 100;
            targetVel = Range.clip(targetVel, 0, 6000);

            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;
*/
            shooter1.setVelocity(targetVel);
            shooter2.setVelocity(targetVel);

            /*double motorRPM = shooter1.getVelocity() * 60.0 / TICKS_PER_REV;

            // telemetry
            telemetry.addData("Target ticks/sec", targetVel);
            telemetry.addData("Motor RPM", motorRPM);
            telemetry.update();*/
        }
    }
}

