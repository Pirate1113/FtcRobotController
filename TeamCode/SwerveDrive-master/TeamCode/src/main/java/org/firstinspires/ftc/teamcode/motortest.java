package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="motortest", group="TeleOp")
public class motortest extends OpMode {

    // Hardware
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;



    // Button debouncing
    private boolean dpadUpPreviousState = false;
    private boolean dpadDownPreviousState = false;
    private boolean dpadLeftPreviousState = false;
    private boolean dpadRightPreviousState = false;

    @Override
    public void init() {
        // Initialize hardware
        FLDrive = hardwareMap.get(DcMotor.class, "fl_motor");
        FRDrive = hardwareMap.get(DcMotor.class, "fr_motor");
        BLDrive = hardwareMap.get(DcMotor.class, "bl_motor");
        BRDrive = hardwareMap.get(DcMotor.class, "br_motor");

        // Set motor directions (adjust if needed)
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Initialize servos to retracted position

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Speed control with D-pad (with debouncing)
        boolean dpadUpCurrentState = gamepad1.dpad_up;
        boolean dpadDownCurrentState = gamepad1.dpad_down;
        boolean dpadLeftCurrentState = gamepad1.dpad_left;
        boolean dpadRightCurrentState = gamepad1.dpad_right;

        // Increase speed with D-pad Up
        if (dpadUpCurrentState && !dpadUpPreviousState) {
            FLDrive.setPower(1.0);

        }
        if (dpadDownCurrentState && !dpadDownPreviousState) {
            FRDrive.setPower(1.0);

        }
        if (dpadLeftCurrentState && !dpadLeftPreviousState) {
            BLDrive.setPower(1.0);

        }
        if (dpadRightCurrentState && !dpadRightPreviousState) {
            BRDrive.setPower(1.0);

        }


    }

    @Override
    public void stop() {
        // Stop motors and retract servos when op mode ends
        FLDrive.setPower(0);

    }
}