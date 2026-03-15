

package org.firstinspires.ftc.teamcode.swerveDrive;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


@TeleOp(name="AxonTester")
public class AxonTester extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //servos
    private CRServo frontRight;
    private CRServo frontLeft;
    private CRServo backLeft;
    private CRServo backRight;


    //encoders
    private AnalogInput axon1;
    private AnalogInput axon2;
    private AnalogInput axon3;
    private AnalogInput axon4;

    private static final double TWO_PI = 2 * Math.PI;


    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.get(CRServo.class, "fr_rotation");
        frontLeft = hardwareMap.get(CRServo.class, "fl_rotation");
        backLeft = hardwareMap.get(CRServo.class, "bl_rotation");
        backRight = hardwareMap.get(CRServo.class, "br_rotation");

        axon1 = hardwareMap.analogInput.get("fr_absolute");
        axon2 = hardwareMap.analogInput.get("fl_absolute");
        axon3 = hardwareMap.analogInput.get("bl_absolute");
        axon4 = hardwareMap.analogInput.get("br_absolute");


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("svoltage", "%4.2f, %4.2f, %4.2f %4.2f", axon1.getVoltage(), axon2.getVoltage(), axon3.getVoltage(), axon4.getVoltage());

            telemetry.addData("fr_servo voltage", axon1.getVoltage());
            telemetry.addData("fl_servo voltage", axon2.getVoltage());
            telemetry.addData("bl_servo voltage", axon3.getVoltage());
            telemetry.addData("br_servo voltage", axon4.getVoltage());

            telemetry.addData("fr_servo pos", axon1.getVoltage() * TWO_PI/3.3);
            telemetry.addData("fl_servo pos", axon2.getVoltage() * TWO_PI/3.3);
            telemetry.addData("bl_servo pos", axon3.getVoltage() * TWO_PI/3.3);
            telemetry.addData("br_servo pos", axon4.getVoltage() * TWO_PI/3.3);

            telemetry.update();
        }
    }}