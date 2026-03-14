package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "TurretTest")
public class TurretTest extends NextFTCOpMode {

    private static final double STEP = 2.0;   // degrees per dpad press
    private static final double SCALE = 1.5;  // degrees per loop tick from stick

    public TurretTest() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.setAngle(90.0); // center on start

        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(
                new InstantCommand(() -> Turret.INSTANCE.adjustAngle(-STEP)));
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(
                new InstantCommand(() -> Turret.INSTANCE.adjustAngle(STEP)));
        Gamepads.gamepad1().a().whenBecomesTrue(
                new InstantCommand(() -> Turret.INSTANCE.setAngle(90.0)));
    }

    @Override
    public void onUpdate() {
        BindingManager.update();

        double stickX = gamepad1.left_stick_x;
        if (Math.abs(stickX) > 0.05) {
            Turret.INSTANCE.adjustAngle(stickX * SCALE);
        }

        telemetry.addData("Turret Angle", "%.1f°", Turret.INSTANCE.getAngle());
        telemetry.addData("Left Stick X", "%.2f", stickX);
        telemetry.addLine("D-Pad L/R: step  |  A: center");
        telemetry.update();
    }
}