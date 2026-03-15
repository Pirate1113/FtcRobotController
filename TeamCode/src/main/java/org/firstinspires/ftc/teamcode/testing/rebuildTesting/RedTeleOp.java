package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Recycler;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Red TeleOp")
public class RedTeleOp extends NextFTCOpMode {

    public RedTeleOp() {
        addComponents(
                new SubsystemComponent(
                        SwerveDrivetrain.INSTANCE,
                        Turret.INSTANCE,
                        AutoAim.INSTANCE,
                        Intake.INSTANCE,
                        Ramp.INSTANCE,
                        Recycler.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        // ── Shooter ──────────────────────────────────────────────────────────
        // Right bumper: toggle auto-aim (limelight-driven RPM + hood angle)
        Gamepads.gamepad2().rightBumper().toggleOnBecomesTrue()
                .whenBecomesTrue(AutoAim.INSTANCE.enable)
                .whenBecomesFalse(AutoAim.INSTANCE.off);

        // Left bumper: full-power manual shot (bypasses auto-aim)
        Gamepads.gamepad2().leftBumper().toggleOnBecomesTrue()
                .whenBecomesTrue(AutoAim.INSTANCE.on)
                .whenBecomesFalse(AutoAim.INSTANCE.off);

        // Dpad right: reverse shooter (unjam)
        Gamepads.gamepad2().dpadRight().whenBecomesTrue(AutoAim.INSTANCE.reverse);

        // ── Recycler ─────────────────────────────────────────────────────────
        // Y: toggle wanted color (green ↔ purple)
        Gamepads.gamepad2().y().whenBecomesTrue(new InstantCommand(() -> {
            if (Recycler.INSTANCE.getSelectedColor() == Recycler.ColorChoice.GREEN) {
                Recycler.INSTANCE.selectPurple();
            } else {
                Recycler.INSTANCE.selectGreen();
            }
        }));

        // ── Intake ───────────────────────────────────────────────────────────
        // X: toggle both intakes + ramp forward (full feed)
        Gamepads.gamepad2().x().toggleOnBecomesTrue()
                .whenBecomesTrue(new ParallelGroup(
                        Intake.INSTANCE.moveBack,
                        Intake.INSTANCE.moveFront,
                        Ramp.INSTANCE.front
                ))
                .whenBecomesFalse(new ParallelGroup(
                        Intake.INSTANCE.stopBack,
                        Intake.INSTANCE.stopFront
                ));

        // B: toggle front intake + ramp reverse
        Gamepads.gamepad2().b().toggleOnBecomesTrue()
                .whenBecomesTrue(new ParallelGroup(
                        Intake.INSTANCE.moveFront,
                        Ramp.INSTANCE.back
                ))
                .whenBecomesFalse(Intake.INSTANCE.stopFront);

        // A: ramp override
        Gamepads.gamepad2().a().toggleOnBecomesTrue()
                .whenBecomesTrue(Ramp.INSTANCE.front)
                .whenBecomesFalse(Ramp.INSTANCE.back);
    }

    @Override
    public void onUpdate() {
        BindingManager.update();

        telemetry.addLine("── Controls ──────────────────────────");
        telemetry.addLine("RB (toggle) : auto-aim on/off");
        telemetry.addLine("LB (toggle) : manual full power on/off");
        telemetry.addLine("D→          : reverse shooter (unjam)");
        telemetry.addLine("Y           : toggle recycler color");
        telemetry.addLine("X (toggle)  : both intakes + ramp fwd");
        telemetry.addLine("B (toggle)  : front intake + ramp rev");
        telemetry.addLine("A (toggle)  : ramp override");
        telemetry.addLine("── Auto Aim ──────────────────────────");
        telemetry.addData("Enabled",        AutoAim.INSTANCE.isEnabled());
        telemetry.addData("Has Target",     AutoAim.INSTANCE.hasTarget());
        telemetry.addData("Distance (in)",  AutoAim.INSTANCE.getDistance());
        telemetry.addData("Yaw error (deg)", AutoAim.INSTANCE.getYaw());
        telemetry.addData("Target RPM",     AutoAim.INSTANCE.getTargetRPM());
        telemetry.addData("Current RPM",    AutoAim.INSTANCE.getCurrentRPM());
        telemetry.addData("Hood Position",  AutoAim.INSTANCE.getHoodPosition());
        telemetry.addLine("── Turret ────────────────────────────");
        telemetry.addData("Encoder angle (deg)", Turret.INSTANCE.getEncoderAngle());
        telemetry.addData("Target angle (deg)",  Turret.INSTANCE.getTargetAngle());
        telemetry.addLine("── Recycler ──────────────────────────");
        telemetry.addData("Selected Color", Recycler.INSTANCE.getSelectedColor());
        telemetry.addData("Sees Green",    Recycler.INSTANCE.isGreen());
        telemetry.addData("Sees Purple",   Recycler.INSTANCE.isPurple());
        telemetry.addData("Empty",         Recycler.INSTANCE.isEmpty());
        telemetry.addData("Gate Position", Recycler.INSTANCE.getGatePosition());
        telemetry.addLine("── Intake ────────────────────────────");
        telemetry.addData("Front Power",   Intake.INSTANCE.getFrontPower());
        telemetry.addData("Back Power",    Intake.INSTANCE.getBackPower());
        telemetry.addData("Ramp",          Ramp.INSTANCE.getRamp());
        telemetry.update();
    }
}
