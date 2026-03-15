package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Recycler;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Intake + Recycler TeleOp")
public class IntakeRecyclerTeleOp extends NextFTCOpMode {
    public IntakeRecyclerTeleOp() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Ramp.INSTANCE, Recycler.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        // Y: toggle selected color
        Gamepads.gamepad1().y().whenBecomesTrue(new InstantCommand(() -> {
            if (Recycler.INSTANCE.getSelectedColor() == Recycler.ColorChoice.GREEN) {
                Recycler.INSTANCE.selectPurple();
            } else {
                Recycler.INSTANCE.selectGreen();
            }
        }));

        // X: back intake toggle
        Gamepads.gamepad1().x().toggleOnBecomesTrue()
                .whenBecomesTrue(new ParallelGroup(
                        Intake.INSTANCE.moveBack,
                        Intake.INSTANCE.moveFront,
                        Ramp.INSTANCE.front
                ))
                .whenBecomesFalse(new ParallelGroup(
                        Intake.INSTANCE.stopBack,
                        Intake.INSTANCE.stopFront
                ));

        // B: front intake toggle
        Gamepads.gamepad1().b().toggleOnBecomesTrue()
                .whenBecomesTrue(new ParallelGroup(
                        Intake.INSTANCE.moveFront,
                        Ramp.INSTANCE.back
                ))
                .whenBecomesFalse(Intake.INSTANCE.stopFront);

        // A: ramp override
        Gamepads.gamepad1().a().toggleOnBecomesFalse()
                .whenBecomesTrue(Ramp.INSTANCE.front)
                .whenBecomesFalse(Ramp.INSTANCE.back);
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addLine("Y=color toggle  X=back intake  B=front intake  A=ramp");
        telemetry.addData("Selected Color", Recycler.INSTANCE.getSelectedColor());
        telemetry.addData("Empty?",         Recycler.INSTANCE.isEmpty());
        telemetry.addData("Sees Green?",    Recycler.INSTANCE.isGreen());
        telemetry.addData("Sees Purple?",   Recycler.INSTANCE.isPurple());
        telemetry.addData("Gate Position",  Recycler.INSTANCE.getGatePosition());
        telemetry.addLine("--- Raw Sensor ---");
        telemetry.addData("Red",   Recycler.INSTANCE.rawRed());
        telemetry.addData("Green", Recycler.INSTANCE.rawGreen());
        telemetry.addData("Blue",  Recycler.INSTANCE.rawBlue());
        telemetry.addLine("--- Intake ---");
        telemetry.addData("Intake Front Power", Intake.INSTANCE.getFrontPower());
        telemetry.addData("Intake Back Power",  Intake.INSTANCE.getBackPower());
        telemetry.addData("Ramp Pos", Ramp.INSTANCE.getRamp());
        telemetry.update();
    }
}
