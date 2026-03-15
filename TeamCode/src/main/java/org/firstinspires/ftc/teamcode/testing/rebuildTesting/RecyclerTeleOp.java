package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Recycler;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Recycler Test")
public class RecyclerTeleOp extends NextFTCOpMode {
    public RecyclerTeleOp() {
        addComponents(
                new SubsystemComponent(Recycler.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().a().whenBecomesTrue(new InstantCommand(() -> Recycler.INSTANCE.selectGreen()));
        Gamepads.gamepad1().b().whenBecomesTrue(new InstantCommand(() -> Recycler.INSTANCE.selectPurple()));
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("Selected Color", Recycler.INSTANCE.getSelectedColor());
        telemetry.addData("Sees Green?", Recycler.INSTANCE.isGreen());
        telemetry.addData("Sees Purple?", Recycler.INSTANCE.isPurple());
        telemetry.addData("Gate Position", Recycler.INSTANCE.getGatePosition());
        telemetry.addLine("--- Raw Sensor ---");
        telemetry.addData("Red",   Recycler.INSTANCE.rawRed());
        telemetry.addData("Green", Recycler.INSTANCE.rawGreen());
        telemetry.addData("Blue",  Recycler.INSTANCE.rawBlue());
        telemetry.update();
    }
}
