package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.bindings.Bindings.*;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Intake Test NextFTC Version")
public class IntakeTestNextFTC extends NextFTCOpMode {
    public IntakeTestNextFTC() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed() {
        //left intake control

        Gamepads.gamepad1().dpadUp().whenTrue(Intake.INSTANCE.increaseLeft);
        Gamepads.gamepad1().dpadDown()
                .whenTrue(Intake.INSTANCE.decreaseLeft);
        Gamepads.gamepad1().x()
                        .whenTrue(Intake.INSTANCE.increaseRight());
        Gamepads.gamepad1().a().whenTrue(Intake.INSTANCE.decreaseRight());

    }
    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("Left Intake Power", Intake.INSTANCE.getLeftPower());
        telemetry.addData("Right Intake Power", Intake.INSTANCE.getRightPower());
        telemetry.update();

    }
}
