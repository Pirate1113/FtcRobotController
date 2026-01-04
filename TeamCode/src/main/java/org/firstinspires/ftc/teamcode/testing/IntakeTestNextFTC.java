package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

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

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(Intake.INSTANCE.increaseLeft());
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(Intake.INSTANCE.decreaseLeft());
        Gamepads.gamepad1().x()
                        .whenBecomesTrue(Intake.INSTANCE.increaseRight());
        Gamepads.gamepad1().a().whenBecomesTrue(Intake.INSTANCE.decreaseRight());
        //TODO do telemetry later lmao
    }
}
