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
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TestSpindexer;


@TeleOp(name = "Intake Test NextFTC Version")
public class IntakeTestNextFTC extends NextFTCOpMode {
    public IntakeTestNextFTC() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Spindexer.INSTANCE,TestSpindexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed() {
        //left intake control

        Gamepads.gamepad1().b().toggleOnBecomesTrue().whenBecomesTrue(Intake.INSTANCE.moveLeft).whenBecomesFalse(Intake.INSTANCE.stopLeft);
        Gamepads.gamepad1().x().toggleOnBecomesTrue().whenBecomesTrue(Intake.INSTANCE.moveRight).whenBecomesFalse(Intake.INSTANCE.stopRight);
        Gamepads.gamepad1().dpadUp().toggleOnBecomesTrue().whenBecomesTrue(Spindexer.INSTANCE.moveRight).whenBecomesFalse(Spindexer.INSTANCE.stop);
        Gamepads.gamepad1().dpadDown().toggleOnBecomesTrue().whenBecomesTrue(Spindexer.INSTANCE.moveLeft).whenBecomesFalse(Spindexer.INSTANCE.stop);
        Gamepads.gamepad1().a().toggleOnBecomesTrue().whenBecomesTrue(Spindexer.INSTANCE.eject).whenBecomesFalse(Spindexer.INSTANCE.uneject);
        Gamepads.gamepad1().y().toggleOnBecomesTrue().whenBecomesTrue(TestSpindexer.INSTANCE.b1).whenBecomesFalse(TestSpindexer.INSTANCE.b2);
    }
    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("Left Intake Power", Intake.INSTANCE.getLeftPower());
        telemetry.addData("Right Intake Power", Intake.INSTANCE.getRightPower());
        telemetry.addData("Left Servo Pow", "%.2f", Spindexer.INSTANCE.getLeftPower());
        telemetry.addData("Right Servo Pow", "%.2f", Spindexer.INSTANCE.getRightPower());
        telemetry.addData("Left Servo Pos", "%.2f", TestSpindexer.INSTANCE.getLeftRawPosition());
        telemetry.addData("Right Servo Pos", "%.2f", TestSpindexer.INSTANCE.getRightRawPosition());
        telemetry.update();

    }
}
