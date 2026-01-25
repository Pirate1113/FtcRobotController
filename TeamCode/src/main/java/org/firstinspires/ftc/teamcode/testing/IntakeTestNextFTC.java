package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.bindings.Bindings.*;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Palm;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@TeleOp(name = "Intake Test NextFTC Version")
public class IntakeTestNextFTC extends NextFTCOpMode {
    public IntakeTestNextFTC() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Spindexer.INSTANCE,Palm.INSTANCE,Flywheel.INSTANCE, SwerveDrivetrain.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        //left intake control

        Gamepads.gamepad2().b().toggleOnBecomesTrue().whenBecomesTrue(Intake.INSTANCE.moveLeft).whenBecomesFalse(Intake.INSTANCE.stopLeft);
        Gamepads.gamepad2().x().toggleOnBecomesTrue().whenBecomesTrue(Intake.INSTANCE.moveRight).whenBecomesFalse(Intake.INSTANCE.stopRight);
        Gamepads.gamepad2().dpadUp().toggleOnBecomesTrue().whenBecomesTrue(Spindexer.INSTANCE.moveRight).whenBecomesFalse(Spindexer.INSTANCE.stop);
        Gamepads.gamepad2().dpadDown().toggleOnBecomesTrue().whenBecomesTrue(Spindexer.INSTANCE.moveLeft).whenBecomesFalse(Spindexer.INSTANCE.stop);
        Gamepads.gamepad2().a().toggleOnBecomesFalse().whenBecomesTrue(Spindexer.INSTANCE.eject).whenBecomesFalse(Spindexer.INSTANCE.uneject);
//        Button dpadLeft = button(() -> someBooleanCondition);

// for example:
        Gamepads.gamepad2().y().toggleOnBecomesFalse().whenBecomesTrue(Palm.INSTANCE.on).whenBecomesFalse(Palm.INSTANCE.off);
        Gamepads.gamepad2().rightTrigger().greaterThan(0.2).whenBecomesTrue(Flywheel.INSTANCE.setFlywheel).whenBecomesFalse(Flywheel.INSTANCE.off);
        Gamepads.gamepad2().rightBumper().toggleOnBecomesFalse().whenBecomesTrue(Flywheel.INSTANCE.backFlywheel).whenBecomesFalse(Flywheel.INSTANCE.off);


    }
    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("Left Intake Power", Intake.INSTANCE.getLeftPower());
        telemetry.addData("Right Intake Power", Intake.INSTANCE.getRightPower());
        telemetry.addData("Left Servo Pow", "%.2f", Spindexer.INSTANCE.getLeftPower());
        telemetry.addData("Right Servo Pow", "%.2f", Spindexer.INSTANCE.getRightPower());
        telemetry.addData("EjectorPos", "%.2f", Spindexer.INSTANCE.getEjectorPos());
        telemetry.addData("PalmPos", Palm.INSTANCE.getPos());
        telemetry.addData("Flywheel RPM", Flywheel.INSTANCE.getRPM());
        telemetry.addData("Flywheel Power", Flywheel.INSTANCE.getPower());
        telemetry.update();

    }



}
