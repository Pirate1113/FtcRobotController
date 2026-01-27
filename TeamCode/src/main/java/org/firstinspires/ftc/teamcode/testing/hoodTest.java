package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import org.firstinspires.ftc.teamcode.subsystems.Hood;

@TeleOp(name = "HoodTest")
public class hoodTest extends NextFTCOpMode {
    public hoodTest() {
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE,Palm.INSTANCE,Flywheel.INSTANCE, Hood.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        //left intake control

        Gamepads.gamepad2().a().toggleOnBecomesFalse().whenBecomesTrue(Spindexer.INSTANCE.eject).whenBecomesFalse(Spindexer.INSTANCE.uneject);
//        Button dpadLeft = button(() -> someBooleanCondition);

// for example:
        Gamepads.gamepad2().y().toggleOnBecomesFalse().whenBecomesTrue(Palm.INSTANCE.on).whenBecomesFalse(Palm.INSTANCE.off);
        Gamepads.gamepad2().rightTrigger().greaterThan(0.2).whenBecomesTrue(Flywheel.INSTANCE.setFlywheel).whenBecomesFalse(Flywheel.INSTANCE.off);
        Gamepads.gamepad2().rightBumper().toggleOnBecomesFalse().whenBecomesTrue(Flywheel.INSTANCE.backFlywheel).whenBecomesFalse(Flywheel.INSTANCE.off);
        Gamepads.gamepad2().leftStickY().greaterThan(0.5).whenBecomesTrue(Hood.INSTANCE.incHood).whenBecomesFalse(Hood.INSTANCE.decHood);
    }
    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("EjectorPos", "%.2f", Spindexer.INSTANCE.getEjectorPos());
        telemetry.addData("PalmPos", Palm.INSTANCE.getPos());
        telemetry.addData("Flywheel RPM", Flywheel.INSTANCE.getRPM());
        telemetry.addData("Flywheel Power", Flywheel.INSTANCE.getPower());
        telemetry.addData("Hood Pos", Hood.INSTANCE.getPos());
        telemetry.addData("YStick", Gamepads.gamepad2().leftStickY().get());
        telemetry.update();

    }



}
