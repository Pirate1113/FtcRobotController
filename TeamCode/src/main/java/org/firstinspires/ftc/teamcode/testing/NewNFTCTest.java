package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Testinggggggggggg")
public class NewNFTCTest extends NextFTCOpMode {
    public NewNFTCTest() {
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        //left intake control

        Gamepads.gamepad2().b().toggleOnBecomesTrue().whenBecomesTrue(Spindexer.INSTANCE.b1).whenBecomesFalse(Spindexer.INSTANCE.b2);
        Gamepads.gamepad2().x().toggleOnBecomesTrue().whenBecomesTrue(Spindexer.INSTANCE.b3);
        Gamepads.gamepad2().y().toggleOnBecomesFalse().whenBecomesTrue(Spindexer.INSTANCE.eject).whenBecomesFalse(Spindexer.INSTANCE.uneject);
        Gamepads.gamepad2().a().toggleOnBecomesFalse().whenBecomesTrue(() -> Spindexer.INSTANCE.shoot().schedule());
    }
    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("Left Power", Spindexer.INSTANCE.getLeftPower());
        telemetry.addData("Left Goal", Spindexer.INSTANCE.getLeftGoal());
        telemetry.addData("Left Pos", Spindexer.INSTANCE.getLeftPosition());
        telemetry.addData("Ejector Pos", Spindexer.INSTANCE.getEjectorPos());
//        telemetry.addData("Left Servo Pow", "%.2f", Spindexer.INSTANCE.getLeftPower());
//        telemetry.addData("Right Servo Pow", "%.2f", Spindexer.INSTANCE.getRightPower());
//        telemetry.addData("EjectorPos", "%.2f", Spindexer.INSTANCE.getEjectorPos());
//        telemetry.addData("PalmPos", Palm.INSTANCE.getPos());
//        telemetry.addData("Flywheel RPM", Flywheel.INSTANCE.getRPM());
//        telemetry.addData("Flywheel Power", Flywheel.INSTANCE.getPower());
        telemetry.update();

    }



}
