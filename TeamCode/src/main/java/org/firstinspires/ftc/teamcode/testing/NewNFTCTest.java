package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TestSpindexer;

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

@TeleOp(name = "Testinggggggggggg")
public class NewNFTCTest extends NextFTCOpMode {
    public NewNFTCTest() {
        addComponents(
                new SubsystemComponent(TestSpindexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        //left intake control

        Gamepads.gamepad2().b().toggleOnBecomesTrue().whenBecomesTrue(TestSpindexer.INSTANCE.b1).whenBecomesFalse(TestSpindexer.INSTANCE.b2);
        Gamepads.gamepad2().x().toggleOnBecomesTrue().whenBecomesTrue(TestSpindexer.INSTANCE.b3);
    }
    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("Left Power", TestSpindexer.INSTANCE.getLeftPower());
        telemetry.addData("Left Goal", TestSpindexer.INSTANCE.getLeftGoal());
        telemetry.addData("Left Pos", TestSpindexer.INSTANCE.getLeftPosition());
        telemetry.addData("Left Raw Pos", TestSpindexer.INSTANCE.getLeftRawPosition());
//        telemetry.addData("Left Servo Pow", "%.2f", Spindexer.INSTANCE.getLeftPower());
//        telemetry.addData("Right Servo Pow", "%.2f", Spindexer.INSTANCE.getRightPower());
//        telemetry.addData("EjectorPos", "%.2f", Spindexer.INSTANCE.getEjectorPos());
//        telemetry.addData("PalmPos", Palm.INSTANCE.getPos());
//        telemetry.addData("Flywheel RPM", Flywheel.INSTANCE.getRPM());
//        telemetry.addData("Flywheel Power", Flywheel.INSTANCE.getPower());
        telemetry.update();

    }



}
