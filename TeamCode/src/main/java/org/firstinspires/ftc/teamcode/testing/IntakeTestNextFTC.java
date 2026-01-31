package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.subsystems.Palm;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@TeleOp(name = "Intake Test NextFTC Version")
public class IntakeTestNextFTC extends NextFTCOpMode {
    public IntakeTestNextFTC() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Spindexer.INSTANCE,Palm.INSTANCE,Flywheel.INSTANCE, SwerveDrivetrain.INSTANCE, Hood.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        //left intake control

        Gamepads.gamepad2().b().toggleOnBecomesTrue().whenBecomesTrue(Intake.INSTANCE.moveLeft).whenBecomesFalse(Intake.INSTANCE.stopLeft);
        Gamepads.gamepad2().x().toggleOnBecomesTrue().whenBecomesTrue(Intake.INSTANCE.moveRight).whenBecomesFalse(Intake.INSTANCE.stopRight);

        Gamepads.gamepad2().dpadLeft().toggleOnBecomesFalse().whenBecomesTrue(Spindexer.INSTANCE.b1);
        Gamepads.gamepad2().dpadUp().toggleOnBecomesFalse().whenBecomesTrue(Spindexer.INSTANCE.b2);
        Gamepads.gamepad2().dpadRight().toggleOnBecomesFalse().whenBecomesTrue(Spindexer.INSTANCE.b3);
        Gamepads.gamepad2().a().toggleOnBecomesFalse().whenBecomesTrue(Spindexer.INSTANCE.eject).whenBecomesFalse(Spindexer.INSTANCE.uneject);//        Button dpadLeft = button(() -> someBooleanCondition);
        Gamepads.gamepad2().y().toggleOnBecomesFalse().whenBecomesTrue(Palm.INSTANCE.on).whenBecomesFalse(Palm.INSTANCE.off);
        Gamepads.gamepad2().rightTrigger().greaterThan(0.2).whenBecomesTrue(Flywheel.INSTANCE.setFlywheel).whenBecomesFalse(Flywheel.INSTANCE.off);
        Gamepads.gamepad2().rightBumper().toggleOnBecomesFalse().whenBecomesTrue(Flywheel.INSTANCE.backFlywheel).whenBecomesFalse(Flywheel.INSTANCE.off);
    }
    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("Left Intake Power", Intake.INSTANCE.getLeftPower());
        telemetry.addData("Right Intake Power", Intake.INSTANCE.getRightPower());
        telemetry.addData("Spindexer Power", Spindexer.INSTANCE.getLeftPower());
        telemetry.addData("Spindexer Goal", Spindexer.INSTANCE.getLeftGoal());
        telemetry.addData("Spindexer Pos", Spindexer.INSTANCE.getLeftPosition());
        telemetry.addData("EjectorPos", "%.2f", Spindexer.INSTANCE.getEjectorPos());
        telemetry.addData("PalmPos", Palm.INSTANCE.getPos());
        telemetry.addData("Flywheel RPM", Flywheel.INSTANCE.getRPM());
        telemetry.addData("Flywheel Power", Flywheel.INSTANCE.getPower());
        telemetry.update();

    }



}
