package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.DashBoardSpindexer;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "spindexerTuneOpMode")
public class spindexerTuneOpMode extends NextFTCOpMode {
    public spindexerTuneOpMode() {
        addComponents(
                new SubsystemComponent(DashBoardSpindexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        //left intake control
    }
    @Override
    public void onUpdate() {
    //        telemetry.addData("Left Servo Pow", "%.2f", Spindexer.INSTANCE.getLeftPower());
//        telemetry.addData("Right Servo Pow", "%.2f", Spindexer.INSTANCE.getRightPower());
//        telemetry.addData("EjectorPos", "%.2f", Spindexer.INSTANCE.getEjectorPos());
//        telemetry.addData("PalmPos", Palm.INSTANCE.getPos());
//        telemetry.addData("Flywheel RPM", Flywheel.INSTANCE.getRPM());
//        telemetry.addData("Flywheel Power", Flywheel.INSTANCE.getPower());

    }



}
