package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;

@TeleOp(name = "Recycler Test")
public class RecyclerTeleOp extends NextFTCOpMode {
    public RecyclerTeleOp() {
        addComponents(new SubsystemComponent(Recycler.INSTANCE));
    }

    @Override
    public void onUpdate() {
        // controller buttons each update
        if (gamepad1.a) {
            Recycler.INSTANCE.selectGreen();
        }

        if (gamepad1.b) {
            Recycler.INSTANCE.selectPurple();
        }

        //  telemetry to see selected color and detected colors
        telemetry.addData("Selected Color", Recycler.INSTANCE.getSelectedColor());
        telemetry.addData("Detected Green?", Recycler.INSTANCE.isGreen());
        telemetry.addData("Detected Purple?", Recycler.INSTANCE.isPurple());
        telemetry.update();
    }
}