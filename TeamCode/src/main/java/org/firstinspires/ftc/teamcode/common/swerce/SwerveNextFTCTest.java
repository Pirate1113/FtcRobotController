//package org.firstinspires.ftc.teamcode.common.swerce;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import dev.nextftc.core.components.BindingsComponent;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.ftc.components.BulkReadComponent;
//
//@TeleOp
//public class SwerveNextFTCTest extends NextFTCOpMode {
//    public SwerveNextFTCTest(){
//        addComponents(
//                new SubsystemComponent(SwerveDrivetrain.INSTANCE),
//                BulkReadComponent.INSTANCE,
//                BindingsComponent.INSTANCE
//        );
//    }
//
//    @Override
//    public void onStartButtonPressed(){
//
//    }
//
//    @Override
//    public void onUpdate(){
//        //telemetry.addData("fl", SwerveDrivetrainBetter.INSTANCE.fl_Module.getPodHeading());
//        //telemetry.addData("bl", SwerveDrivetrainBetter.INSTANCE.bl_Module.getPodHeading());
//        //telemetry.addData("br", SwerveDrivetrainBetter.INSTANCE.br_Module.getPodHeading());
//        //telemetry.addData("fr", SwerveDrivetrainBetter.INSTANCE.fr_Module.getPodHeading());
//        telemetry.update();
//    }
//}
