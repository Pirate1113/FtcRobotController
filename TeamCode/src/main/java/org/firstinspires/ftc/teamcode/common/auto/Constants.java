import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
        .forwardEncoder_HardwareMapName("lf_encoder")
        .strafeEncoder_HardwareMapName("rb_encoder")
        .forwardTicksToInches(multiplier) //tune
        .strafeTicksToInches(multiplier) //tune
        .IMU_HardwareMapName("imu")
        .IMU_Orientation(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        return new <hardwareMap, followerConstants> FollowerBuilder(followerConstants, hardwareMap)
    .twoWheelLocalizer(localizerConstants)
/* other builder steps */
    .build();