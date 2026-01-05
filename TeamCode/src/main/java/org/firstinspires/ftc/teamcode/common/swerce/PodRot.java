//package org.firstinspires.ftc.teamcode.common.swerce;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.robotcore.hardware.CRServoImplEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
//
//import dev.nextftc.control.ControlSystem;
//import dev.nextftc.control.feedback.AngleType;
//import dev.nextftc.control.feedback.PIDCoefficients;
//import dev.nextftc.hardware.impl.FeedbackCRServoEx;
//
//
//@Configurable
//public class PodRot {
//    private AbsoluteAnalogEncoder encoder;
//    private CRServoImplEx axon;
//    private FeedbackCRServoEx ax3on;
//
//    private double target;
//    private double current;
//    private double error;
//    private double velocity;
//
//    private boolean reversed;
//    private final boolean WHEEL_FLIPPING;
//
//    private double power;
//
//    public static double P, I, D, k;
//
//    private PIDCoefficients coeffs = new PIDCoefficients(P, I, D);
//    private ControlSystem rotController = ControlSystem.builder().angular(AngleType.RADIANS, feedback -> feedback.posPid(coeffs)).build();
//
//    public PodRot(CRServoImplEx a, AbsoluteAnalogEncoder enc, boolean reversed, boolean r, boolean motorFlipping) {
//        this.axon = a;
//        this.encoder = enc;
//        this.ax3on = new FeedbackCRServoEx()
//
//        this.reversed = r;
//
//        WHEEL_FLIPPING = motorFlipping;
//
//        if (reversed) {
//            axon.setDirection(DcMotorSimple.Direction.REVERSE);
//            encoder.setInverted(true);
//        }
//
//
//    }
//
//    public void goTo(double target) {
//        rotController.setGoal();
//
//    }
//}
