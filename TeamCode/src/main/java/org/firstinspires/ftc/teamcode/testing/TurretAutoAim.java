//package org.firstinspires.ftc.teamcode.testing;
//
//import dev.nextftc.core.commands.Command;
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//import org.firstinspires.ftc.teamcode.testing.LimelightAngle;
//
//public class TurretAutoAim extends Command {
//
//    private final Turret turret;
//    private final LimelightAngle limelight;
//    private final double maxDeltaPerUpdate;
//
//    public TurretAutoAim(Turret turret, LimelightAngle limelight, double maxDeltaPerUpdate) {
//        this.turret = turret;
//        this.limelight = limelight;
//        this.maxDeltaPerUpdate = maxDeltaPerUpdate;
//    }
//
//    @Override
//    public void start() {
//    }
//
//    @Override
//    public void update() {
//
//        if (!limelight.hasTarget()) return;
//
//        double tx = limelight.getYaw();
//
//        double newAngle = turret.getAngle() + tx;
//
//        if (Math.abs(tx) > maxDeltaPerUpdate) {
//            tx = Math.signum(tx) * maxDeltaPerUpdate;
//        }
//
//        turret.adjustAngle(tx);
//    }
//
//    @Override
//    public boolean isDone() {
//        return false;
//    }
//
//    @Override
//    public void stop(boolean interrupted) {
//    }
//}
