//
//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.pedropathing.math.Pose;
//
//import org.firstinspires.ftc.teamcode.config.Alliance;
//import org.firstinspires.ftc.teamcode.config.AllianceState;
//import org.firstinspires.ftc.teamcode.config.RobotConfig;
//
//import java.util.function.Supplier;
//
//import dev.nextftc.hardware.actuators.NextMotor;
//import dev.nextftc.robot.Mechanism;
//import dev.nextftc.units.Units;
//
//public class Turret implements Mechanism {
//
//    private final NextMotor turret = new NextMotor("turret");
//
//    private Supplier<Pose> poseSupplier;
//    private Supplier<Boolean> umbrellaActiveSupplier;
//    private Supplier<Shooter.Cell> cellSupplier;
//
//    private boolean manualOverride = false;
//    private double manualAngleDeg = 0.0;
//
//    public Turret() {
//        turret.setDirection(NextMotor.Direction.FORWARD);
//    }
//
//    public void setCellSupplier(Supplier<Shooter.Cell> supplier) {
//        cellSupplier = supplier;
//    }
//
//    public void setPoseSupplier(Supplier<Pose> supplier) {
//        poseSupplier = supplier;
//    }
//
//    public void setUmbrellaActiveSupplier(Supplier<Boolean> supplier) {
//        umbrellaActiveSupplier = supplier;
//    }
//
//    public void setManualAngle(double angleDeg) {
//        manualOverride = true;
//        manualAngleDeg = angleDeg;
//    }
//
//    public void clearManualOverride() {
//        manualOverride = false;
//    }
//
//    private double[] getGoalPosition(Shooter.Cell cell) {
//        boolean red = AllianceState.current == Alliance.RED;
//        boolean left = cell == Shooter.Cell.LEFT;
//
//        if (red) {
//            return left
//                    ? new double[]{
//                    RobotConfig.Turret.goalRedLeftX,
//                    RobotConfig.Turret.goalRedLeftY
//            }
//                    : new double[]{
//                    RobotConfig.Turret.goalRedRightX,
//                    RobotConfig.Turret.goalRedRightY
//            };
//        }
//
//        return left
//                ? new double[]{
//                RobotConfig.Turret.goalBlueLeftX,
//                RobotConfig.Turret.goalBlueLeftY
//        }
//                : new double[]{
//                RobotConfig.Turret.goalBlueRightX,
//                RobotConfig.Turret.goalBlueRightY
//        };
//    }
//
//    private double normalizeDegrees(double angle) {
//        return ((angle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
//    }
//
//    private double calculateTargetAngle(Pose robotPose, Shooter.Cell cell) {
//        double[] goal = getGoalPosition(cell);
//
//        double dx = goal[0] - robotPose.x();
//        double dy = goal[1] - robotPose.y();
//
//        double absoluteAngle = Math.atan2(dy, dx);
//        double headingDeg = Math.toDegrees(robotPose.heading());
//        double absoluteAngleDeg = Math.toDegrees(absoluteAngle);
//
//        return normalizeDegrees(absoluteAngleDeg - headingDeg);
//    }
//
//    @Override
//    public void periodic() {
//        boolean umbrellaOn = umbrellaActiveSupplier != null
//                && Boolean.TRUE.equals(umbrellaActiveSupplier.get());
//
//        double targetAngleDeg;
//
//        if (umbrellaOn) {
//            targetAngleDeg = RobotConfig.Turret.umbrellaAngleDeg;
//            manualOverride = false;
//        } else if (manualOverride) {
//            targetAngleDeg = manualAngleDeg;
//        } else if (poseSupplier != null) {
//            Pose robotPose = poseSupplier.get();
//
//            if (robotPose == null) {
//                turret.setThrottle(0);
//                return;
//            }
//
//            Shooter.Cell cell;
//
//            if (robotPose.y() < 72) {
//                cell = Shooter.Cell.RIGHT;
//            } else {
//                cell = Shooter.Cell.LEFT;
//            }
//
//            targetAngleDeg = calculateTargetAngle(robotPose, cell);
//        } else {
//            turret.setThrottle(0);
//            return;
//        }
//
//        targetAngleDeg = Math.max(
//                RobotConfig.Turret.minAngleDeg,
//                Math.min(
//                        RobotConfig.Turret.maxAngleDeg,
//                        targetAngleDeg
//                )
//        );
//
//        double currentAngleDeg =
//                turret.getEncoderPosition().into(Units.Degrees);
//
//        double errorDeg = targetAngleDeg - currentAngleDeg;
//
//        if (Math.abs(errorDeg)
//                <= RobotConfig.Turret.angleToleranceDeg) {
//            turret.setThrottle(0);
//            return;
//        }
//
//        double power = errorDeg * RobotConfig.Turret.kP;
//        power = Math.max(-1.0, Math.min(1.0, power));
//
//        turret.setThrottle(power);
//    }
//}
