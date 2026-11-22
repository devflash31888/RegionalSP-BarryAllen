package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Turret.angleToleranceDeg;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;
import org.firstinspires.ftc.teamcode.ConfigRegional.TuningConfig;
import dev.easyfamily.EasyStack.control.EasyPIDController;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotor;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorType;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;
import dev.easyfamily.EasyStack.util.EasyInterpLUT;
import dev.easyfamily.EasyStack.util.EasyMathFunctions;

public class Turret extends EasySubsystemBase {

    private EasyMotor turretMotor;
    private EasyPIDController pid;

    private static final double MOTOR_TPR = 537.7;
    private static final double GEAR_RATIO = 144.0 / 63.0;
    private static final double TICKS_PER_DEGREE = (MOTOR_TPR * GEAR_RATIO) / 360.0;
    private double targetDegrees = 0.0;
    private double manualOffset = 0.0;
    private static final double STEP_DEGREE = 5.0;

    private EasyInterpLUT flightTime;

    private double distanceToGoal, targetFlightTime;

    @Override
    public void init(HardwareMap hardwareMap) {
        turretMotor = new EasyMotor(hardwareMap, "turret", EasyMotorType.GOBILDA_312);
        turretMotor.setInverted(false);
        turretMotor.stopAndResetEncoder();

        pid = new EasyPIDController(
                RobotConfig.Turret.kP,
                RobotConfig.Turret.kI,
                RobotConfig.Turret.kD
        );
        pid.setTolerance(1.0);
        pid.setMaxIntegralSum(50.0);

        flightTime = new EasyInterpLUT();

        flightTime.add(40.0, 0.02);
        flightTime.add(50.0, 0.03);
        flightTime.add(60.0, 0.04);
        flightTime.add(70.0, 0.05);

        flightTime.createLUT();
    }

    private double normalizeAngleDeg(double angle) {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    public void calculateDistance(double robotX, double robotY) {
        double goalX = (robotY <= 72.0) ? RobotConfig.Field.goalRightX : RobotConfig.Field.goalLeftX;
        double goalY = (robotY <= 72.0) ? RobotConfig.Field.goalRightY : RobotConfig.Field.goalLeftY;

        distanceToGoal = Math.hypot(goalX - robotX, goalY - robotY);

        if (flightTime != null && flightTime.size() > 0) {
            targetFlightTime = flightTime.get(distanceToGoal);
        } else {
            targetFlightTime = 0.0;
        }
    }

    public void swm(double robotX, double robotY, double robotHeading, double vx, double vy) {
        double goalX;
        double goalY;

        if (robotY <= 72.0) {
            goalY = RobotConfig.Field.goalRightY;
            goalX = RobotConfig.Field.goalRightX;
        } else {
            goalY = RobotConfig.Field.goalLeftY;
            goalX = RobotConfig.Field.goalLeftX;
        }

        calculateDistance(robotX, robotY);

        double robotXFuturo = robotX + (vx * targetFlightTime);
        double robotYFuturo = robotY + (vy * targetFlightTime);

        double deltaX = goalX - robotXFuturo;
        double deltaY = goalY - robotYFuturo;

        double absoluteGoalAngleRad = Math.atan2(deltaY, deltaX);
        double relativeAngleRad = absoluteGoalAngleRad - robotHeading;
        double targetFieldDegrees = Math.toDegrees(relativeAngleRad);
        double normalizedAngle = normalizeAngleDeg(targetFieldDegrees);

        double rawTarget = normalizedAngle + manualOffset;
        targetDegrees = EasyMathFunctions.clamp(rawTarget, -90, 180);
    }

    public void setManualTarget() {
        targetDegrees = 0.0;
        manualOffset = 0.0;
    }

    public void incrementManualOffset() {
        manualOffset += STEP_DEGREE;
    }

    public void decrementManualOffset() {
        manualOffset -= STEP_DEGREE;
    }

    @Override
    public void loop() {
        if (turretMotor == null || pid == null) return;

        pid.setPID(
                TuningConfig.TurretTuningConfig.kP,
                TuningConfig.TurretTuningConfig.kI,
                TuningConfig.TurretTuningConfig.kD
        );

        double currentTicks = turretMotor.getPosition();
        double currentDegrees = currentTicks / TICKS_PER_DEGREE;

        double pidOutput = pid.calculate(currentDegrees, targetDegrees);
        double errorDegrees = pid.getError();

        double power = pidOutput;

        if (!pid.atTarget() && Math.abs(errorDegrees) > 0.5 && Math.abs(errorDegrees) < 15.0) {
            power += Math.signum(errorDegrees) * TuningConfig.TurretTuningConfig.kS;
        }

        if (Math.abs(errorDegrees) < angleToleranceDeg) {
            power = 0.0;
        }

        power = EasyMathFunctions.clamp(power, -0.65, 0.65);
        turretMotor.setPower(power);
    }

    public void stop() {
        if (turretMotor != null) {
            turretMotor.setPower(0.0);
        }
    }

    public double getTargetDegrees() {
        return targetDegrees;
    }

    public double getErrorDegrees() {
        return pid != null ? pid.getError() : 0.0;
    }

    public double getManualOffset() {
        return manualOffset;
    }
}