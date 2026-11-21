package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;
import org.firstinspires.ftc.teamcode.ConfigRegional.TuningConfig;
import dev.easyfamily.EasyStack.control.EasyPIDController;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotor;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorType;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;
import dev.easyfamily.EasyStack.util.EasyMathFunctions;

public class Turret extends EasySubsystemBase {

    private EasyMotor turretMotor;
    private EasyPIDController pid;

    private static final double MOTOR_TPR = 537.7;
    private static final double GEAR_RATIO = 144.0 / 62.0;
    private static final double TICKS_PER_DEGREE = (MOTOR_TPR * GEAR_RATIO) / 360.0;
    private double targetDegrees = 0.0;
    private double manualOffset = 0.0;
    private static final double STEP_DEGREE = 5.0;

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
    }

    private double normalizeAngleDeg(double angle) {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    public void aimAtGoal(double robotX, double robotY, double robotHeading) {
        double goalX;
        double goalY;

        if (robotY <= 72.0) {
            goalY = RobotConfig.Field.goalRightY;
            goalX = RobotConfig.Field.goalRightX;
        } else {
            goalY = RobotConfig.Field.goalLeftY;
            goalX = RobotConfig.Field.goalLeftX;
        }
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

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

        if (Math.abs(errorDegrees) < 0.3) {
            power = 0.0;
        }

        power = EasyMathFunctions.clamp(power, -0.3, 0.3);
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