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
    private static final double GEAR_RATIO = 2.32;
    private static final double TICKS_PER_DEGREE = (MOTOR_TPR * GEAR_RATIO) / 360.0;
    private double targetDegrees = 0.0;

    @Override
    public void init(HardwareMap hardwareMap) {
        turretMotor = new EasyMotor(hardwareMap, "turret", EasyMotorType.GOBILDA_312);
        turretMotor.setCurrentPositionAsZero();

        pid = new EasyPIDController(
                RobotConfig.Turret.kP,
                RobotConfig.Turret.kI,
                RobotConfig.Turret.kD
        );
        pid.setTolerance(1.0);
        pid.setMaxIntegralSum(50.0);
    }

    public void aimAtGoal(double robotX, double robotY, double robotHeading) {
        double goalX, goalY;

        if (robotY <= 72.0) {
            goalX = RobotConfig.Field.goalRightX;
            goalY = RobotConfig.Field.goalRightY;
        } else {
            goalX = RobotConfig.Field.goalLeftX;
            goalY = RobotConfig.Field.goalLeftY;
        }

        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        double absoluteGoalAngleRad = Math.atan2(deltaY, deltaX);
        double relativeAngleRad = absoluteGoalAngleRad - robotHeading;

        double targetFieldDegrees = Math.toDegrees(relativeAngleRad);

        targetDegrees = EasyMathFunctions.clamp(targetFieldDegrees, -90, 180);
    }

    public void setManualTarget() {
        targetDegrees = 0.0;
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

        power = dev.easyfamily.EasyStack.util.EasyMathFunctions.clamp(power, -0.35, 0.35);
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
}