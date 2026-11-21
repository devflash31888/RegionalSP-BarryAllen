package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;

import dev.easyfamily.EasyStack.control.EasyPIDFController;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotor;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorType;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;
import dev.easyfamily.EasyStack.util.EasyMathFunctions;

public class Shooter extends EasySubsystemBase {

    private EasyMotor shooterMotor, shooterMotorB;
    private EasyPIDFController pidA, pidB;
    private Servo hood;

    private final double kS = 0.025;
    private final double kV = 0.00042;
    private final double kP = 0.0025;

    private double targetVelocity = 1100.0;
    private double targetHood = 0.0;
    private double distanceToGoal = 0.0;

    private static final double VEL_STEP = 25.0;
    private static final double HOOD_STEP = 0.015;

    @Override
    public void init(HardwareMap hardwareMap) {
        shooterMotor = new EasyMotor(hardwareMap, "shooterA", EasyMotorType.REV_HD_NAKED);
        shooterMotorB = new EasyMotor(hardwareMap, "shooterB", EasyMotorType.REV_HD_NAKED);
        shooterMotorB.setInverted(true);

        hood = hardwareMap.get(Servo.class, "hood");

        pidA = new EasyPIDFController(kP, 0.0, 0.0, 0.0);
        pidB = new EasyPIDFController(kP, 0.0, 0.0, 0.0);
    }

    // Método para calcular a distância e atualizar com base na posição do Drivetrain
    public void calculateDistance(double robotX, double robotY) {
        double goalX = (robotY <= 72.0) ? RobotConfig.Field.goalRightX : RobotConfig.Field.goalLeftX;
        double goalY = (robotY <= 72.0) ? RobotConfig.Field.goalRightY : RobotConfig.Field.goalLeftY;

        distanceToGoal = Math.hypot(goalX - robotX, goalY - robotY);
    }

    public void incrementVelocity() {
        targetVelocity += VEL_STEP;
    }

    public void decrementVelocity() {
        targetVelocity = Math.max(0, targetVelocity - VEL_STEP);
    }

    public void incrementHood() {
        targetHood = EasyMathFunctions.clamp(targetHood + HOOD_STEP, 0.0, 1.0);
    }

    public void decrementHood() {
        targetHood = EasyMathFunctions.clamp(targetHood - HOOD_STEP, 0.0, 1.0);
    }

    @Override
    public void loop() {
        if (shooterMotor == null || shooterMotorB == null || pidA == null || pidB == null) return;

        if (targetVelocity <= 0.0) {
            shooterMotor.setPower(0.0);
            shooterMotorB.setPower(0.0);
            pidA.reset();
            pidB.reset();
            if (hood != null) hood.setPosition(targetHood);
            return;
        }

        double currentA = shooterMotor.getVelocity();
        double currentB = shooterMotorB.getVelocity();

        double ffPower = (kS * Math.signum(targetVelocity)) + (kV * targetVelocity);

        double powerA = ffPower + pidA.calculate(currentA, targetVelocity);
        double powerB = ffPower + pidB.calculate(currentB, targetVelocity);

        shooterMotor.setPower(EasyMathFunctions.clamp(powerA, 0.0, 1.0));
        shooterMotorB.setPower(EasyMathFunctions.clamp(powerB, 0.0, 1.0));

        if (hood != null) {
            hood.setPosition(targetHood);
        }
    }

    public void stop() {
        targetVelocity = 0.0;
        if (shooterMotor != null) shooterMotor.setPower(0.0);
        if (shooterMotorB != null) shooterMotorB.setPower(0.0);
    }

    public double getCurrentVelocityA() {
        return shooterMotor != null ? shooterMotor.getVelocity() : 0.0;
    }

    public double getCurrentVelocityB() {
        return shooterMotorB != null ? shooterMotorB.getVelocity() : 0.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetHood() {
        return targetHood;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }
}