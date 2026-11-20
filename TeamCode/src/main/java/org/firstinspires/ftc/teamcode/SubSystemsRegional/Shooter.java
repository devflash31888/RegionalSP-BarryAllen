package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;

import dev.easyfamily.EasyStack.control.EasyPIDFController;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotor;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorType;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;
import dev.easyfamily.EasyStack.util.EasyMathFunctions;
import dev.easyfamily.EasyStack.util.EasyInterpLUT; // Certifique-se do import correto da sua lib

public class Shooter extends EasySubsystemBase {

    private EasyMotor shooterMotor, shooterMotorB;
    private EasyPIDFController pidA, pidB;
    private Servo hood;

    private double kS = 0.025;
    private double kV = 0.00042;
    private double kP = 0.0035;

    private double targetVelocity = 0;
    private double targetHood = 0.0;
    private double distanceToGoal = 0.0;

    private EasyInterpLUT velocityTable;
    private EasyInterpLUT hoodTable;

    @Override
    public void init(HardwareMap hardwareMap) {
        shooterMotor = new EasyMotor(hardwareMap, "shooterA", EasyMotorType.REV_HD_NAKED);
        shooterMotorB = new EasyMotor(hardwareMap, "shooterB", EasyMotorType.REV_HD_NAKED);
        shooterMotorB.setInverted(true);

        hood = hardwareMap.get(Servo.class, "hood");

        pidA = new EasyPIDFController(kP, 0.0, 0.0, 0.0);
        pidB = new EasyPIDFController(kP, 0.0, 0.0, 0.0);

        velocityTable = new EasyInterpLUT();
        hoodTable = new EasyInterpLUT();

        velocityTable.add(100.0, 1100.0);
        velocityTable.add(200.0, 1400.0);
        velocityTable.add(300.0, 1700.0);

        hoodTable.add(100.0, 0.0);
        hoodTable.add(200.0, 0.3);
        hoodTable.add(300.0, 0.6);

        velocityTable.createLUT();
        hoodTable.createLUT();
    }

    public void calculateDistanceAndInterpolate(double robotX, double robotY) {
        double goalX = (robotY <= 72.0) ? RobotConfig.Field.goalRightX : RobotConfig.Field.goalLeftX;
        double goalY = (robotY <= 72.0) ? RobotConfig.Field.goalRightY : RobotConfig.Field.goalLeftY;

        distanceToGoal = Math.hypot(goalX - robotX, goalY - robotY);

        if (velocityTable != null && velocityTable.size() > 0) {
            targetVelocity = velocityTable.get(distanceToGoal);
        }
        if (hoodTable != null && hoodTable.size() > 0) {
            targetHood = EasyMathFunctions.clamp(hoodTable.get(distanceToGoal), 0.0, 1.0);
        }
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

    public void setDefault() {
        targetVelocity = 1250;
        targetHood = 0.0;
    }
}