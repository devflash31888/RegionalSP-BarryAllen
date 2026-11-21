package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;

import dev.easyfamily.EasyStack.control.EasyPIDFController;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotor;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorType;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;
import dev.easyfamily.EasyStack.util.EasyMathFunctions;
import dev.easyfamily.EasyStack.util.EasyInterpLUT;

public class Shooter extends EasySubsystemBase {

    private EasyMotor shooterMotor, shooterMotorB;
    private EasyPIDFController pidA, pidB;
    private Servo hood;

    private double kS = 0.1;
    private double kV = 0.00036;
    private double kP = 0.0025;

    private double targetVelocity = 0;
    private double targetHood = 0.0;
    private double distanceToGoal = 0.0;

    private EasyInterpLUT velocityTable;
    private EasyInterpLUT hoodTable;

    private static final double BANG_BANG_MARGIN = 90.0;

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

        velocityTable.add(40.0, 1275);
        velocityTable.add(50.0, 1350);
        velocityTable.add(60.0, 1375);
        velocityTable.add(70.0, 1450);

        hoodTable.add(40.0, 0.0);
        hoodTable.add(50.0, 0.1305);
        hoodTable.add(60.0, 0.18);
        hoodTable.add(70.0, 0.345);

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

        double errorA = targetVelocity - currentA;
        double errorB = targetVelocity - currentB;

        double ffPower = (kS * Math.signum(targetVelocity)) + (kV * targetVelocity);

        double powerA;
        double powerB;

        powerA = ffPower + pidA.calculate(currentA, targetVelocity);
        powerB = ffPower + pidB.calculate(currentB, targetVelocity);

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