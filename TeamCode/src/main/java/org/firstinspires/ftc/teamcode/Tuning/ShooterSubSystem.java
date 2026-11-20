package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.easyfamily.EasyStack.control.EasyPIDFController;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotor;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorType;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;

public class ShooterSubSystem extends EasySubsystemBase {

    private EasyMotor shooterMotor, shooterMotorB;
    private EasyPIDFController pidA, pidB;

    private double kS = 0.0;
    private double kV = 0.0;
    private double targetVelocity = 0.0;

    @Override
    public void init(HardwareMap hardwareMap) {
        shooterMotor = new EasyMotor(hardwareMap, "shooterA", EasyMotorType.REV_HD_NAKED);
        shooterMotorB = new EasyMotor(hardwareMap, "shooterB", EasyMotorType.REV_HD_NAKED);
        shooterMotorB.setInverted(true);

        pidA = new EasyPIDFController(0.0, 0.0, 0.0, 0.0);
        pidB = new EasyPIDFController(0.0, 0.0, 0.0, 0.0);
    }

    public void setGains(double kP, double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
        if (pidA != null && pidB != null) {
            pidA.setPIDF(kP, 0.0, 0.0, 0.0);
            pidB.setPIDF(kP, 0.0, 0.0, 0.0);
        }
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    @Override
    public void loop() {
        if (shooterMotor == null || shooterMotorB == null || pidA == null) return;

        if (targetVelocity <= 0.0) {
            shooterMotor.setPower(0.0);
            shooterMotorB.setPower(0.0);
            pidA.reset();
            pidB.reset();
            return;
        }

        double currentA = shooterMotor.getVelocity();
        double currentB = shooterMotorB.getVelocity();

        double ffPower = (kS * Math.signum(targetVelocity)) + (kV * targetVelocity);

        double powerA = ffPower + pidA.calculate(currentA, targetVelocity);
        double powerB = ffPower + pidB.calculate(currentB, targetVelocity);

        shooterMotor.setPower(powerA);
        shooterMotorB.setPower(powerB);
    }

    public void stop() {
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
}