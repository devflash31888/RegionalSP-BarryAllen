package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;

import dev.easyfamily.EasyStack.control.EasyPIDFController;
import dev.easyfamily.EasyStack.dashboard.EasyTunable;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotor;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorGroup;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorType;
import dev.easyfamily.EasyStack.hardware.Servo.EasyServo;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;
import dev.easyfamily.EasyStack.util.EasyMathFunctions;
import dev.easyfamily.EasyStack.util.EasyInterpLUT;

public class Shooter extends EasySubsystemBase {

    private EasyMotor shooterMotor, shooterMotorB;
    private EasyMotorGroup shooter_motors;
    private EasyServo hood;

    @EasyTunable
    public static double rpm = 0; // velocidade para alterar em tempo real

    @EasyTunable
    public static double hoodPosition = 0; // Posição do capuz em tempo real

    @EasyTunable
    public static double kS = 0.06;

    @EasyTunable
    public static double kV = 0.00021;

    @EasyTunable
    public static double kP = 0.0015;

    private double targetHood = 0.0;
    private double distanceToGoal = 0.0;

    private EasyInterpLUT velocityTable;
    private EasyInterpLUT hoodTable;

    public void configurePIDF() {
        shooter_motors
                .setVelocityFF(kS, kV)
                .setVelocityPIDF(kP, 0, 0, 0);
    }

    public void lut() {
        velocityTable = new EasyInterpLUT();
        hoodTable = new EasyInterpLUT();

        velocityTable.add(40.0, 2500);
        velocityTable.add(50.0, 2500);
        velocityTable.add(60.0, 2750);
        velocityTable.add(70.0, 2750);

        hoodTable.add(40.0, 0.0);
        hoodTable.add(50.0, 0.15);
        hoodTable.add(60.0, 0.21);
        hoodTable.add(70.0, 0.3);

        velocityTable.createLUT();
        hoodTable.createLUT();
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        shooterMotor = new EasyMotor(hardwareMap, "shooterA", EasyMotorType.REV_HD_NAKED);
        shooterMotorB = new EasyMotor(hardwareMap, "shooterB", EasyMotorType.REV_HD_NAKED).setInverted(true);

        shooter_motors = new EasyMotorGroup(shooterMotor, shooterMotorB)
                .setLeaderMotor(0);

        hood = new EasyServo(hardwareMap, "hood");

        lut();
    }

    public void calculateDistanceAndInterpolate(double robotX, double robotY) {
        double goalX = (robotY <= 72.0) ? RobotConfig.Field.goalRightX : RobotConfig.Field.goalLeftX;
        double goalY = (robotY <= 72.0) ? RobotConfig.Field.goalRightY : RobotConfig.Field.goalLeftY;

        distanceToGoal = Math.hypot(goalX - robotX, goalY - robotY);

        if (velocityTable != null && velocityTable.size() > 0) {
            rpm = velocityTable.get(distanceToGoal);
        }
        if (hoodTable != null && hoodTable.size() > 0) {
            targetHood = EasyMathFunctions.clamp(hoodTable.get(distanceToGoal), 0.0, 1.0);
        }
    }

    @Override
    public void loop() {
        if (shooterMotor == null || shooterMotorB == null) return;

        if (rpm <= 0.0) {
            shooter_motors.setPower(0);
            shooter_motors.resetControllers();
            if (hood != null) hood.setPosition(targetHood);
            return;
        }

        configurePIDF();

        shooter_motors.setRPM(rpm);

        if (hood != null) {
            hood.setPosition(hoodPosition);
        }

    }

    public void stop() {
        rpm = 0.0;
        if (shooter_motors != null) shooter_motors.setPower(0.0);
    }

    public double getCurrentVelocity() {
        return shooter_motors != null ? shooter_motors.getRPM() : 0.0;
    }

    public double getTargetVelocity() {
        return rpm;
    }

    public double getTargetHood() {
        return targetHood;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public void setDefault() {
        rpm = 2500;
        targetHood = 0.0;
    }

    public void tuningLUT() {
        shooter_motors.setRPM(rpm);
        hood.setPosition(hoodPosition);
    }

    public void calculateDistance(double robotX, double robotY) {
        double goalX = (robotY <= 72.0) ? RobotConfig.Field.goalRightX : RobotConfig.Field.goalLeftX;
        double goalY = (robotY <= 72.0) ? RobotConfig.Field.goalRightY : RobotConfig.Field.goalLeftY;

        distanceToGoal = Math.hypot(goalX - robotX, goalY - robotY);
    }

}