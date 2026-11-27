package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;

import dev.easyfamily.EasyStack.dashboard.EasyTunable;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotor;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorGroup;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorType;
import dev.easyfamily.EasyStack.hardware.Servo.EasyServo;
import dev.easyfamily.EasyStack.subsytem.EasySubsystemBase;
import dev.easyfamily.EasyStack.util.EasyInterpLUT;
import dev.easyfamily.EasyStack.util.EasyMathFunctions;

public class Shooter extends EasySubsystemBase {

    private EasyMotor shooterMotor, shooterMotorB;
    private EasyMotorGroup shooter_motors;
    private EasyServo hood;
    private VoltageSensor batteryVoltage;

    @EasyTunable
    public static double rpm = 0;

    @EasyTunable
    public static double hoodPosition = 0;

    @EasyTunable
    public static double kS = 0.06;

    @EasyTunable
    public static double kV = 0.00018;

    @EasyTunable
    public static double kP = 0.00075;

    @EasyTunable
    public static double voltageCompGain = 1.0;

    @EasyTunable
    public static double bangbang = 800;

    @EasyTunable
    public static double bangbangExit = 600;

    @EasyTunable
    public static double readyTolerance = 100;

    private static final double BOOST_RPM = 5000;
    private static final double HOOD_STOWED = 0.0;
    private static final double READY_TIME_S = 0.15;
    private static final double CALIBRATION_VOLTAGE = 13.0;
    private static final double MIN_VOLTAGE = 9.0;
    private static final double MAX_VOLTAGE_SCALE = 1.35;
    private static final double VOLTAGE_REFRESH_S = 0.25;
    private static final double VOLTAGE_FILTER_ALPHA = 0.15;
    private static final double VOLTAGE_DEADBAND = 0.15;
    private static final double RPM_FILTER_ALPHA = 0.35;

    private static final double[] LUT_DISTANCE = {40.0, 50.0, 60.0, 70.0};
    private static final double[] LUT_RPM      = {2450, 2650, 2750, 2850};
    private static final double[] LUT_HOOD     = {0.0, 0.2, 0.23, 0.27};

    private boolean initialized = false;
    private boolean boosting = false;

    private double currentVoltage = CALIBRATION_VOLTAGE;
    private double filteredVoltage = CALIBRATION_VOLTAGE;
    private double filteredRpm = 0.0;
    private double commandedHood = HOOD_STOWED;
    private double distanceToGoal = 0.0;
    private double lastRpm = 0.0;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private double lastAppliedKs = Double.NaN;
    private double lastAppliedKv = Double.NaN;
    private double lastAppliedKp = Double.NaN;

    private EasyInterpLUT velocityTable;
    private EasyInterpLUT hoodTable;

    private void buildLut() {
        velocityTable = new EasyInterpLUT();
        hoodTable = new EasyInterpLUT();

        for (int i = 0; i < LUT_DISTANCE.length; i++) {
            velocityTable.add(LUT_DISTANCE[i], LUT_RPM[i]);
            hoodTable.add(LUT_DISTANCE[i], LUT_HOOD[i]);
        }

        velocityTable.createLUT();
        hoodTable.createLUT();
    }

    public void lut() {
        buildLut();
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        shooterMotor = new EasyMotor(hardwareMap, "shooterA", EasyMotorType.REV_HD_NAKED);
        shooterMotorB = new EasyMotor(hardwareMap, "shooterB", EasyMotorType.REV_HD_NAKED).setInverted(true);

        shooter_motors = new EasyMotorGroup(shooterMotor, shooterMotorB)
                .setLeaderMotor(0);

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = readVoltage();
        filteredVoltage = currentVoltage;

        hood = new EasyServo(hardwareMap, "hood");

        buildLut();

        voltageTimer.reset();
        readyTimer.reset();
        initialized = true;
    }

    private void updateDistance(double robotX, double robotY) {
        double goalX = (robotY <= 72.0) ? RobotConfig.Field.goalRightX : RobotConfig.Field.goalLeftX;
        double goalY = (robotY <= 72.0) ? RobotConfig.Field.goalRightY : RobotConfig.Field.goalLeftY;

        distanceToGoal = Math.hypot(goalX - robotX, goalY - robotY);
    }

    public void calculateDistance(double robotX, double robotY) {
        updateDistance(robotX, robotY);
    }

    public void calculateDistanceAndInterpolate(double robotX, double robotY) {
        updateDistance(robotX, robotY);

        double d = EasyMathFunctions.clamp(
                distanceToGoal,
                LUT_DISTANCE[0],
                LUT_DISTANCE[LUT_DISTANCE.length - 1]);

        if (velocityTable != null) {
            rpm = velocityTable.get(d);
        }
        if (hoodTable != null) {
            hoodPosition = EasyMathFunctions.clamp(hoodTable.get(d), 0.0, 1.0);
        }
    }

    @Override
    public void loop() {
        if (!initialized) return;

        if (rpm <= 0.0) updateVoltage();

        if (rpm <= 0.0) {
            shooter_motors.setPower(0);
            shooter_motors.resetControllers();
            invalidateGainCache();
            boosting = false;
            lastRpm = 0.0;
            readyTimer.reset();
            commandedHood = HOOD_STOWED;
            hood.setPosition(commandedHood);
            return;
        }

        if (lastRpm <= 0.0) {
            shooter_motors.resetControllers();
            invalidateGainCache();
            filteredRpm = shooter_motors.getRPM();
            readyTimer.reset();
        }
        lastRpm = rpm;

        double rawScale = EasyMathFunctions.clamp(
                CALIBRATION_VOLTAGE / Math.max(currentVoltage, MIN_VOLTAGE),
                1.0 / MAX_VOLTAGE_SCALE,
                MAX_VOLTAGE_SCALE);
        double gain = EasyMathFunctions.clamp(voltageCompGain, 0.0, 1.0);
        double compensatedKv = kV * (1.0 + gain * (rawScale - 1.0));
        applyGains(kS, compensatedKv, kP);

        double rawRpm = shooter_motors.getRPM();
        filteredRpm += RPM_FILTER_ALPHA * (rawRpm - filteredRpm);
        double error = rpm - filteredRpm;

        if (!boosting && error > bangbang) {
            boosting = true;
        } else if (boosting && error < Math.min(bangbangExit, bangbang)) {
            boosting = false;
        }

        shooter_motors.setRPM(boosting ? BOOST_RPM : rpm);

        if (Math.abs(error) > readyTolerance) {
            readyTimer.reset();
        }

        commandedHood = EasyMathFunctions.clamp(hoodPosition, 0.0, 1.0);
        hood.setPosition(commandedHood);
    }

    private void applyGains(double ks, double kv, double kp) {
        if (ks != lastAppliedKs || kv != lastAppliedKv) {
            shooter_motors.setVelocityFF(ks, kv);
            lastAppliedKs = ks;
            lastAppliedKv = kv;
        }
        if (kp != lastAppliedKp) {
            shooter_motors.setVelocityPIDF(kp, 0, 0, 0);
            lastAppliedKp = kp;
        }
    }

    private void invalidateGainCache() {
        lastAppliedKs = Double.NaN;
        lastAppliedKv = Double.NaN;
        lastAppliedKp = Double.NaN;
    }

    private void updateVoltage() {
        if (voltageTimer.seconds() >= VOLTAGE_REFRESH_S) {
            double raw = readVoltage();
            filteredVoltage += VOLTAGE_FILTER_ALPHA * (raw - filteredVoltage);
            if (Math.abs(filteredVoltage - currentVoltage) > VOLTAGE_DEADBAND) {
                currentVoltage = filteredVoltage;
            }
            voltageTimer.reset();
        }
    }

    private double readVoltage() {
        double v = batteryVoltage != null ? batteryVoltage.getVoltage() : CALIBRATION_VOLTAGE;
        return (v > 0) ? v : CALIBRATION_VOLTAGE;
    }

    public void stop() {
        rpm = 0.0;
        if (shooter_motors != null) {
            shooter_motors.setPower(0.0);
            shooter_motors.resetControllers();
            invalidateGainCache();
        }
        boosting = false;
    }

    public void setDefault() {
        rpm = 2500;
        hoodPosition = 0.0;
    }

    public boolean isReady() {
        return initialized
                && rpm > 0.0
                && !boosting
                && readyTimer.seconds() >= READY_TIME_S;
    }

    public boolean isBoosting() {
        return boosting;
    }

    public double getCurrentVelocity() {
        return shooter_motors != null ? shooter_motors.getRPM() : 0.0;
    }

    public double getFilteredVelocity() {
        return filteredRpm;
    }

    public double getError() {
        return rpm - filteredRpm;
    }

    public double getTargetVelocity() {
        return rpm;
    }

    public double getTargetHood() {
        return commandedHood;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public double getBatteryVoltage() {
        return currentVoltage;
    }

    public void tuningLUT() {
    }
}
