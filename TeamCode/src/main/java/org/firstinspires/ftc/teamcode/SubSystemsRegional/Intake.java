package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Intake.def;
import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Intake.intakePower;
import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Intake.reversePower;
import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Led.off;
import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Led.on;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.easyfamily.EasyStack.hardware.Motors.EasyMotor;
import dev.easyfamily.EasyStack.hardware.Servo.EasyPwmPresets;
import dev.easyfamily.EasyStack.hardware.Servo.EasyServo;
import dev.easyfamily.EasyStack.subsytem.EasyStateSubsystem;
import dev.easyfamily.EasyStack.util.EasyDebouncer;

public class Intake extends EasyStateSubsystem<Intake.Mode> {

    public enum Mode {OFF, IN, OUT;}

    private EasyMotor intake;
    private EasyServo led;
    private DistanceSensor distanceSensor;
    private EasyDebouncer pieceDebouncer;
    private boolean pieceDetected = false;

    public Intake() {
        super(Mode.OFF);
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        intake = new EasyMotor(hardwareMap, "intake").reversed();
        intake.setPower(def);

        led = new EasyServo(hardwareMap, "led")
                .addState("OFF", off)
                .addState("ON", on);
        led.setState("OFF");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        pieceDebouncer = new EasyDebouncer(1000, EasyDebouncer.Type.RISING);
    }

    @Override
    public void loop() {
        double currentDistance = distanceSensor.getDistance(DistanceUnit.MM);
        boolean rawDetection = (currentDistance >= 6.35 && currentDistance <= 7.5);

        pieceDetected = pieceDebouncer.calculate(rawDetection);

        if(pieceDetected) {
            led.setState("ON");
        } else {
            led.setState("OFF");
        }
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }

    public void turnOn() {
        intake.setPower(intakePower);
    }

    public void turnOff() {
        intake.setPower(def);
    }

    public void turnReverse() {
        intake.setPower(reversePower);
    }

    public boolean hasPiece() {
        return pieceDetected;
    }
}