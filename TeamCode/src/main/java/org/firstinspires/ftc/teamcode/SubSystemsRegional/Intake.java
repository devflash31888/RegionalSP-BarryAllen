package org.firstinspires.ftc.teamcode.SubSystemsRegional;

import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Intake.def;
import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Intake.intakePower;
import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Intake.reversePower;
import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Led.off;
import static org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig.Led.on;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.easyfamily.EasyStack.controllable.EasyDistanceSensor;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotor;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorType;
import dev.easyfamily.EasyStack.hardware.Sensors.EasyDigitalDevice;
import dev.easyfamily.EasyStack.hardware.Sensors.EasyRevColorSensorV3;
import dev.easyfamily.EasyStack.hardware.Servo.EasyServo;
import dev.easyfamily.EasyStack.hardware.Servo.PwmPresets;
import dev.easyfamily.EasyStack.subsytem.EasyStateSubsystem;
import dev.easyfamily.EasyStack.subsytem.EasySubsystem;
import dev.easyfamily.EasyStack.util.Debouncer;

public class Intake extends EasyStateSubsystem<Intake.Mode> {

    public enum Mode {OFF, IN, OUT;}

    private EasyMotor intake;
    private EasyServo led;
    private EasyRevColorSensorV3 distanceSensor;
    private EasyDistanceSensor.DistanceTarget pieceDetectedTarget;
    private Debouncer pieceDebouncer;

    private boolean pieceDetected = false;

    public Intake() {
        super(Mode.OFF);
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        intake = new EasyMotor(hardwareMap, "intake").reversed();
        intake.setPower(def);

        led = new EasyServo(hardwareMap, "led")
                .setPwmRange(PwmPresets.FULL_RANGE)
                .addState("OFF", off)
                .addState("ON", on);
        led.setState("OFF");

        distanceSensor = new EasyRevColorSensorV3(hardwareMap, "distanceSensor");
        pieceDetectedTarget = new EasyDistanceSensor.DistanceTarget(DistanceUnit.MM, 0, 40, "PieceDetected");
        distanceSensor.addTarget(pieceDetectedTarget);

        pieceDebouncer = new Debouncer(500, Debouncer.Type.RISING);
    }

    @Override
    public void loop() {
        boolean rawDetection = distanceSensor.targetReached(pieceDetectedTarget);
        pieceDetected = pieceDebouncer.calculate(rawDetection);

        if(pieceDetected) {
            led.setState("ON");
        } else {
            led.setState("OFF");
        }
    }

    public void turnOn() {
        intake.setPower(intakePower);
    }

    public void turnOff() {
        intake.setPower(def);
    }

    public void reverse() {
        intake.setPower(reversePower);
    }

    public boolean hasPiece() {
        return pieceDetected;
    }
}
