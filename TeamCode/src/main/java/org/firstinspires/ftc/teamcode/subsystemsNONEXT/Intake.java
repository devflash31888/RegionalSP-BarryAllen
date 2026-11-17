package org.firstinspires.ftc.teamcode.subsystemsNONEXT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ConfigRegional.RobotConfig;

public class Intake {

    private enum Mode { OFF, IN, OUT }

    private DcMotorEx motor;
    private Mode mode = Mode.OFF;

    public void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mode = Mode.OFF;
    }

    public void intake()  { mode = Mode.IN; }
    public void reverse() { mode = Mode.OUT; }
    public void stop()    { mode = Mode.OFF; }

    public boolean isRunning() { return mode != Mode.OFF; }

    public void runIntake()  { intake(); }
    public void runReverse() { reverse(); }
    public void stopIntake() { stop(); }

    public void update() {
        if (motor == null) return;

        switch (mode) {
            case IN:
                motor.setPower(RobotConfig.Intake.intakePower);
                break;
            case OUT:
                motor.setPower(RobotConfig.Intake.reversePower);
                break;
            default:
                motor.setPower(0.0);
                break;
        }
    }
}