package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystemsRegional.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.FlowerRemover;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Gate;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Intake;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Shooter;

import dev.easyfamily.EasyStack.components.EasyCommandComponent;
import dev.easyfamily.EasyStack.components.EasySubsystemComponent;
import dev.easyfamily.EasyStack.dashboard.EasyDash;
import dev.easyfamily.EasyStack.gamepad.EasyGamepad;
import dev.easyfamily.EasyStack.license.EasyLicense;
import dev.easyfamily.EasyStack.opmode.EasyOpMode;

@TeleOp(name = "Teleop Regional Shooter Tuning")
public class TeleopTuningShooter extends EasyOpMode {

    private final Intake intake = new Intake();
    private final Gate gate = new Gate();
    private final DriveTrain drivetrain = new DriveTrain();
    private final FlowerRemover flower = new FlowerRemover();
    private final Shooter shooter = new Shooter();
    private EasyGamepad score, nav;

    @Override
    public void initialize() {
        EasyLicense.setKey("ES1-eyJ2IjoxLCJ0ZWFtIjozMTg4OCwiaWF0IjoxNzk1MTkyMDY4LCJleHAiOjE4MDI5NjgwNjh9.MEUCIFno9EWlAZ3uT2o8OEpbicP1u1cJC2QqlGv5xDhNbkf3AiEA_eiNxQT8dv4xm2adHsWuc-m75HMWCJp5bzjdBwx75Pc");
        nav = new EasyGamepad(gamepad1);
        score = new EasyGamepad(gamepad2);

        addComponents(
                new EasySubsystemComponent(intake),
                new EasySubsystemComponent(gate),
                new EasySubsystemComponent(drivetrain),
                new EasySubsystemComponent(flower),
                new EasySubsystemComponent(shooter),
                new EasyCommandComponent()
        );
    }

    @Override
    public void onUpdate() {
        nav.update();
        score.update();

        drivetrain.teleopDrive(nav);

        if (nav.xPressed()) {
            drivetrain.resetHeading();
        }

        if (nav.aPressed()) {
            drivetrain.relocalize();
        }

        if (score.x()) {
            intake.turnReverse();
        } else if (score.a()) {
            gate.open();

            if (gate.hasWaitTimeElapsed()) {
                intake.turnOn();
            }
        } else {
            gate.close();

            if (score.yToggle()) {
                intake.turnOn();
            } else {
                intake.turnOff();
            }
        }

        if (score.bToggle()) {
            flower.down();
        } else {
            flower.up();
        }

        EasyDash.tunables(Shooter.class);

        telemetry.addData("Posição X", drivetrain.getPose().x());
        telemetry.addData("Posição Y", drivetrain.getPose().y());
        telemetry.addData("Heading", drivetrain.getPose().heading());
        telemetry.addLine("--- SHOOTER TUNING ---");
        telemetry.addData("Shooter Alvo (RPM)", shooter.getTargetVelocity());
        telemetry.addData("Shooter Atual (RPM)",  shooter.getCurrentVelocity());
        telemetry.update();
    }

    @Override
    public void onStop() {
        intake.turnOff();
        gate.close();
        shooter.stop();
    }
}