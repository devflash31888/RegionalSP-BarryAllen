package org.firstinspires.ftc.teamcode.OpModesRegional.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystemsRegional.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.FlowerRemover;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Gate;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Intake;

import dev.easyfamily.EasyStack.components.EasyCommandComponent;
import dev.easyfamily.EasyStack.components.EasySubsystemComponent;
import dev.easyfamily.EasyStack.gamepad.EasyGamepad;
import dev.easyfamily.EasyStack.opmode.EasyOpMode;

@TeleOp(name = "Teleop Regional")
public class TeleOpRegional extends EasyOpMode {

    private final Intake intake = new Intake();
    private final Gate gate = new Gate();
    private final DriveTrain drivetrain = new DriveTrain();
    private final FlowerRemover flower = new FlowerRemover();
    private EasyGamepad score, nav;

    @Override
    public void initialize() {
        nav = new EasyGamepad(gamepad1);
        score = new EasyGamepad(gamepad2);

        addComponents(
                new EasySubsystemComponent(intake),
                new EasySubsystemComponent(gate),
                new EasySubsystemComponent(drivetrain),
                new EasySubsystemComponent(flower),
                new EasyCommandComponent()
        );
    }

    @Override
    public void onUpdate() {
        nav.update();
        score.update();

        drivetrain.teleopDrive(nav);

        if(nav.xPressed()) {
            drivetrain.resetHeading();
        }

        if(nav.aPressed()) {
            drivetrain.relocalize();
        }

        if (score.x()) {
            intake.reverse();
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

        if(score.bToggle()) {
            flower.down();
        } else {
            flower.up();
        }
    }

    @Override
    public void onStop() {
        intake.turnOff();
        gate.close();
    }

}
