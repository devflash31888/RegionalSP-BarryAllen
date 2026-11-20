package org.firstinspires.ftc.teamcode.OpModesRegional.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystemsRegional.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.FlowerRemover;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Gate;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Intake;
import org.firstinspires.ftc.teamcode.SubSystemsRegional.Turret;

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
    private final Turret turret = new Turret();
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
                new EasySubsystemComponent(turret),
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

        if (nav.leftBumperToggle()) {
            turret.setManualTarget();
        } else {
            double robotX = drivetrain.getPose().x();
            double robotY = drivetrain.getPose().y();
            double robotHeading = drivetrain.getHeading();

            turret.aimAtGoal(robotX, robotY, robotHeading);
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

        if(score.bToggle()) {
            flower.down();
        } else {
            flower.up();
        }

        telemetry.addData("Turret Alvo", turret.getTargetDegrees());
        telemetry.addData("Turret Erro", turret.getErrorDegrees());
        telemetry.addData("Posição X", drivetrain.getPose().x());
        telemetry.addData("Posição Y", drivetrain.getPose().y());
        telemetry.addData("Posição Z", drivetrain.getPose().heading());
        telemetry.update();

    }

    @Override
    public void onStop() {
        intake.turnOff();
        gate.close();
        turret.stop();
    }

}
