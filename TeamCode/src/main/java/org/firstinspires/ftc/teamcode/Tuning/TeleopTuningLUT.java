package org.firstinspires.ftc.teamcode.Tuning;

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

@TeleOp(name = "TeleopTuningLUT")
public class TeleopTuningLUT extends EasyOpMode {

    private final Intake intake = new Intake();
    private final Gate gate = new Gate();
    private final DriveTrain drivetrain = new DriveTrain();
    private final FlowerRemover flower = new FlowerRemover();
    private final Turret turret = new Turret();
    private final Shooter shooter = new Shooter();

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

        double robotX = drivetrain.getPose().x();
        double robotY = drivetrain.getPose().y();
        double robotHeading = drivetrain.getHeading();

        // Atualiza o cálculo da distância direto no subsystem do Shooter
        shooter.calculateDistance(robotX, robotY);

        // Controle da Torre
        if (nav.leftBumperToggle()) {
            turret.setManualTarget();
        } else {

            turret.aimAtGoal(robotX, robotY, robotHeading);

            if (score.rightBumperPressed()) {
                turret.incrementManualOffset();
            } else if (score.leftBumperPressed()) {
                turret.decrementManualOffset();
            }
        }

        // Calibração do Shooter e Hood via D-Pad do Gamepad 1
        if (score.dpadUpPressed()) {
            shooter.incrementVelocity();
        } else if (score.dpadDownPressed()) {
            shooter.decrementVelocity();
        }

        if (score.dpadRightPressed()) {
            shooter.incrementHood();
        } else if (score.dpadLeftPressed()) {
            shooter.decrementHood();
        }

        // Intake e Gate
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

        // Telemetria puxando os dados direto do Shooter
        telemetry.addData("Distância Goal (cm)", shooter.getDistanceToGoal());
        telemetry.addData("Shooter Alvo (Vel)", shooter.getTargetVelocity());
        telemetry.addData("Shooter Vel A", shooter.getCurrentVelocityA());
        telemetry.addData("Shooter Vel B", shooter.getCurrentVelocityB());
        telemetry.addData("Hood Alvo (Pos)", shooter.getTargetHood());
        telemetry.addData("Turret Alvo", turret.getTargetDegrees());
        telemetry.addData("Turret Erro", turret.getErrorDegrees());
        telemetry.addData("Turret Offset Manual", turret.getManualOffset());
        telemetry.addData("Posição X", robotX);
        telemetry.addData("Posição Y", robotY);
        telemetry.addData("Posição Z", robotHeading);
        telemetry.update();
    }

    @Override
    public void onStop() {
        intake.turnOff();
        gate.close();
        turret.stop();
        shooter.stop();
    }
}