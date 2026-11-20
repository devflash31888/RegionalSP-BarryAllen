package org.firstinspires.ftc.teamcode.Tuning;

import static org.firstinspires.ftc.teamcode.ConfigRegional.TuningConfig.TurretTuningConfig.targetDegreesDash;
import static org.firstinspires.ftc.teamcode.ConfigRegional.TuningConfig.TurretTuningConfig.useDashTarget;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ConfigRegional.TuningConfig;

import dev.easyfamily.EasyStack.control.EasyPIDController;
import dev.easyfamily.EasyStack.gamepad.EasyGamepad;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotor;
import dev.easyfamily.EasyStack.hardware.Motors.EasyMotorType;
import dev.easyfamily.EasyStack.opmode.EasyOpMode;
import dev.easyfamily.EasyStack.util.EasyMathFunctions;

@TeleOp(name = "Turret Panels Tuning", group = "Calibration")
public class TurretTuning extends EasyOpMode {

    // --- CONFIGURAÇÃO MECÂNICA ---
    private static final EasyMotorType MOTOR_TYPE = EasyMotorType.GOBILDA_435; // Troque pelo seu motor se necessário
    private static final double MOTOR_TPR = 537.7; // 384.5 para 435 RPM | 537.6 para 312 RPM
    private static final double GEAR_RATIO = 2.32;
    private static final double MAX_ANGLE_DEG = 180;

    private double ticksPerTurretDegree;
    private EasyMotor turretMotor;
    private EasyGamepad pad;
    private EasyPIDController pid;
    private TelemetryManager panelsTelemetry;

    private double targetDegrees = 0.0;

    @Override
    public void initialize() {
        pad = new EasyGamepad(gamepad1);

        // Inicializa o gerenciador de telemetria e gráficos do Panels
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Inicializa o motor e define a posição atual como ZERO (0°)
        turretMotor = new EasyMotor(hardwareMap, "turret", EasyMotorType.GOBILDA_312);
        turretMotor.setCurrentPositionAsZero();
        turretMotor.setInverted(true);

        ticksPerTurretDegree = (MOTOR_TPR * GEAR_RATIO) / 360.0;

        // Inicializa o PID com os valores do TurretTuningConfig
        pid = new EasyPIDController(
                TuningConfig.TurretTuningConfig.kP,
                TuningConfig.TurretTuningConfig.kI,
                TuningConfig.TurretTuningConfig.kD
        );
        pid.setTolerance(1.0);
        pid.setMaxIntegralSum(50.0);
    }

    @Override
    public void onUpdate() {
        pad.update();

        // Atualiza os ganhos do PID automaticamente conforme você altera no Panels
        pid.setPID(
                TuningConfig.TurretTuningConfig.kP,
                TuningConfig.TurretTuningConfig.kI,
                TuningConfig.TurretTuningConfig.kD
        );

        // --- ESCOLHA DO ALVO (GAMEPAD OU PANELS) ---
        if (TuningConfig.TurretTuningConfig.useDashTarget) {
            targetDegrees = TuningConfig.TurretTuningConfig.targetDegreesDash;
        } else {
            if (pad.aPressed()) {
                targetDegrees = 0.0;
                pid.reset();
            } else if (pad.bPressed()) {
                targetDegrees = 90.0;
                pid.reset();
            } else if (pad.xPressed()) {
                targetDegrees = -90.0;
                pid.reset();
            } else if (pad.yPressed()) {
                targetDegrees = -180.0;
                pid.reset();
            }
        }

        // Trava de segurança de +/- 160 graus
        targetDegrees = EasyMathFunctions.clamp(targetDegrees, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);

        // Leitura da posição atual em graus
        double currentTicks = turretMotor.getPosition();
        double currentDegrees = currentTicks / ticksPerTurretDegree;

        // Cálculo do PID + kS (atrito estático)
        double pidOutput = pid.calculate(currentDegrees, targetDegrees);
        double errorDegrees = pid.getError();

        double power = pidOutput;
        if (!pid.atTarget() && Math.abs(errorDegrees) > 0.5) {
            power += Math.signum(errorDegrees) * TuningConfig.TurretTuningConfig.kS;
        }

        if (Math.abs(errorDegrees) < 0.3) {
            power = 0.0;
        }

        // Limite de potência seguro para calibração
        power = EasyMathFunctions.clamp(power, -0.5, 0.5);
        turretMotor.setPower(power);

        // --- ENVIO PARA OS GRÁFICOS DO PANELS ---
        panelsTelemetry.debug("Alvo (deg)", targetDegrees);
        panelsTelemetry.debug("Atual (deg)", currentDegrees);
        panelsTelemetry.debug("Erro (deg)", errorDegrees);
        panelsTelemetry.debug("Potencia", power);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop() {
        turretMotor.setPower(0.0);
    }
}