package org.firstinspires.ftc.teamcode.ConfigRegional;

import com.bylazar.configurables.annotations.Configurable;

public class TuningConfig {

    @Configurable
    public static class TurretTuningConfig {
        public static double kP = 0.02;
        public static double kI = 0.0;
        public static double kD = 0.001;
        public static double kS = 0.01;

        // Permite testar alvos direto pelo navegador sem nem precisar do controle!
        public static double targetDegreesDash = 0.0;
        public static boolean useDashTarget = false;
    }
}
