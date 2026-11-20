package org.firstinspires.ftc.teamcode.ConfigRegional;

public class RobotConfig {

    public static class Drive {
        public static double stickDeadzone = 0.05;
        public static double stickExponent = 2.0;
        public static double slowModeFactor = 0.5;
        public static double turnSensitivity = 1.0;
    }

    public static class Field {
        public static double relocX = 0;
        public static double relocY = 0;
        public static double relocHeading = 0;

        public static double goalLeftX = 52.05, goalLeftY = 80.64;
        public static double goalRightX = 52.05, goalRightY = 46.98;
    }

    public static class Intake {
        public static double intakePower = 1.0;
        public static double reversePower = -1.0;
        public static double def = 0.0;
    }

    public static class Led {
        public static double on = 1.0;
        public static double off = 0.0;
    }

    public static class Gate {
        public static double openPosition = 0.4;
        public static double closedPosition = 0.14;
        public static double moveTimeMS = 50;
    }

    public static class FlowerRemover {
        public static double downPositionRight = 0.5;
        public static double upPositionRight = 0.0;
        public static double moveTimeMs = 400;
    }


    public static class Vision {
        public static int tagPipeline = 0;
        public static int[] allowTagIdsBlue = {39, 40, 43, 44};
        public static int[] allowTagIdsRed = {0, 0, 0, 0};
    }

    public static class Turret {
        public static double angleToleranceDeg = 0.3;

        public static double angleDefault = 0.0;

        public static double kP = 0.02;
        public static double kI = 0.0;
        public static double kD = 0.001;
        public static double kS = 0.01;
    }

    public static class Shooter {
        public static double kP = 0.004;
        public static double kV = 0.000475;
        public static double kS = 0.055;
        public static double boostThreshold = 50;
        public static double readyTolerance = 25;
        public static double fixedVelocity = 1250;

        public static double[] shootDistances = {0, 0, 0, 0, 0};// ajustar
        public static double[] shootSpeeds = {0, 0, 0, 0, 0};// ajustar
        public static double[] shootHood = {0, 0, 0, 0, 0};// ajustar
    }
}
