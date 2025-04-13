package frc.robot.subsystems;

public interface GenericPresicionSystemIO {

    public static class GenericPresicionSystemIOInputs {
        public double Speed = 0; //deg/s
        public double MotorVoltageLeft = 0; //volts
        public double MotorVoltageRight = 0; //volts
        public double MotorTempLeft = 0; // celsius
        public double MotorTempRight = 0; // celsius
        public double Angle = 0; //degrees
        public double TargetAngle = 0; //degrees

    }

    public enum Goal {
        CLIMB,
        SCORE_MIDDLE,
        SCORE_UP,
        INTAKE_DOWN,
        INTAKE_UP
    }

    default public void setSpeed(double speed) {}

    default public void setTargetAngle(double position) {}

    default public void moveTargetAngle() {}

    default public void setVoltage(double voltage) {}

    default public void stop() {setSpeed(0);}

    default public void updateInputs(GenericPresicionSystemIOInputs inputs) {}

    default public void setPID(double KP, double KI, double KD) {}

    default public void setFF(double KS, double KG, double KV) {}

    default public void setPIDF(double KP, double KI, double KD, double KS, double KG, double KV) {
        setPID(KP, KI, KD);
        setFF(KS, KG, KV);
    }

    default public void setIdleModeBreak() {}

    default public void setIdleModeCoast() {}

    default public void enabledInit() {}
}
