package frc.robot;

public class PIDSetting {
    double kp, ki, kd, kf;
    PIDSetting(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }
}