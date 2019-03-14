package frc.robot.lib;

public class PIDSetting {
	double kp, ki, kd, kf;

	public PIDSetting(double kp, double ki, double kd, double kf) {
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.kf = kf;
	}
}
