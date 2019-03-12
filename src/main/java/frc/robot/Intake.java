package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem {

	Talon talon;
	private static Intake inst;

	public static Intake getInstance() {
		if (inst == null) {
			inst = new Intake(RobotConfig.intakePort);
		}
		return inst;
	}

	protected Intake(int port) {
		this.talon = new Talon(port);
	}

	public void setSpeed(double speed) {
		talon.set(speed);
	}

	public void stop() {
		this.setSpeed(0);
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new IntakeTeleop());
	}

	public class IntakeTeleop extends Command {

		public IntakeTeleop() {
		}

		@Override
		protected void execute() {
			Intake.getInstance().setSpeed(OI.getIntakeAxis());
		}

		@Override
		protected boolean isFinished() {
			return false;
		}

	}

}
