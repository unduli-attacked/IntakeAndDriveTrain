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
		Joystick stick = new Joystick(RobotConfig.intakeJoystickPort);
		setDefaultCommand(new IntakeTeleop(stick, RobotConfig.intakeJoystickAxis));
	}


	public class IntakeTeleop extends Command {

		public final Joystick stick;
		public final int axis;
	
		public IntakeTeleop(Joystick stick, int axis) {
			this.stick = stick;
			this.axis = axis;
		}
	
		@Override
		protected void execute() {
			Intake.getInstance().setSpeed(stick.getRawAxis(axis));
		}
	
		@Override
		protected boolean isFinished() {
			return false;
		}
	
	}

}
