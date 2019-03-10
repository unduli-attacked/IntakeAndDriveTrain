package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

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
