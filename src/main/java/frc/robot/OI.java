package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.HelperRocketAuto;

public class OI {
	private static Joystick drive = new Joystick(RobotConfig.driveTrainJoystickPort);
	private static Joystick intake = new Joystick(RobotConfig.intakeJoystickPort);

	private Button runHelperButton = new JoystickButton(drive, RobotConfig.runHelperButton);

	public OI() {
		runHelperButton.whenPressed(new HelperRocketAuto());
	}

	public double getForwardAxis() {
		return -1 * drive.getRawAxis(RobotConfig.forwardAxis);
	}

	public double getTurnAxis() {
		return drive.getRawAxis(RobotConfig.turnAxis);
	}

	public double getIntakeAxis() {
		return intake.getRawAxis(RobotConfig.intakeJoystickAxis);
	}
}
