package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.PWMDriveTrain;
import frc.robot.Robot;
import frc.robot.lib.HalfBakedEncodedPWMMotorController;
import frc.robot.lib.drivebases.DriveTrainBase;

public class TeleopDriving extends Command {

	//TODO uncomment one of these based on the drivetrain type
	DriveTrainBase<HalfBakedEncodedPWMMotorController> drivetrain = PWMDriveTrain.getInstance();
	// DriveTrainBase<FalconSRX<Length>> drivetrain = DriveTrain.getInstance();

	public static enum DriveType {
		TANK, ARCADE, CURVATURE;
	}

	DriveType type = DriveType.ARCADE;

	public TeleopDriving(DriveType type) {
		requires(drivetrain.getRealSubsystem());
		this.type = type;
	}

	@Override
	protected void initialize() {
		drivetrain.setNeutralMode(NeutralMode.Brake);
		drivetrain.getLeftMotor().configOpenloopRamp(0.1);
		drivetrain.getRightMotor().configOpenloopRamp(0.1);
		drivetrain.tankDrive(0, 0);
		System.out.println("arcade drive command init");
	}

	@Override
	protected void execute() {
		double arg1 = Robot.oi.getForwardAxis();
		double arg2 = Robot.oi.getTurnAxis();

		switch (type) {
		case TANK:
			//FIXME this uses forward and turn as left and right
			drivetrain.tankDrive(arg1, arg2);
		case ARCADE:
			drivetrain.arcadeDrive(arg1 * 1,
					arg2 * 1, true);
		case CURVATURE:
			boolean isQuickTurn = false; //FIXME help i forgot how this was done
			drivetrain.curvatureDrive(arg1, arg2, isQuickTurn);
		}

	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		drivetrain.tankDrive(0, 0);
		System.out.println("arcade end called");
	}

	@Override
	protected void interrupted() {
		drivetrain.tankDrive(0, 0);
	}
}
