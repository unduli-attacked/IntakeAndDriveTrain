package frc.robot.lib.drivebases;

import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.physics.DifferentialDrive.ChassisState;
import com.team254.lib.physics.DifferentialDrive.WheelState;

import edu.wpi.first.wpilibj.command.Subsystem;
import kotlin.ranges.RangesKt;

public interface DriveTrainBase<T> {

	public Subsystem getRealSubsystem();

	public void setNeutralMode(NeutralMode mode);

	public void tankDrive(double leftPercent, double rightPercent);

	public T getLeftMotor();

	public T getRightMotor();

	public default void arcadeDrive(double linearPercent, double rotationPercent, boolean squareInputs) {
		linearPercent = Math.min(1, Math.max(-1, linearPercent));
		linearPercent = (Math.abs(linearPercent) < 0.02) ? 0 : linearPercent;

		rotationPercent = Math.min(1, Math.max(-1, rotationPercent));
		rotationPercent = (Math.abs(rotationPercent) < 0.02) ? 0 : rotationPercent;

		// Square the inputs (while preserving the sign) to increase fine control
		// while permitting full power.
		if (squareInputs) {
			linearPercent = Math.copySign(linearPercent * linearPercent, linearPercent);
			rotationPercent = Math.copySign(rotationPercent * rotationPercent, rotationPercent);
		}

		double leftMotorOutput;
		double rightMotorOutput;

		double maxInput = Math.copySign(Math.max(Math.abs(linearPercent), Math.abs(rotationPercent)), linearPercent);

		if (linearPercent >= 0.0) {
			// First quadrant, else second quadrant
			if (rotationPercent >= 0.0) {
				leftMotorOutput = maxInput;
				rightMotorOutput = linearPercent - rotationPercent;
			} else {
				leftMotorOutput = linearPercent + rotationPercent;
				rightMotorOutput = maxInput;
			}
		} else {
			// Third quadrant, else fourth quadrant
			if (rotationPercent >= 0.0) {
				leftMotorOutput = linearPercent + rotationPercent;
				rightMotorOutput = maxInput;
			} else {
				leftMotorOutput = maxInput;
				rightMotorOutput = linearPercent - rotationPercent;
			}
		}

		ChassisState mTarget = new ChassisState(linearPercent * 6, -1 * rotationPercent * 6);

		WheelState mCalced = getDifferentialDrive().solveInverseKinematics(mTarget);

		double left = mCalced.get(true);

		double right = mCalced.get(false);

		// tankDrive(left/12, right/12);

		tankDrive(leftMotorOutput, rightMotorOutput);
		// tankDrive(0.2, 0.2);
	}

	public default void curvatureDrive(double linearPercent, double curvaturePercent, boolean isQuickTurn) {
		double angularPower;
		boolean overPower;

		double kQuickStopThreshold = 0.2;// DifferentialDrive.kDefaultQuickStopThreshold;
		double kQuickStopAlpha = 0.1;// DifferentialDrive.kDefaultQuickStopAlpha;
		double quickStopAccumulator = 0;

		if (isQuickTurn) {
			if (Math.abs(linearPercent) < kQuickStopThreshold) {
				quickStopAccumulator = (1 - kQuickStopAlpha) * quickStopAccumulator
						+ kQuickStopAlpha * RangesKt.coerceIn(curvaturePercent, -1, 1) * 2.0;
			}

			overPower = true;
			angularPower = curvaturePercent;
		} else {
			overPower = false;
			angularPower = Math.abs(linearPercent) * curvaturePercent - quickStopAccumulator;

			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0;
			}
		}

		double leftMotorOutput = linearPercent + angularPower;
		double rightMotorOutput = linearPercent - angularPower;

		// If rotation is overpowered, reduce both outputs to within acceptable range
		if (overPower) {
			if (leftMotorOutput > 1.0) {
				rightMotorOutput -= leftMotorOutput - 1.0;
				leftMotorOutput = 1.0;
			} else if (rightMotorOutput > 1.0) {
				leftMotorOutput -= rightMotorOutput - 1.0;
				rightMotorOutput = 1.0;
			} else if (leftMotorOutput < -1.0) {
				rightMotorOutput -= leftMotorOutput + 1.0;
				leftMotorOutput = -1.0;
			} else if (rightMotorOutput < -1.0) {
				leftMotorOutput -= rightMotorOutput + 1.0;
				rightMotorOutput = -1.0;
			}
		}

		// Normalize the wheel speeds
		double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
		if (maxMagnitude > 1.0) {
			leftMotorOutput /= maxMagnitude;
			rightMotorOutput /= maxMagnitude;
		}

		tankDrive(leftMotorOutput, rightMotorOutput);
	}

	public DifferentialDrive getDifferentialDrive();

	public default void stop() {
		tankDrive(0, 0);
	}

	public TrajectoryTracker getTrajectoryTracker();
}
