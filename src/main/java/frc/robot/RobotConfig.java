package frc.robot;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;

import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;

/**
 * the only variables that need to be changed in order to work with another robot.
 * nothing else should require edits
 */
public class RobotConfig {
	public static final int intakePort = 0;
	public static final int intakeJoystickPort = 0;
	public static final int intakeJoystickAxis = 0;

	public static final int leftMasterMotorPort = 1;
	public static final int leftSlaveMotorPort = 2;
	public static final int rightMasterMotorPort = 3;
	public static final int rightSlaveMotorPort = 4;

	public static final NativeUnit driveTrainUnitsPerRot = NativeUnitKt.getNativeUnits(1024);// NativeUnitKt.getNativeUnit(1024);
	public static final Length rightWheelRadius = LengthKt.getInch(2);
	public static final Length leftWheelRadius = LengthKt.getInch(2);

	public static final double kRobotMass = 50 /* Robot, kg */ + 5f /* Battery, kg */ + 2f /* Bumpers, kg */;
	public static final double kRobotMomentOfInertia = 10.0; // kg m^2 // TODO Tune
	public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec)

	public static final double kWheelRadius = (3f / 12f) * 0.3048;// meters. TODO tune
	public static final double kTrackWidth = (26f / 12f) * 0.3048;// meters

	private static final double kVDriveLeftLow = 0.265 * 1d; // Volts per radians per second - Calculated emperically
	private static final double kADriveLeftLow = 0.027 * 1d; // Volts per radians per second per second TODO tune
	private static final double kVInterceptLeftLow = 0.95 * 1d; // Volts - tuned!

	private static final double kVDriveRightLow = 0.275 * 1d; // Volts per radians per second - Calculated emperically
	private static final double kADriveRightLow = 0.0286 * 1d; // Volts per radians per second per second TODO tune
	private static final double kVInterceptRightLow = 0.96 * 1d; // Volts - tuned!

	public static class VeryStatic {
		public static final DCMotorTransmission kLeftTransmissionModelLowGear = new DCMotorTransmission(1 / kVDriveLeftLow,
				kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveLeftLow),
				kVInterceptLeftLow);

		public static final DCMotorTransmission kRightTransmissionModelLowGear = new DCMotorTransmission(1 / kVDriveRightLow,
				kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveRightLow),
				kVInterceptRightLow);

		public static final DifferentialDrive kLowGearDifferentialDrive = new DifferentialDrive(kRobotMass, kRobotMomentOfInertia,
				kRobotAngularDrag, kWheelRadius, kTrackWidth / 2.0, kLeftTransmissionModelLowGear, kRightTransmissionModelLowGear);
	}
}
