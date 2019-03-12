package frc.robot.commands;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PickupHatch extends CommandGroup {

	/**
	 * Pickup a hatch from the loading station using some jank open loop code.
	 * Vision code coming soon, I hope
	 * 
	 * @author Matthew Morley
	 */
	public PickupHatch() {

		// addSequential(new LimeLight.SetLEDs(LimeLight.LEDMode.kON));

		// addSequential(new LimeLight.setPipeline(PipelinePreset.k3dVision));


		// addSequential(new FollowVisionTargetTheSecond(8)); // in high res mode

		// ======================================================================


		// addSequential(SequentialCommandFactory.getSequentialCommands(Arrays.asList(
		// new WaitCommand(0.75),
		// new RunIntake(1, 0, 0.5))));

		// addParallel(new DriveDistanceTheSecond(LengthKt.getFeet(0.5), false)); // TODO run the next spline, saves time, vs backing up

		addSequential(new RunIntake(1, .5));

		// addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(1), true)); // TODO run the next spline, saves time, vs backing up

		// addSequential(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));

	}
}
