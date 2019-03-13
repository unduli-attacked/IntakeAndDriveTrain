package frc.robot;

import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Main robot class. There shouldn't be a *ton* of stuff here, mostly init
 * functions and smartdashboard stuff.
 *
 * @author Matthew Morley
 */
public class Robot extends TimedRobot {

	public SendableChooser<String> startingPos = new SendableChooser<String>();
	DriverStationSim sim = new DriverStationSim();

	public static PWMDriveTrain drive = new PWMDriveTrain();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		startingPos.setDefaultOption("Center", "M");
		startingPos.addOption("Left", "L");
		startingPos.addOption("Right", "R");

		Logger.log("robot program starttttting");
		sim.setEnabled(true);

	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		Logger.log("hulllllllo there");
	}

	/**
	 * This function is called periodically during operator control.
	 *
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		drive.getLeftMotor().setVelocityAndArbitraryFeedForward(VelocityKt.getVelocity(LengthKt.getFeet(2)), 0.1);

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {}

	/**
	 * This function is called every robot packet, no matter the mode. Use // * this
	 * for items like diagnostics that you want ran during disabled, // *
	 * autonomous, teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		SmartDashboard.putData(startingPos);
		// Logger.log("Is the robot enabled? " + sim.getEnabled());

	}

	public static class Logger {
		public static void log(Object o) {
			System.out.println(o);
		}

		public static void log(String thing) {
			System.out.println(thing);
		}
	}

}
