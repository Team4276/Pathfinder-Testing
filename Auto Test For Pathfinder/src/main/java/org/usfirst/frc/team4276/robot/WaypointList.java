package org.usfirst.frc.team4276.robot;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory;

public class WaypointList {

	public static Waypoint[] route1 = new Waypoint[] {
		new Waypoint(0, 0, 0),      // Waypoint @ x=0, y=0, exit angle=0 degrees
		new Waypoint(10, 0, 0)      // Waypoint @ x=10, y=0, exit angle=0 radians
	};
	
	public static Waypoint[] route2 = new Waypoint[] {
		new Waypoint(0, 0, 0),      // Waypoint @ x=0, y=0, exit angle=0 degrees
		new Waypoint(10, 4, 0)      // Waypoint @ x=10, y=0, exit angle=0 radians
	};

	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
	Trajectory trajectory = Pathfinder.generate(route1, config);
	
}
