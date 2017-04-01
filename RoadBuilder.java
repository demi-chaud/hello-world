package driving1;

import repast.simphony.context.Context;
import repast.simphony.context.DefaultContext;
import repast.simphony.context.space.continuous.ContinuousSpaceFactory;
import repast.simphony.context.space.continuous.ContinuousSpaceFactoryFinder;
import repast.simphony.dataLoader.ContextBuilder;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.SimpleCartesianAdder;
import repast.simphony.space.continuous.StrictBorders;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.engine.schedule.*;

/**
 * Initializing script to build the space, context, and schedule.
 * @author Darryl Michaud
 */

public class RoadBuilder extends DefaultContext<Object> implements ContextBuilder<Object> {
	
	// if unspecified, units are model space units and ticks 
	static final  double spaceScale	= 0.5;		// m (length of 1 spatial unit)
	static final  double roadL		= 3000;		// standard: 3000 = 1.5km
	static final  double laneW		= 3.3/spaceScale;
	static final  double roadW		= 4*laneW;
	static final  double sidewalk	= 4/spaceScale;
	static final  double worldW		= roadW + 2*sidewalk;
	static final  double xWalkx		= roadL/2;	// places x-walk in middle of environment
	static public ISchedule clock;
	static public Scheduler flowSource;
	static public int ticker = 0;
	
	@SuppressWarnings({"unused"})
	@Override
	public Context<Object> build(Context<Object> context) {
		context.setId("driving1");
		ContinuousSpaceFactory spaceFactory = 
				ContinuousSpaceFactoryFinder.createContinuousSpaceFactory(null);
		ContinuousSpace<Object> space =	
				spaceFactory.createContinuousSpace("space",context, new SimpleCartesianAdder<Object>(),
												   new StrictBorders(), roadL, worldW);
		clock = RunEnvironment.getInstance().getCurrentSchedule();
		flowSource = new Scheduler();
		context.add(flowSource);
		UserPanel panel = new UserPanel();
		return context;
	}
}