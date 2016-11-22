package driving1;

import repast.simphony.context.Context;
import repast.simphony.context.DefaultContext;
import repast.simphony.context.space.continuous.ContinuousSpaceFactory;
import repast.simphony.context.space.continuous.ContinuousSpaceFactoryFinder;
//import repast.simphony.context.space.grid.GridFactory;
//import repast.simphony.context.space.grid.GridFactoryFinder;
import repast.simphony.dataLoader.ContextBuilder;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.SimpleCartesianAdder;
import repast.simphony.space.continuous.StrictBorders;
//import repast.simphony.space.grid.Grid;
//import repast.simphony.space.grid.GridBuilderParameters;
//import repast.simphony.space.grid.SimpleGridAdder;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.engine.schedule.*;

/**
 * Initializing script to build the space and context.
 * @author Darryl Michaud
 */

public class RoadBuilder extends DefaultContext<Object> implements ContextBuilder<Object> {
	// declare variables
	// these are means, distributed normally with stdDev = 8% of mean
	static final double spaceScale	= 7.5;	// length of 1 spatial unit in meters
	static final double timeStep	= 0.18;	// duration of 1 tick in seconds
	static final double vBase 		= (spaceScale/timeStep)*3600/1000;
		   // vBase is the natural speed of the model (one unit per tick), converted to km/hr
	static final double roadL		= 200;
	static final double laneW		= 3/spaceScale;		//lane width from http://safety.fhwa.dot.gov/geometric/pubs/mitigationstrategies/chapter3/3_lanewidth.cfm
	static final double roadW		= 4*laneW;
	static final double sidewalk	= 2/spaceScale;
	static final double worldW		= roadW + 2*sidewalk;
	static final double xWalkx		= roadL/2;	//places x-walk in middle of environment
	static final double pedVavg		= 5;		//km/hr
	static final double maxa		= 0.33;
	static final double mina		= 0.08;
	static final double tGap		= 1.9;
	static final double jamHead		= 0.3;
	static final double Vs			= 0.1;			// relative standard deviation of headEst from head
	static final double errPers		= 20;			// persistence time of estimation errors in seconds
	static final double wien1		= Math.exp(-timeStep/errPers);		//constants in the calculation
	static final double wien2		= Math.sqrt(2*timeStep/errPers);		//of perception errors
//	static public boolean carsYes	= true;			// turns cars on/off for testing
//	static public boolean pedsYes	= false;		// ditto for peds
//  static public int ticker		= 1;			// timer for testing
//	static final double pedRho		= 60;		//pph
//	static final double vehRho		= 600;		//vph
//	static final double vLimit		= 45;		//km/hr 
	static public ISchedule clock;
	
	@SuppressWarnings({ "rawtypes", "unused" })
	@Override
	public Context build(Context<Object> context) {
		context.setId("driving1");
		ContinuousSpaceFactory spaceFactory = 
				ContinuousSpaceFactoryFinder.createContinuousSpaceFactory(null);
		ContinuousSpace<Object> space =	
				spaceFactory.createContinuousSpace("space",context, new SimpleCartesianAdder<Object>(),
												   new StrictBorders(), roadL, worldW);
//		GridFactory gridFactory = GridFactoryFinder.createGridFactory(null);
//		Grid<Object> grid = 
//				gridFactory.createGrid("grid",context,
//									   new GridBuilderParameters<Object>(new repast.simphony.space.grid.StrictBorders(),
//					   							 new SimpleGridAdder<Object>(),true, (int)roadL, (int)worldW));
		clock = RunEnvironment.getInstance().getCurrentSchedule();
		Scheduler flowSource = new Scheduler();
		context.add(flowSource);
		
		UserPanel panel = new UserPanel();
		return context;}
}