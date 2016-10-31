package driving1;

import repast.simphony.context.Context;
import repast.simphony.context.space.continuous.ContinuousSpaceFactory;
import repast.simphony.context.space.continuous.ContinuousSpaceFactoryFinder;
import repast.simphony.context.space.grid.GridFactory;
import repast.simphony.context.space.grid.GridFactoryFinder;
import repast.simphony.dataLoader.ContextBuilder;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.SimpleCartesianAdder;
import repast.simphony.space.continuous.StrictBorders;
import repast.simphony.space.grid.Grid;
import repast.simphony.space.grid.GridBuilderParameters;
import repast.simphony.space.grid.SimpleGridAdder;

/**
 * Initializing script to build the space and context. Will only focus on central area
 * in analysis to avoid edge effects of (dis)appearing vehicles.
 * @author Darryl Michaud
 */

public class RoadBuilder implements ContextBuilder<Object> {
	// declare variables
	// these are means, distributed normally with stdDev = 8% of mean
	static final double spaceScale = 7.5; // length of 1 spatial unit in meters
	static final double timeStep = 0.18;  // duration of 1 tick in seconds
	static final double vBase = (spaceScale/timeStep)*3600/1000;
		   // vBase is the natural speed of the model (one unit per tick), converted to km/hr
	static final double roadL    = 200;
	static final double roadW    = 4;
	static final double sidewalk = 1;
	static final double worldW   = roadW + 2*sidewalk;
	static final double xWalkx   = roadL/2; // places x-walk in middle of environment
//	static final double pedRho   = 60;  //pph
	static final double pedVavg  = 5;  //km/hr
//	static final double vehRho   = 600; //vph
	static final double vLimit   = 45;  //km/hr 
	static final double maxa     = 0.33;
	static final double mina     = 0.08;
	static final double tGap     = 1.9;
	static final double jamHead  = 0.3;
//	static public boolean carsYes = true;  // turns cars on/off for testing
//	static public boolean pedsYes = false;   // ditto for peds
	static public int ticker     = 1;           // timer for testing
		
/*  The HCM gives warrants for the need to install traffic signals:
	http://mutcd.fhwa.dot.gov/htm/2009/part4/part4c.htm (specifically Figure 4C-5)
	Here, a traffic volume of 600 vph and a pedestrian volume of 200 pph is used.
	A speed of 45 km/hr was chosen to lie comfortably below the 35mph threshold of the HCM
	TODO: add veh_vol, ped_vol, v_limit, time_step variables to interface: http://repast.sourceforge.net/docs/RepastReference.pdf (GUI params)
	TODO: use grid (and, maybe, EditedValueLayerStyle) to add colored background, e.g. in mousetrap
*/
	
	@SuppressWarnings({ "rawtypes", "unused" })
	@Override
	public Context build(Context<Object> context) {
		context.setId("driving1");
		ContinuousSpaceFactory spaceFactory = 
				ContinuousSpaceFactoryFinder.createContinuousSpaceFactory(null);
		ContinuousSpace<Object> space =	
				spaceFactory.createContinuousSpace("space",context, new SimpleCartesianAdder<Object>(),
												   new StrictBorders(), roadL, worldW);
		GridFactory gridFactory = GridFactoryFinder.createGridFactory(null);
		Grid<Object> grid = 
				gridFactory.createGrid("grid",context,
									   new GridBuilderParameters<Object>(new repast.simphony.space.grid.StrictBorders(),
					   							 new SimpleGridAdder<Object>(),true, (int)roadL, (int)worldW));
		Scheduler flowSource = new Scheduler();
		context.add(flowSource);
		
		UserPanel panel = new UserPanel();
		
		return context;}
}

//ISchedulableActionFactory scheduleFactory =
