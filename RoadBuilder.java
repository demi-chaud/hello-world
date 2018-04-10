package driving1;

import java.util.ArrayList;
import java.util.Random;

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
import repast.simphony.parameter.Parameters;

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
	static final  double rl1x		= roadL/4;	// places left red light
	static final  double rl2x		= 3*roadL/4;// places right red light
	static public ArrayList<RedLight> lights = new ArrayList<RedLight>();
	static final  Random rnd		= new Random();
	static public ISchedule clock;
	static public Scheduler flowSource;
	static public int ticker = 0;
	static public RedLight rl1;
	static public RedLight rl2;
	static public UserPanel panel;
	public double percV2X;
	public double percAuto;
	public double percBoth;
	public int	  vehRho;
	public int	  pedRho;
	public double hPercLimM;
	public double sLimitKH;
	public double stopBarM;
	
	@Override
	public Context<Object> build(Context<Object> context) {
		context.setId("driving1");
		ContinuousSpaceFactory spaceFactory = 
				ContinuousSpaceFactoryFinder.createContinuousSpaceFactory(null);
		ContinuousSpace<Object> space =	
				spaceFactory.createContinuousSpace("space",context, new SimpleCartesianAdder<Object>(),
												   new StrictBorders(), roadL, worldW);
		clock = RunEnvironment.getInstance().getCurrentSchedule();
		Parameters param = RunEnvironment.getInstance().getParameters();
		percV2X  = (double)param.getValue("percV2X");
		percAuto = (double)param.getValue("percAuto");
		percBoth = (double)param.getValue("percBoth");
		vehRho	 = (int)param.getValue("vehRho");
		pedRho	 = (int)param.getValue("pedRho");
		hPercLimM= (double)param.getValue("hPercLimM");
		sLimitKH = (double)param.getValue("sLimitKH");
		stopBarM = 9;
		panel = new UserPanel(this);
		flowSource = new Scheduler();
		flowSource.allConf = new ArrayList<Turtle.Conflict>();
		flowSource.allCrash = new ArrayList<Turtle.Crash>();
		context.add(flowSource);
		int rnd1  = (int)Math.round(rnd.nextDouble()*UserPanel.greenDurS);
		int rnd2  = (int)Math.round(rnd.nextDouble()*UserPanel.greenDurS);
		if (UserPanel.inclRL) {
			rl1 = new RedLight();
			rl2 = new RedLight();
			rl1.timeInState = rnd1;
			rl1.myState = RedLight.state.GREEN;
			rl1.xLoc = rl1x;
			rl2.timeInState = rnd2;
			rl2.myState = RedLight.state.GREEN;
			rl2.xLoc = rl2x;
			context.add(rl1);
			context.add(rl2);
			lights.add(rl1);
			lights.add(rl2);
			space.moveTo(rl1, rl1x, 0);
			space.moveTo(rl2, rl2x, 0);}
		
		return context;
	}
	
	public String[] getInitParam() {
		String[] params = {"pedRho","vehRho","sLimitKH","hPercLimM","percV2X","percAuto","percBoth"};
		return params;
	}
	
	
}