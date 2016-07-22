package driving;

import java.util.ArrayList;
import java.util.Random;

import repast.simphony.context.Context;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.grid.Grid;
import repast.simphony.util.ContextUtils;

/**
 * Class created to run schedule of car creation, etc.
 * @author Darryl Michaud
 */
public class Scheduler extends Agent {
	
	static ArrayList<Turtle> allCars;
	static ArrayList<Ped> allPeds;
	Random rndP = new Random(); //initiates random number generator for Poisson arrival
	
	double lambda = RoadBuilder.vehRho * RoadBuilder.timeStep / 3600; // vehicle arrivals per tick, currently 0.03
	double Pof1 = lambda * Math.exp(-lambda); // Poisson probability of 1 arrival in a tick (higher counts are negligible)
	
	/**
	 * Schedules actions outside agents.
	 * Car creation scheduled as Poisson arrival
	 */
	@Override
	public void step() {
		//TODO: make cars and peds come from both directions	
		if (rndP.nextDouble() <= Pof1) {
			Turtle addedTurtle = addCar();
			allCars.add(addedTurtle);
		}
		//TODO: addPed();
	}
	
	/**
	 * Used to add cars to left edge
	 */
	//TODO: create distribution code to vary vehicle parameters
	@SuppressWarnings({ "unchecked", "rawtypes" })
	public Turtle addCar() {
		Context context = ContextUtils.getContext(this);
		ContinuousSpace<Object> space = (ContinuousSpace<Object>) context.getProjection("space");
		Grid<Object> grid = (Grid) context.getProjection("grid");
		
		int lane = rndP.nextInt(2); // picks between 0 and 1 randomly
		double yPlacement = (double)lane * (RoadBuilder.roadWidth/2) + (RoadBuilder.roadWidth/4);
		
		Turtle newTurtle = new Turtle(space,grid);
		context.add(newTurtle);
		space.moveTo(newTurtle,0,yPlacement);
		return(newTurtle);}
	
	/**
	 * Initializes lists of all agents at start of run.
	 */
	public Scheduler() {
		allCars = new ArrayList<Turtle>();
		allPeds = new ArrayList<Ped>();
	}
	
}