package driving1;

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
	Random rndCar = new Random(); //initiates random number generator for Poisson vehicle arrival
	Random rndPed = new Random(); //ditto for peds so the two are independent
	
	/**
	 * Schedules actions outside agents.
	 * Car creation scheduled as Poisson arrival
	 */
	@Override
	public void step() {
		//TODO: add second term of poisson PMF to allow for cars arriving in both lanes:
		//      exp(-lam) * lam^x / x!  will require if statement
		double lambdaCar = UserPanel.vehRho * RoadBuilder.timeStep / 3600; // vehicle arrivals per tick, currently 0.03
		double lambdaPed = UserPanel.pedRho * RoadBuilder.timeStep / 3600; // peds per tick, currently 0.003
		double Pof1Car = lambdaCar * Math.exp(-lambdaCar); // Poisson probability of 1 arrival in a tick (higher counts are negligible)
		double Pof1Ped = lambdaPed * Math.exp(-lambdaPed); // ditto for peds
		// From MUTCD 4C.4, traffic higher than 1450 VPM (both ways, w/ PPH > 133) warrants signal
		//TODO: make cars and peds come from both directions	
		//TODO: need added buffer to keep cars/peds from being created too close to each other
		RoadBuilder.ticker += 1;
		if (UserPanel.carsYes == true) {
			if (rndCar.nextDouble() <= Pof1Car) {
				Turtle addedTurtle = addCar();
				allCars.add(addedTurtle);}}
		if (UserPanel.pedsYes == true) {
//			if (RoadBuilder.ticker % 100 == 0) {
			if (rndPed.nextDouble() <= Pof1Ped) {
				Ped addedPed = addPed();
				allPeds.add(addedPed);}}
		
		//space holder for breakpoint
		@SuppressWarnings("unused")
		int foo = 0; 
	}
	
	/**
	 * Used to add cars to left edge
	 */
	@SuppressWarnings({ "unchecked", "rawtypes" })
	public Turtle addCar() {
		Context context = ContextUtils.getContext(this);
		ContinuousSpace<Object> space = (ContinuousSpace<Object>) context.getProjection("space");
		Grid<Object> grid = (Grid) context.getProjection("grid");
		
		int lane = rndCar.nextInt(2); // picks between 0 and 1 randomly
		double yPlacement = (double)lane * (RoadBuilder.roadW/2) + (RoadBuilder.roadW/4);
		
		Turtle newTurtle = new Turtle(space,grid);
		context.add(newTurtle);
		space.moveTo(newTurtle,0,yPlacement);
		return(newTurtle);
	}
	
	/**
	 * Used to add peds at x-walk
	 */
	@SuppressWarnings({ "unchecked", "rawtypes" })
	public Ped addPed() {
		Context context = ContextUtils.getContext(this);
		ContinuousSpace<Object> space = (ContinuousSpace<Object>) context.getProjection("space");
		Grid<Object> grid = (Grid) context.getProjection("grid");
		
		/* TODO: add peds coming from both directions
		int dir0 = rndPed.nextInt(2); //picks between 0 (walking up) and 1 (walking down)
		int dir = dir0 == 0 ? 1 : -1;
		double yPlacement = (double)dir0 * RoadBuilder.worldW;
		*/
		
		int dir = 1;
		double yPlacement = 0;
		
		Ped newPed = new Ped(space,grid,dir);
		context.add(newPed);
		space.moveTo(newPed,RoadBuilder.xWalkx,yPlacement);
		return(newPed);
	}
	
	/**
	 * Initializes lists of all agents at start of run.
	 */
	public Scheduler() {
		allCars = new ArrayList<Turtle>();
		allPeds = new ArrayList<Ped>();
	}
}
