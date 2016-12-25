package driving1;

import java.util.ArrayList;
import java.util.Random;

import repast.simphony.context.Context;
import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.util.ContextUtils;


/**
 * Class created to contain schedule.
 * Agents are created, they observe their surroundings, then move.
 * @author Darryl Michaud
 */
public class Scheduler extends Agent {
	
	static ArrayList<Turtle> allCars;
	static ArrayList<Ped> allPeds;
	static ArrayList<Turtle> killListC = new ArrayList<Turtle>();
	static ArrayList<Ped>	 killListP = new ArrayList<Ped>();
	Random rndCar = new Random(); //initiates random number generator for Poisson vehicle arrival
	Random rndPed = new Random(); //ditto for peds so the two are independent
	double rndC, rndP, yPlacement;
	int    lane, dir;
	boolean carsYes, pedsYes;
	
	/*
	 * The scheduled method that contains all major steps.
	 */
	@ScheduledMethod(start = 1, interval = 1, priority = 1)
	public void doStuff() {
		RoadBuilder.flowSource.calc();
		carsYes = (allCars.size() > 0);
		pedsYes = (allPeds.size() > 0);
		if (carsYes) {
			for (Turtle a : allCars) {
				a.calc();}}
		if (pedsYes) {
			for (Ped a : allPeds) {
				a.calc();}}
		if (carsYes) {
			for (Turtle b : allCars) {
				b.drive();}}
		if (pedsYes) {
			for (Ped b : allPeds) {
				b.walk();}}
		if (!killListC.isEmpty()) {
			for (Turtle c : killListC) {
				c.die();}
			killListC = new ArrayList<Turtle>();}
		if (!killListP.isEmpty()) {
			for (Ped c : killListP) {
				c.die();}
			killListP = new ArrayList<Ped>();}
	}
	
	public void calc() {
		if (UserPanel.vehRho > 0) {
			rndC = rndCar.nextDouble();
			if (rndC <=  UserPanel.Pof2Car) {
				Turtle addedTurtle1 = addCar(0);
				Turtle addedTurtle2 = addCar(1);
				allCars.add(addedTurtle1);
				allCars.add(addedTurtle2);}
			else if (rndC <= UserPanel.Pof1Car) {
				lane = rndCar.nextInt(2);		// picks between 0 and 1 randomly
				Turtle addedTurtle = addCar(lane);
				allCars.add(addedTurtle);}}
		if (UserPanel.pedRho > 0) {
			rndP = rndPed.nextDouble();
//			if (RoadBuilder.ticker % 100 == 0) { // Constant stream for testing
			if (rndP <=  UserPanel.Pof2Ped) {
				Ped addedPed1 = addPed();
				Ped addedPed2 = addPed();
				allPeds.add(addedPed1);
				allPeds.add(addedPed2);}
			else if (rndP <= UserPanel.Pof1Ped) {
				Ped addedPed = addPed();
				allPeds.add(addedPed);}}
	}
	
	/**
	 * Used to add cars to left edge
	 */
	@SuppressWarnings("unchecked")
	public Turtle addCar(int lane) {
		Context<Object> context = ContextUtils.getContext(this);
		ContinuousSpace<Object> space = (ContinuousSpace<Object>) context.getProjection("space");
//		Grid<Object> grid = (Grid) context.getProjection("grid");		
		yPlacement = RoadBuilder.sidewalk + (double)lane * (RoadBuilder.laneW) + (RoadBuilder.laneW/2);
		
//		Turtle newTurtle = new Turtle(space,grid);
		Turtle newTurtle = new Turtle(space,lane);
		context.add(newTurtle);
		space.moveTo(newTurtle,0,yPlacement);
		return(newTurtle);
	}
	
	/**
	 * Used to add peds at x-walk
	 */
	@SuppressWarnings("unchecked")
	public Ped addPed() {
		Context<Object> context = ContextUtils.getContext(this);
		ContinuousSpace<Object> space = (ContinuousSpace<Object>) context.getProjection("space");
//		Grid<Object> grid = (Grid) context.getProjection("grid");
		/* TODO: add peds coming from both directions
		int dir0 = rndPed.nextInt(2); //picks between 0 (walking up) and 1 (walking down)
		int dir = dir0 == 0 ? 1 : -1;
		double yPlacement = (double)dir0 * RoadBuilder.worldW;
		*/
		
		dir = 1;
		yPlacement = 0;
		
//		Ped newPed = new Ped(space,grid,dir);
		Ped newPed = new Ped(space,dir);
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