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
	
	static  ArrayList<Turtle> allCars;
	static  ArrayList<Ped>	 allPeds;
	static  ArrayList<Turtle> killListC = new ArrayList<Turtle>();
	static  ArrayList<Ped>	 killListP = new ArrayList<Ped>();
	Random  rndCar = new Random(); //initiates random number generator for Poisson vehicle arrival
	Random  rndPed = new Random(); //ditto for peds so the two are independent
	double  rndC, rndP, rndC2, rndP2, yPlacement;
	int     lane, dir;
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
		RoadBuilder.ticker ++;
	}
	
	public void calc() {
		if (UserPanel.vehRho > 0) {
			rndC = rndCar.nextDouble();
			if (rndC <=  UserPanel.Pof2Car) {
				Turtle addedTurtle1 = addCar(0,1);
				Turtle addedTurtle2 = addCar(1,1);
				allCars.add(addedTurtle1);
				allCars.add(addedTurtle2);}
			else if (rndC <= UserPanel.Pof1Car) {
				lane = rndCar.nextInt(2);		// picks between 0 and 1 randomly
				Turtle addedTurtle = addCar(lane,1);
				allCars.add(addedTurtle);}
			if (UserPanel.bothCar == true) {
				rndC2 = rndCar.nextDouble();
				if (rndC2 <=  UserPanel.Pof2Car) {
					Turtle addedTurtle1 = addCar(0,-1);
					Turtle addedTurtle2 = addCar(1,-1);
					allCars.add(addedTurtle1);
					allCars.add(addedTurtle2);}
				else if (rndC2 <= UserPanel.Pof1Car) {
					lane = rndCar.nextInt(2);		// picks between 0 and 1 randomly
					Turtle addedTurtle = addCar(lane,-1);
					allCars.add(addedTurtle);}}}
		if (UserPanel.pedRho > 0) {
			if (UserPanel.pedsUp == true) {
				rndP = rndPed.nextDouble();
//				if (RoadBuilder.ticker % 200 == 0) { // Constant stream for testing
				if (rndP <=  UserPanel.Pof2Ped) {
					Ped addedPed1 = addPed(1);
					Ped addedPed2 = addPed(1);
					allPeds.add(addedPed1);
					allPeds.add(addedPed2);}
				else if (rndP <= UserPanel.Pof1Ped) {
					Ped addedPed = addPed(1);
					allPeds.add(addedPed);}}
			if (UserPanel.pedsDn == true) {
				rndP2 = rndPed.nextDouble();
				if (rndP2 <=  UserPanel.Pof2Ped) {
					Ped addedPed1 = addPed(-1);
					Ped addedPed2 = addPed(-1);
					allPeds.add(addedPed1);
					allPeds.add(addedPed2);}
				else if (rndP2 <= UserPanel.Pof1Ped) {
					Ped addedPed = addPed(-1);
					allPeds.add(addedPed);}}}
	}
	
	/**
	 * Used to add cars to edge
	 */
	@SuppressWarnings("unchecked")
	public Turtle addCar(int lane, int dir) {
		Context<Object> context = ContextUtils.getContext(this);
		ContinuousSpace<Object> space = (ContinuousSpace<Object>) context.getProjection("space");
//		Grid<Object> grid = (Grid) context.getProjection("grid");		
		
//		Turtle newTurtle = new Turtle(space,grid);
		Turtle newTurtle = new Turtle(space,lane,dir);
		context.add(newTurtle);
		if (dir == 1) {				//1 = going right, -1 = going left
			yPlacement = RoadBuilder.sidewalk + (double)lane * (RoadBuilder.laneW) + (RoadBuilder.laneW/2);
			space.moveTo(newTurtle,0,yPlacement);}
		else {
			yPlacement = RoadBuilder.sidewalk + RoadBuilder.roadW - (double)lane * (RoadBuilder.laneW) - (RoadBuilder.laneW/2);
			space.moveTo(newTurtle,RoadBuilder.roadL - 1,yPlacement);}
		return(newTurtle);
	}
	
	/**
	 * Used to add peds at x-walk
	 * @param direction: 1 walks up, -1 walks down
	 */
	@SuppressWarnings("unchecked")
	public Ped addPed(int direction) {
		Context<Object> context = ContextUtils.getContext(this);
		ContinuousSpace<Object> space = (ContinuousSpace<Object>) context.getProjection("space");
//		Grid<Object> grid = (Grid) context.getProjection("grid");
//		int dir0 = rndPed.nextInt(2); //picks between 0 (walking up) and 1 (walking down)
		double dir0 = direction == 1 ? 0 : 1;
		double yPlacement = dir0 * RoadBuilder.worldW;
		if (direction == -1) yPlacement -= 1e-15;
		
//		Ped newPed = new Ped(space,grid,direction);
		Ped newPed = new Ped(space,direction);
		context.add(newPed);
		space.moveTo(newPed,RoadBuilder.xWalkx,yPlacement);
		newPed.myLoc = space.getLocation(newPed);
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