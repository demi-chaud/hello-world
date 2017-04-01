package driving1;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Random;

import repast.simphony.context.Context;
import repast.simphony.engine.environment.RunEnvironment;
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
	static	ArrayList<Turtle.Conflict> allConf;
	static	ArrayList<Turtle.Crash> crashes;
	static  ArrayList<Turtle> killListC = new ArrayList<Turtle>();
	static  ArrayList<Ped>	 killListP = new ArrayList<Ped>();
	Random  rndCar = new Random(); //initiates random number generator for Poisson vehicle arrival
	Random  rndPed = new Random(); //ditto for peds so the two are independent
	Random	rndCAV = new Random(); //ditto for choosing connected/automated
	String	directory = "C:\\Users\\demi_chaud\\workspace\\driving1\\";
	DateFormat dateFormat = new SimpleDateFormat("MM-dd_HH-mm");
	double  rndC, rndP, rndC2, rndP2, yPlacement, thisTick;
	int     lane, dir;
	boolean carsYes, pedsYes;
	
	/**
	 * The scheduled method that contains all major steps
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
	
	/**
	 * Calculates Poisson arrival of agents, saves conflicts at end
	 */
	public void calc() {
		double	rndType1, rndType2, rndType3, rndType4;
		boolean bV2X1  = false;
		boolean bV2X2  = false;
		boolean bV2X3  = false;
		boolean bV2X4  = false;
		boolean bAut1  = false;
		boolean bAut2  = false;
		boolean bAut3  = false;
		boolean bAut4  = false;
		boolean bBoth1 = false;
		boolean bBoth2 = false;
		boolean bBoth3 = false;
		boolean bBoth4 = false;
		
		if (UserPanel.vehRho > 0) {
			rndC = rndCar.nextDouble();
			rndType1 = rndCAV.nextDouble();
			if (rndType1 < UserPanel.V2Xhi) { //random selects from [0,1)
				bV2X1 = true;}
			if (rndType1 >= UserPanel.autHi && rndType1 < UserPanel.bothLo) {
				bAut1 = true;}
			if (rndType1 >= UserPanel.bothLo && rndType1 < UserPanel.bothHi) {
				bBoth1 = true;}
			if (rndC <=  UserPanel.Pof2Car) {
				rndType2 = rndCAV.nextDouble();
				if (rndType2 < UserPanel.V2Xhi) { //random selects from [0,1)
					bV2X2 = true;}
				if (rndType2 >= UserPanel.autHi && rndType2 < UserPanel.bothLo) {
					bAut2 = true;}
				if (rndType2 >= UserPanel.bothLo && rndType2 < UserPanel.bothHi) {
					bBoth2 = true;}
				Turtle addedTurtle1 = addCar(0,1,bV2X1,bAut1,bBoth1);
				Turtle addedTurtle2 = addCar(1,1,bV2X2,bAut2,bBoth2);
				allCars.add(addedTurtle1);
				allCars.add(addedTurtle2);}
			else if (rndC <= UserPanel.Pof1Car) {
				lane = rndCar.nextInt(2);		// picks between 0 and 1 randomly
				Turtle addedTurtle = addCar(lane,1,bV2X1,bAut1,bBoth1);
				allCars.add(addedTurtle);}
			if (UserPanel.bothCar == true) {
				rndC2 = rndCar.nextDouble();
				rndType3 = rndCAV.nextDouble();
				if (rndType3 < UserPanel.V2Xhi) { //random selects from [0,1)
					bV2X3 = true;}
				if (rndType3 >= UserPanel.autHi && rndType3 < UserPanel.bothLo) {
					bAut3 = true;}
				if (rndType3 >= UserPanel.bothLo && rndType3 < UserPanel.bothHi) {
					bBoth3 = true;}
				if (rndC2 <=  UserPanel.Pof2Car) {
					rndType4 = rndCAV.nextDouble();
					if (rndType4 < UserPanel.V2Xhi) { //random selects from [0,1)
						bV2X4 = true;}
					if (rndType4 >= UserPanel.autHi && rndType4 < UserPanel.bothLo) {
						bAut4 = true;}
					if (rndType4 >= UserPanel.bothLo && rndType4 < UserPanel.bothHi) {
						bBoth4 = true;}
					Turtle addedTurtle3 = addCar(0,-1,bV2X3,bAut3,bBoth3);
					Turtle addedTurtle4 = addCar(1,-1,bV2X4,bAut4,bBoth4);
					allCars.add(addedTurtle3);
					allCars.add(addedTurtle4);}
				else if (rndC2 <= UserPanel.Pof1Car) {
					lane = rndCar.nextInt(2);		// picks between 0 and 1 randomly
					Turtle addedTurtle3 = addCar(lane,-1,bV2X3,bAut3,bBoth3);
					allCars.add(addedTurtle3);}}}
		if (UserPanel.pedRho > 0) {
			if (UserPanel.pedsUp == true) {
				rndP = rndPed.nextDouble();
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
		
		//write log of conflicts at end
		thisTick = RoadBuilder.clock.getTickCount();
		if (thisTick == UserPanel.simLength) { //TODO: add flag to only use this during batch runs
			String nP	= String.valueOf(UserPanel.pedRho) + "_";
			String nC	= String.valueOf(UserPanel.vehRho) + "_";
			String del	= String.valueOf(UserPanel.delayTs) + "_";
			String lim	= String.valueOf(UserPanel.sLimitKH);
			String thisRun = nP + nC + del + lim;
			Date date = new Date();
			String now = dateFormat.format(date) + "_";
			String fileName = directory + now + thisRun + ".csv";
			CSVWriter.writeCSV(fileName,allConf);}
	}
	
	/**
	 * Adds cars to edge
	 */
	@SuppressWarnings("unchecked")
	public Turtle addCar(int lane, int dir, boolean connected, boolean autonomous, boolean both) {
		Context<Object> context = ContextUtils.getContext(this);
		ContinuousSpace<Object> space = (ContinuousSpace<Object>) context.getProjection("space");
		double carW = UserPanel.carWidth;
		if (both == true) {
			connected  = true;
			autonomous = true;}
		Turtle newTurtle = new Turtle(space,lane,dir,connected,autonomous);
		context.add(newTurtle);
		if (dir == 1) {				//1 = going right, -1 = going left
			yPlacement = RoadBuilder.sidewalk + (double)lane * (RoadBuilder.laneW) + (RoadBuilder.laneW/2);
			space.moveTo(newTurtle,0,yPlacement);
			newTurtle.yLoc = yPlacement;			//if lane-change is added, move this within calc
			newTurtle.driverY = yPlacement + (double)dir*carW/6;}
		else {
			yPlacement = RoadBuilder.sidewalk + RoadBuilder.roadW - (double)lane * (RoadBuilder.laneW) - (RoadBuilder.laneW/2);
			space.moveTo(newTurtle,RoadBuilder.roadL - 1,yPlacement);
			newTurtle.yLoc = yPlacement;			//if lane-change is added, move this within calc
			newTurtle.driverY = yPlacement - (double)dir*carW/6;}
		return(newTurtle);
	}
	
	/**
	 * Adds peds at x-walk
	 * @param direction: 1 walks up, -1 walks down
	 */
	@SuppressWarnings("unchecked")
	public Ped addPed(int direction) {
		Context<Object> context = ContextUtils.getContext(this);
		ContinuousSpace<Object> space = (ContinuousSpace<Object>) context.getProjection("space");
		double dir0 = (direction == 1) ? 0 : 0.9999999;
		double yPlacement = dir0 * RoadBuilder.worldW;
		if (direction == -1) yPlacement -= 1e-15;
		
		Ped newPed = new Ped(space,direction);
		context.add(newPed);
		space.moveTo(newPed,RoadBuilder.xWalkx,yPlacement);
		newPed.myLoc = space.getLocation(newPed);
		return(newPed);
	}
	
	/**
	 * Initializes schedule agent to hold lists
	 */
	public Scheduler() {
		allCars = new ArrayList<Turtle>();
		allPeds = new ArrayList<Ped>();
		allConf = new ArrayList<Turtle.Conflict>();
		crashes = new ArrayList<Turtle.Crash>();
		RunEnvironment.getInstance().endAt(UserPanel.simLength); //TODO: add flag to only use this during batch runs
	}
}