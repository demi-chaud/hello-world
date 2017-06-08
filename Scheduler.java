package driving1;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Random;

import driving1.Turtle.Conflict;
import driving1.Turtle.Video;
import repast.simphony.context.Context;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.util.ContextUtils;

import driving1.RedLight.state;


/**
 * Class created to contain schedule.
 * Agents are created, they observe their surroundings, then move.
 * @author Darryl Michaud
 */

public class Scheduler extends Agent {
	
	static  ArrayList<Turtle> allCars;
	static  ArrayList<Ped>	 allPeds;
	static	ArrayList<Turtle.Conflict> allConf;
	static  ArrayList<Turtle> killListC = new ArrayList<Turtle>();
	static  ArrayList<Ped>	 killListP = new ArrayList<Ped>();
	static  ArrayList<RedLight> lights = RoadBuilder.lights;
	static	ArrayList<Turtle> passed = new ArrayList<Turtle>();
	static	ArrayList<Turtle> justPassed = new ArrayList<Turtle>();
	static	ArrayList<DensityPt> passed1 = new ArrayList<DensityPt>();
	static	ArrayList<DensityPt> passed2 = new ArrayList<DensityPt>();
	static	ArrayList<Double> speeds = new ArrayList<Double>();
	static	ArrayList<Double> densities = new ArrayList<Double>();
	static	ArrayList<Double[]> diagram = new ArrayList<Double[]>();
	Random  rndCar = new Random(); //initiates random number generator for Poisson vehicle arrival
	Random  rndPed = new Random(); //ditto for peds so the two are independent
	Random	rndCAV = new Random(); //ditto for choosing connected/automated
	String  homeDir = System.getProperty("user.home");
	String	directory = homeDir + "\\workspace\\driving1\\";
	DateFormat dateFormat = new SimpleDateFormat("MM-dd_HH-mm");
	double  rndC, rndP, rndC2, rndP2, yPlacement, thisTick;
	int     lane, dir, greenDur, amberDur, redDur;
	int		counter = 0;
	int		counter1 = 0;
	boolean carsYes, pedsYes;
	int		tSpan = (int)(15/UserPanel.tStep);
	int		prepT = (int)Math.ceil(60*RoadBuilder.roadL/UserPanel.sLimit);
	double	dxM = 5.;
	double	dx = dxM/RoadBuilder.spaceScale;
	
	
	
	/**
	 * The scheduled method that contains all major steps
	 */
	@ScheduledMethod(start = 1, interval = 1, priority = 1)
	public void doStuff() {
		RoadBuilder.flowSource.calc();
		carsYes = (allCars.size() > 0);
		pedsYes = (allPeds.size() > 0);
		boolean calcFun = UserPanel.calcFun;
		for (RedLight l : lights) {
			lightTick(l);}
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
		if (calcFun) {
			if (RoadBuilder.ticker > prepT) {
				diagramIt();}}
		if (!killListC.isEmpty()) {
			for (Turtle c : killListC) {
				c.die();}
			killListC = new ArrayList<Turtle>();}
		if (!killListP.isEmpty()) {
			for (Ped c : killListP) {
				c.die();}
			killListP = new ArrayList<Ped>();}
		if(!allConf.isEmpty()) {
			for (Conflict d : allConf) {
				int size = d.pedVid.size();
				if (size < 5) {
					double[] pedLoc = new double[2];
					d.ped.myLoc.toDoubleArray(pedLoc);
					d.pedVid.add(pedLoc);
					for (Video v : d.video) {
						double[] newLoc = new double[2];
						v.car.myLoc.toDoubleArray(newLoc);
						v.locs.add(newLoc);}}}}
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
//			if (UserPanel.pedsUp == true) {
				rndP = rndPed.nextDouble();
				if (rndP <=  UserPanel.Pof2Ped) {
					Ped addedPed1 = addPed(1);
					Ped addedPed2 = addPed(1);
					allPeds.add(addedPed1);
					allPeds.add(addedPed2);}
				else if (rndP <= UserPanel.Pof1Ped) {
					Ped addedPed = addPed(1);
					allPeds.add(addedPed);}
//				}
//			if (UserPanel.pedsDn == true) {
				rndP2 = rndPed.nextDouble();
				if (rndP2 <=  UserPanel.Pof2Ped) {
					Ped addedPed1 = addPed(-1);
					Ped addedPed2 = addPed(-1);
					allPeds.add(addedPed1);
					allPeds.add(addedPed2);}
				else if (rndP2 <= UserPanel.Pof1Ped) {
					Ped addedPed = addPed(-1);
					allPeds.add(addedPed);}}
//			}
		
		//write log of conflicts at end
		thisTick = RoadBuilder.clock.getTickCount();
		if (thisTick == UserPanel.simLength) { //TODO: add flag to only use this during batch runs
			String nP	= String.valueOf(UserPanel.pedRho) + "_";
			String nC	= String.valueOf(UserPanel.vehRho) + "_";
			String del	= String.valueOf(UserPanel.delayTs) + "_";
			String lim	= String.valueOf(UserPanel.sLimitKH);
			String thisRunC = nP + nC + del + lim;
			String thisRunD = nC + lim;
			Date date = new Date();
			String now = dateFormat.format(date) + "_";
			String confFileName = directory + now + thisRunC + ".csv";
			String diagFileName = directory + now + thisRunD + "fundDiagram.csv";
			
			CSVWriter.writeConfCSV(confFileName,allConf);
			if (UserPanel.calcFun) {
				CSVWriter.writeDiagCSV(diagFileName,diagram);}
			}
	}
	
	public void diagramIt() {
		justPassed.clear();
		double line = 2*RoadBuilder.xWalkx/3;
		for (Turtle t : allCars) {
			if (t.xLoc >= line - dx) {
				boolean inIt = false;
				for (DensityPt i : passed1) {
					if (i.car == t) {
						inIt = true;
						break;}}
				if (!inIt) {
					if (t.v != 0) {
						double t1 = (double)RoadBuilder.ticker - (t.xLoc - line + dx)/t.v;
						DensityPt thisPt = new DensityPt(t,t1);
						passed1.add(thisPt);}}
				if (t.xLoc > line) {
					justPassed.add(t);}}}
		ArrayList<DensityPt> pToRemove = new ArrayList<DensityPt>(); 
		for (DensityPt p : passed1) {
			if (p.car.xLoc > line) {
				if (p.car.v != 0) {
					double t2 = (double)RoadBuilder.ticker - (p.car.xLoc - line)/p.car.v;
					double dt = t2 - p.t1;
					densities.add(dt);
					pToRemove.add(p);}}}
		passed1.removeAll(pToRemove);
		if (!justPassed.isEmpty()) {
			ArrayList<Turtle> diff = new ArrayList<Turtle>();
			diff.addAll(justPassed);
			diff.removeAll(passed);
			passed.clear();
			passed.addAll(justPassed);
			for (Turtle s : diff) {
				speeds.add(s.v);}}
		counter++;
		if (counter == tSpan) {
			if (!speeds.isEmpty()) {
				double n = (double)speeds.size();
				double q = n*60; // veh/hr
				double invSum = 0;
				for (double v : speeds) {
					invSum += 1/v;}
				double u = 1/(invSum/n);
				double timeSum = 0;
				for (double t : densities) {
					timeSum += t;}
				double density = timeSum/((double)tSpan*dx);
				Double[] dataPt = {u,q,density};
				diagram.add(dataPt);
				speeds = new ArrayList<Double>();
				densities = new ArrayList<Double>();}
			counter = 0;}
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
		boolean blocked = false;
		for (Turtle t : allCars) {
			if (!t.equals(newTurtle)) {
				if (t.lane == lane) {
					if (t.dir == dir) {
						if (dir == 1) {
							if (t.xLoc < 0.1) {
								blocked = true;
								break;}}
						else {
							if (t.xLoc > RoadBuilder.roadL - 1.01) {
								blocked = true;
								break;}}}}}}
		if (blocked) newTurtle.v = 0;
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
	 * Runs red lights
	 */
	public void lightTick(RedLight l) {
		int cycleTick = (int)(thisTick % UserPanel.cycleTime);
		switch (l.myState) {
		case GREEN:
			if (cycleTick == UserPanel.greenDur) {
				l.myState = state.AMBER;
				l.timeInState = 0;}
			else l.timeInState += 1;
			break;
		case AMBER:
			if (cycleTick == UserPanel.amberDur) {
				l.myState = state.RED;
				l.timeInState = 0;}
			else l.timeInState += 1;
			break;
		case RED:
			if (cycleTick == UserPanel.redDur) {
				l.myState = state.GREEN;
				l.timeInState = 0;}
			else l.timeInState += 1;
			break;
		default: break;}	
	}
	
	public class DensityPt {
		Turtle car;
		double t1;
		DensityPt(Turtle turtle, double t) {
			car = turtle;
			t1 = t;}
	}
	
	
	/**
	 * Initializes schedule agent to hold lists
	 */
	public Scheduler() {
		allCars = new ArrayList<Turtle>();
		allPeds = new ArrayList<Ped>();
		allConf = new ArrayList<Turtle.Conflict>();
//		crashes = new ArrayList<Turtle.Crash>();
		RunEnvironment.getInstance().endAt(UserPanel.simLength); //TODO: add flag to only use this during batch runs
	}
}