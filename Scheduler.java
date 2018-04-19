package driving1;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Random;
import java.io.Console;
import java.io.File;

import driving1.Turtle.Conflict;
import driving1.Turtle.Video;
import repast.simphony.context.Context;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.parameter.Parameters;
import repast.simphony.parameter.SweeperProducer;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.batch.*;
import repast.simphony.util.ContextUtils;
import driving1.RedLight.state;


/**
 * Class created to contain schedule.
 * Agents are created, they observe their surroundings, then move.
 * @author Darryl Michaud
 */

public class Scheduler extends Agent {
	
	public  ArrayList<Turtle> allCars = new ArrayList<Turtle>();
	public  ArrayList<Ped>	 allPeds = new ArrayList<Ped>();
	public	ArrayList<Turtle.Conflict> allConf = new ArrayList<Turtle.Conflict>();
	public  ArrayList<Turtle.Crash> allCrash = new ArrayList<Turtle.Crash>();
	public  ArrayList<Turtle> killListC = new ArrayList<Turtle>();
	public  ArrayList<Ped>	 killListP = new ArrayList<Ped>();
	public  ArrayList<RedLight> lights = RoadBuilder.lights;
	//public	ArrayList<Turtle> passed = new ArrayList<Turtle>();
	public	ArrayList<Turtle> justPassed = new ArrayList<Turtle>();
	public	ArrayList<DensityPt> passed = new ArrayList<DensityPt>();
	public	ArrayList<DensityPt> passed2 = new ArrayList<DensityPt>();
	public	ArrayList<Double> dxTimes = new ArrayList<Double>();
	public	ArrayList<Double[]> diagram = new ArrayList<Double[]>();
	Random  rndCar = new Random(); //initiates random number generator for Poisson vehicle arrival
	Random  rndPed = new Random(); //ditto for peds so the two are independent
	Random	rndCAV = new Random(); //ditto for choosing connected/automated
	String  homeDir = System.getProperty("user.home");
	String	directory = homeDir + "\\Desktop\\thesis\\driving1\\results\\";
	//String	directory = homeDir + "\\workspace\\driving1\\results\\";
	DateFormat dateFormat = new SimpleDateFormat("MM-dd_HH-mm");
	double  rndC, rndP, rndC2, rndP2, yPlacement;
	public static double thisTick;
	int     lane, dir, greenDur, amberDur, redDur;
	int		counter = 0;
	int		counter1 = 0;
	boolean carsYes, pedsYes;
	int		tSpan = UserPanel.calcTSpan;
	int		tSpanS = (int)UserPanel.calcTSpanS;
	int		prepT = (int)Math.ceil(2*RoadBuilder.roadL/RoadBuilder.panel.sLimit);
	double	dxM = 5.;
	double	dx = dxM/RoadBuilder.spaceScale;
	public int nXing = 0;
	public LocalTime endTime;
	
	/**
	 * The scheduled method that contains all major steps
	 */
	@ScheduledMethod(start = 1, interval = 1, priority = 1)
	public void doStuff() {
		LocalTime nowTime = LocalTime.now();
		if (nowTime.compareTo(endTime) > 1) {
			System.out.println(RoadBuilder.panel.paramStr + "aborted after 20min");
			RunEnvironment.getInstance().endRun();}
		if (RoadBuilder.flowSource.allCars.size() > 2000) {
			System.out.println(RoadBuilder.panel.paramStr + "got stuck and aborted");
			RunEnvironment.getInstance().endRun();}
		if (RoadBuilder.panel.deathKnell) {
			RunEnvironment.getInstance().endRun();}
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
			int nPeds = RoadBuilder.flowSource.allPeds.size();
			if (nPeds > 20) {
				nXing = 0;
				for (Ped p : RoadBuilder.flowSource.allPeds) {
					if (p.crossing == 2) {
						nXing++;}}}
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
				c.blockedPeds.clear();
				c.die();}
			killListC = new ArrayList<Turtle>();}
		if (!killListP.isEmpty()) {
			for (Ped c : killListP) {
				c.yielders.clear();
				c.die();}
			killListP = new ArrayList<Ped>();}
//		if(!allConf.isEmpty()) {
//			for (Conflict d : allConf) {
//				int size = d.pedVid.size();
//				if (size < 5) {
//					double[] pedLoc = new double[2];
//					d.ped.myLoc.toDoubleArray(pedLoc);
//					d.pedVid.add(pedLoc);
//					for (Video v : d.video) {
//						double[] newLoc = new double[2];
//						v.car.myLoc.toDoubleArray(newLoc);
//						v.locs.add(newLoc);}}}}
		RoadBuilder.ticker ++;
	}
	
	/* Button method to create car for testing */
	public void forceCar() {
		lane = 0;
		Turtle addedTurtle = addCar(lane,1,false,false,false);
		allCars.add(addedTurtle);
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
		
		if (RoadBuilder.panel.vehRho > 0) {
			rndC = rndCar.nextDouble();
			rndType1 = rndCAV.nextDouble();
			if (rndType1 < RoadBuilder.panel.V2Xhi) { //random selects from [0,1)
				bV2X1 = true;}
			if (rndType1 >= RoadBuilder.panel.autLo && rndType1 < RoadBuilder.panel.bothLo) {
				bAut1 = true;}
			if (rndType1 >= RoadBuilder.panel.bothLo && rndType1 < RoadBuilder.panel.bothHi) {
				bBoth1 = true;}
			if (rndC <=  RoadBuilder.panel.Pof2Car) {
				rndType2 = rndCAV.nextDouble();
				if (rndType2 < RoadBuilder.panel.V2Xhi) { //random selects from [0,1)
					bV2X2 = true;}
				if (rndType2 >= RoadBuilder.panel.autLo && rndType2 < RoadBuilder.panel.bothLo) {
					bAut2 = true;}
				if (rndType2 >= RoadBuilder.panel.bothLo && rndType2 < RoadBuilder.panel.bothHi) {
					bBoth2 = true;}
				Turtle addedTurtle1 = addCar(0,1,bV2X1,bAut1,bBoth1);
				Turtle addedTurtle2 = addCar(1,1,bV2X2,bAut2,bBoth2);
				allCars.add(addedTurtle1);
				allCars.add(addedTurtle2);}
			else if (rndC <= RoadBuilder.panel.Pof1Car) {
				lane = rndCar.nextInt(2);		// picks between 0 and 1 randomly
				Turtle addedTurtle = addCar(lane,1,bV2X1,bAut1,bBoth1);
				allCars.add(addedTurtle);}
			if (UserPanel.bothCar == true) {
				rndC2 = rndCar.nextDouble();
				rndType3 = rndCAV.nextDouble();
				if (rndType3 < RoadBuilder.panel.V2Xhi) { //random selects from [0,1)
					bV2X3 = true;}
				if (rndType3 >= RoadBuilder.panel.autLo && rndType3 < RoadBuilder.panel.bothLo) {
					bAut3 = true;}
				if (rndType3 >= RoadBuilder.panel.bothLo && rndType3 < RoadBuilder.panel.bothHi) {
					bBoth3 = true;}
				if (rndC2 <=  RoadBuilder.panel.Pof2Car) {
					rndType4 = rndCAV.nextDouble();
					if (rndType4 < RoadBuilder.panel.V2Xhi) { //random selects from [0,1)
						bV2X4 = true;}
					if (rndType4 >= RoadBuilder.panel.autLo && rndType4 < RoadBuilder.panel.bothLo) {
						bAut4 = true;}
					if (rndType4 >= RoadBuilder.panel.bothLo && rndType4 < RoadBuilder.panel.bothHi) {
						bBoth4 = true;}
					Turtle addedTurtle3 = addCar(0,-1,bV2X3,bAut3,bBoth3);
					Turtle addedTurtle4 = addCar(1,-1,bV2X4,bAut4,bBoth4);
					allCars.add(addedTurtle3);
					allCars.add(addedTurtle4);}
				else if (rndC2 <= RoadBuilder.panel.Pof1Car) {
					lane = rndCar.nextInt(2);		// picks between 0 and 1 randomly
					Turtle addedTurtle3 = addCar(lane,-1,bV2X3,bAut3,bBoth3);
					allCars.add(addedTurtle3);}}}
		if (RoadBuilder.panel.pedRho > 0) {
			rndP = rndPed.nextDouble();
			if (rndP <=  RoadBuilder.panel.Pof2Ped) {
				Ped addedPed1 = addPed(1);
				Ped addedPed2 = addPed(1);
				allPeds.add(addedPed1);
				allPeds.add(addedPed2);}
			else if (rndP <= RoadBuilder.panel.Pof1Ped) {
				Ped addedPed = addPed(1);
				allPeds.add(addedPed);}
			rndP2 = rndPed.nextDouble();
			if (rndP2 <=  RoadBuilder.panel.Pof2Ped) {
				Ped addedPed1 = addPed(-1);
				Ped addedPed2 = addPed(-1);
				allPeds.add(addedPed1);
				allPeds.add(addedPed2);}
			else if (rndP2 <= RoadBuilder.panel.Pof1Ped) {
				Ped addedPed = addPed(-1);
				allPeds.add(addedPed);}}
		
		//write log of conflicts at end
		thisTick = RoadBuilder.clock.getTickCount();
		if (thisTick == UserPanel.simLength) {
			System.out.println(RoadBuilder.panel.paramStr + " - END");
			String nP	= "p" + String.valueOf((int)RoadBuilder.panel.pedRho) + "_";
			String nC	= "v" + String.valueOf((int)RoadBuilder.panel.vehRho) + "_";
			String lim	= "s" + String.valueOf((int)RoadBuilder.panel.sLimitKH) + "_";
			String perc = "d" + String.valueOf((int)RoadBuilder.panel.hPercLimM) + "_";
			String dur  = "h" + String.valueOf((int)UserPanel.simHours) + "_";
			String bools = String.valueOf(RoadBuilder.panel.estErr) + '.' + String.valueOf(RoadBuilder.panel.BRT) + '.' + 
					String.valueOf(RoadBuilder.panel.inclDist) + '.' + String.valueOf(RoadBuilder.panel.inclObstruct);
			String thisRunC = nP + nC + lim + perc + dur + bools;
			String thisRunD = nP + nC + lim + perc + bools;
//			String percs = String.valueOf((int)RoadBuilder.panel.percV2X) + '.' + String.valueOf((int)RoadBuilder.panel.percAuto) +
//					'.' + String.valueOf((int)RoadBuilder.panel.percBoth);
//			String thisRunC = nP + nC + lim + perc + dur + percs;
//			String thisRunD = nP + nC + lim + perc + percs;
			Date date = new Date();
			String now = dateFormat.format(date) + "_";
			String fileString = now + thisRunC;
			String confFileName = directory + fileString + ".csv";
			String diagFileName = directory + now + thisRunD + "_fundDiagram.csv";
			if (!UserPanel.calcFun) {
				File dir = new File(directory);
				ArrayList<String> fNames = new ArrayList<String>();
				ArrayList<String> allNames = new ArrayList<String>(Arrays.asList(dir.list()));
				for (String aName : allNames) {
					if (aName.contains(fileString)) {
						fNames.add(aName);}}
				if (fNames.size() == 0) {
					CSVWriter.writeConfCSV(confFileName,allConf);}
				else {
					boolean taken = true;
					int dupes = 1;
					while (taken) {
						String newFileName = fileString + '_' + String.valueOf(dupes) + ".csv";
						if (!fNames.contains(newFileName)) {
							String newFullName = directory + newFileName;
							CSVWriter.writeConfCSV(newFullName,allConf);
							taken = false;}
						dupes++;}}}
			else {
				CSVWriter.writeDiagCSV(diagFileName,diagram);}
			RunEnvironment.getInstance().endRun();}
	}
	
	public void diagramIt() {
		double line = 3*RoadBuilder.roadL/7;
		for (Turtle t : allCars) {
			if (t.xLoc >= line - dx && t.xLoc < line) {
				boolean inIt = false;
				for (DensityPt i : passed) {
					if (i.car == t) {
						inIt = true;
						break;}}
				if (!inIt) {
					if (t.v != 0) {
						double t1 = (double)RoadBuilder.ticker - (t.xLoc - line + dx)/t.v;
						DensityPt thisPt = new DensityPt(t,t1);
						passed.add(thisPt);}
					else {
						double t1 = (double)RoadBuilder.ticker;
						DensityPt thisPt = new DensityPt(t,t1);
						passed.add(thisPt);}}}}
		ArrayList<DensityPt> pToRemove = new ArrayList<DensityPt>();
		for (DensityPt d : passed) {
			Turtle dCar = d.car;
			if (dCar.xLoc >= line) {
				if (dCar.v != 0) {
					double t2 = (double)RoadBuilder.ticker - (dCar.xLoc - line)/dCar.v;
					double dt = t2 - d.t1;
					dxTimes.add(dt);}
				else {
					double t2 = (double)RoadBuilder.ticker;
					double dt = t2 - d.t1;
					dxTimes.add(dt);}
				pToRemove.add(d);}}
		passed.removeAll(pToRemove);
		counter++;
		if (counter == tSpan) {
			if (!dxTimes.isEmpty()) {
				double n = (double)dxTimes.size();
				double q0 = 3600/tSpanS;
				double q = n*q0; // veh/hr
				double invSum = 0;
				double timeSum = 0;
				for (double dt : dxTimes) {
					invSum += (dt*UserPanel.tStep)/dxM;
					timeSum += dt;}
				double u = 1/(invSum/n); // m/s
				double density = timeSum/((double)tSpan*dxM);
				Double[] dataPt = {u,q,density};
				diagram.add(dataPt);
				dxTimes = new ArrayList<Double>();}
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
//		boolean blocked = false;
//		for (Turtle t : allCars) {
//			if (!t.equals(newTurtle)) {
//				if (t.lane == lane) {
//					if (t.dir == dir) {
//						if (dir == 1) {
//							if (t.xLoc < 0.1) {
//								blocked = true;
//								break;}}
//						else {
//							if (t.xLoc > RoadBuilder.roadL - 1.01) {
//								blocked = true;
//								break;}}}}}}
//		if (blocked) newTurtle.v = 0;
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
		int cycleTick = (int)(thisTick % RoadBuilder.panel.cycleTime);
		switch (l.myState) {
		case GREEN:
			if (cycleTick == RoadBuilder.panel.greenDur) {
				l.myState = state.AMBER;
				l.timeInState = 0;}
			else l.timeInState += 1;
			break;
		case AMBER:
			if (cycleTick == RoadBuilder.panel.amberDur) {
				l.myState = state.RED;
				l.timeInState = 0;}
			else l.timeInState += 1;
			break;
		case RED:
			if (cycleTick == RoadBuilder.panel.redDur) {
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
		allCrash = new ArrayList<Turtle.Crash>();
		LocalTime startTime = LocalTime.now();
		endTime = startTime.plusMinutes(20);
	}
}