package driving1;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.apache.commons.math3.util.FastMath;

import repast.simphony.parameter.Parameter;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;

/**
 * Pedestrian agent class
 * @author Darryl Michaud
 */
public class Ped extends Agent{
	private	ContinuousSpace<Object> space;
//	private	Grid<Object> grid;
	public  NdPoint myLoc;
	private	NdPoint endPt;
	private boolean go, curbed;
	public  double[] v, dv, newV, frogger;
	public  double[] zero = new double[] {0,0};
	private	double xLoc, yLoc, maxV, gap, endPtDist, endPtTheta;
	private double side = RoadBuilder.sidewalk;
	private Random rnd = new Random();
	private double wS, etaS, wV, etaV, sigR;
	public	int age, dir;	// dir = 1 walks up, -1 walks down
	public	int crossing;	// 0=not yet, 1=waiting, 2=yes, 3=done
	public	Turtle nearest;

	
	//3-circle variables - from Helbing, et al (2000)
	private double accT  = 0.5/UserPanel.tStep;							//acceleration time
	private double m     = 80;											//avg ped mass in kg
	private double horiz = 5/RoadBuilder.spaceScale;					//distance at which peds affect each other
	private double A     = 2000*UserPanel.tStep/RoadBuilder.spaceScale;	//ped interaction constant (kg*space units/time units)
	private double B     = 0.08/RoadBuilder.spaceScale;					//ped distance interaction constant (space units)
	public  double r     = 0.6/RoadBuilder.spaceScale;					//ped radius (space units)
	private double k	 = 120000*UserPanel.tStep;						//wall force constant
	//TODO: figure out A & k wrt one or two factors of time (currently one like acceleration)
	
	//TODO: inroad() script/flag
	
	public void calc() {
		myLoc = space.getLocation(this);
		xLoc  = myLoc.getX();
		yLoc  = myLoc.getY();
		dv    = accel(myLoc,dir,curbed);
		newV  = sumV(v,dv);
		
		switch (crossing) {
		case 0: if (dir == 1) {
					if (yLoc + newV[1] >= side - r) {
						curbed = true;
						dv = accel(myLoc,dir,curbed);
						newV = sumV(v,dv);}}
				else {
					if (yLoc + newV[1] <= side + RoadBuilder.roadW + r) {
						curbed = true;
						dv = accel(myLoc,dir,curbed);
						newV = sumV(v,dv);}}
				break;
		case 1: dv = yield();
				newV = sumV(v,dv);
				break;		
		default: break;}
		
		if (this.dir == 1) {
			if (newV[1] < 0) {
				newV[1] = 0;}}
		else {
			if (newV[1] > 0) {
				newV[1] = 0;}}
		
		double totalV = Math.sqrt(newV[0]*newV[0] + newV[1]*newV[1]);
		if (totalV > maxV) {
			double norm = maxV/totalV;
			newV[0] = newV[0]*norm;
			newV[1] = newV[1]*norm;}
	}
	
	
	
	/*
	 * Scheduled movement method
	 */
	public void walk() {
		v = newV;
		move(myLoc,v);
		yLoc = myLoc.getY();
		switch (crossing) {
		case 0:	if (dir == 1) {
					if (side - yLoc < r || v[1] == 0) {
						curbed = true;
						crossing = 1;}}
				else {
					if (yLoc - RoadBuilder.roadW - side < r || v[1] == 0) {
						curbed = true;
						crossing = 1;}}
				break;
		case 2: if (dir == 1) {
					if (yLoc >= side + RoadBuilder.roadW) crossing = 3;}
				else {
					if (yLoc <= side) crossing = 3;}
				break;
		default: break;}
		age ++;
	}
	
	
	public double[] accel(NdPoint location, int direct, boolean curb) {
		List<Double>  forcesX = new ArrayList<Double>();
		List<Double>  forcesY = new ArrayList<Double>();
		double xF = 0;
		double yF = 0;
		double[] acc;
		
		//calculate heading to endpoint
		endPtDist  = space.getDistance(location, endPt); 
		double endPtDelX  = endPt.getX()-location.getX();
		endPtTheta = FastMath.asin((double)direct*endPtDelX/endPtDist);
		if (direct == -1) {
			endPtTheta += Math.PI;}
		
		//calculate motive force
		Double motFx = (maxV*Math.sin(endPtTheta) - v[0])/accT;
		Double motFy = (maxV*Math.cos(endPtTheta) - v[1])/accT;
		forcesX.add(motFx);
		forcesY.add(motFy);
		
		//TODO: add walls to the ped waiting area, maybe add line to make ppl walk if those near them decide to
		
		//calculate interactive forces
		//TODO: write code to make a threshold for interaction instead of the arbitrary horizon
		for (Ped a : Scheduler.allPeds) {
			if (a != this) {
				NdPoint otherLoc = space.getLocation(a);
				double absDist = space.getDistance(location, otherLoc);
				if (absDist < horiz) {
					double delX    = location.getX()-otherLoc.getX();
					double delY    = location.getY()-otherLoc.getY();
					double delXabs = Math.abs(delX);
					double signFx  = Math.signum(delX);
					double signFy  = Math.signum(delY);
					double theta   = FastMath.asin(delXabs/absDist);
					double rij     = r + a.r;
					Double interFx = signFx*A*Math.exp((rij-absDist)/B)*Math.sin(theta)/m;
					Double interFy = signFy*A*Math.exp((rij-absDist)/B)*Math.cos(theta)/m;
					forcesX.add(interFx);
					forcesY.add(interFy);}}}
		
		//stop at curb if necessary
		if (curb == true) {
			Double curbF;
			if (direct == 1) {
				double dCurb = side - yLoc;
				curbF = -(A*Math.exp((r-dCurb)/B) + k*(r-dCurb))/m;}
			else {
				double dCurb = yLoc - side - RoadBuilder.roadW;
				curbF = (A*Math.exp((r-dCurb)/B) + k*(r-dCurb))/m;}
			forcesY.add(curbF);}

		//sum all forces
		for (Double b : forcesX) {
			xF += b;}
		for (Double c : forcesY) {
			yF += c;}
		acc = new double[] {xF, yF}; 
		return acc;
	}

	
	
	/**
	 * Moves peds and updates context
	 * @param loc
	 * overloaded. optional parameters:
	 * @param displacement
	 * @param destination
	 */
	public void move(NdPoint loc, double[] displacement) {
		double yl = loc.getY();
		if (yl + displacement[1] > RoadBuilder.worldW || yl + displacement[1] < 0) {
			Scheduler.killListP.add(this);}
		else if (displacement != zero) {	
			space.moveByDisplacement(this,displacement);
			myLoc = space.getLocation(this);
//			grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
		}
	}
	public void move(NdPoint loc, NdPoint destination) {
		double xd = destination.getX();
		double yd = destination.getY();
		if (yd > RoadBuilder.worldW || yd < 0) {
			Scheduler.killListP.add(this);}
		else if (loc != destination) {
				space.moveTo(this,xd,yd);
				myLoc = space.getLocation(this);
//				grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
		}
	}
	
	/**
	 * Peds call this to determine if there is room to cross
	 * TODO: Currently only works for direction = 1
	 * TODO: will need fundamental rewriting to take double threat into account
	 * TODO: need to add cars coming from other direction
	 */
	
	public double[] yield() {
		ArrayList<Turtle> approaching = new ArrayList<Turtle>();
//		ArrayList<Turtle> approaching0 = new ArrayList<Turtle>();
//		ArrayList<Turtle> approaching1 = new ArrayList<Turtle>();
//		ArrayList<Turtle> approaching2 = new ArrayList<Turtle>();
//		ArrayList<Turtle> approaching3 = new ArrayList<Turtle>();		
//		ArrayList<Turtle> near = new ArrayList<Turtle>();
		//TODO: add double threat using theta to oncoming cars. if within range of same value, not seen
		
		nearest = null;
		xLoc = myLoc.getX();
		gap = RoadBuilder.roadL/2;
		for (Turtle m : Scheduler.allCars) {
			double thisGap = xLoc - m.xLoc;
			int threat = m.dir * (int)Math.signum(thisGap);
			if (threat == 1) {
				approaching.add(m);}}
		if (!approaching.isEmpty()) {
//			for (Turtle n : approaching) {
//				if (n.lane == 0) {
//					approaching0.add(n);}
//				else if (n.lane == 1) {
//					approaching1.add(n);}}
//			if(!approaching0.isEmpty()) {
				for (Turtle o : approaching) {
					double thisGap = Math.abs(xLoc - o.xLoc);
					if(thisGap < gap) {
						gap = thisGap;
						nearest = o;}}}
		if (nearest != null) {
			go = gap(nearest);
			if (go==true) {
				curbed = false;
				crossing = 2;}
			}
		else {
			curbed = false;
			crossing = 2;}
		frogger = accel(myLoc,dir,curbed);
		return frogger;
	}
	
	//TODO: include estimation errors
	public boolean gap(Turtle t) {
		go = false;
//		int approachL;
		double approachV, approachX, dist, TTC;
		//TODO: xTime can be calculated more precisely (compare to HCM eq 18-17)
		//TODO: make xTime lane-dependent for some peds
		double xTime = accT + (endPtDist - side) / Math.abs(maxV*Math.cos(endPtTheta)); 
		sigR = 0.01*UserPanel.tStep; //standard deviation of relative approach rate TODO: should this vary?
		
		
		approachX = t.xLoc;
		dist	  = Math.abs(xLoc - approachX); 
		approachV = t.v;
//		approachL = t.lane;
		
		if (UserPanel.estErr == true) {
			etaS = rnd.nextGaussian();
			etaV = rnd.nextGaussian();
			wS = UserPanel.wien1*wS + UserPanel.wien2*etaS;
			wV = UserPanel.wien1*wV + UserPanel.wien2*etaV;
			approachV = t.v - dist*sigR*wV;
			dist = dist*Math.exp(UserPanel.Vs*wS);}
		
		TTC  = dist/approachV;
		if (xTime < TTC) go = true;
		
		return go;
	}
	
	/**
	 * Adds 2-dimensional arrays
	 * @param  a = double[2]
	 * @param  b = double[2]
	 * @return c = double[2]
	 */
	public double[] sumV(double[] a, double[] b) {
		double[] c = new double[2];
		for (int i = 0; i < 2; i++) {
			c[i] = a[i] + b[i];}
		return c;
	}
	
	//TODO: give x-walk width, distribute peds
	
	/**
	 * Creates pedestrian agents and initializes values
	 * Called by scheduler in Agent.java
	 * @param contextSpace
	 * @param contextGrid
	 * @param direction
	 */
//	public Ped(ContinuousSpace<Object> contextSpace, Grid<Object> contextGrid, int direction) {
	public Ped(ContinuousSpace<Object> contextSpace, int direction) {
		space = contextSpace;
//		grid  = contextGrid;
//		v     = RoadBuilder.pedVavg * 1000 / 3600;
		maxV  = UserPanel.pedVavg;
		dir   = direction; // 1 moves up, -1 moves down
		v     = new double[] {0,(double)dir*.5*maxV};
		if (dir == 1) endPt = new NdPoint(RoadBuilder.xWalkx + 2/RoadBuilder.spaceScale, RoadBuilder.worldW);
		else endPt = new NdPoint(RoadBuilder.xWalkx - 2/RoadBuilder.spaceScale, 0);
		crossing = 0;
		curbed   = false;
		wS = etaS = rnd.nextGaussian();
		wV = etaV = rnd.nextGaussian();
		age = 0;
	}
	
	/**
	 * Getter for identification
	 */
	@Override
	public int isPed() {
		return 1;}
	
	/**
	 * Parameter declarations for probe
	 */
	@Parameter(usageName="v", displayName="Current vel")
	public double[] getVel() {
		System.out.println(nearest);
		return v;}
	@Parameter(usageName="crossing", displayName="Crossing?")
	public double getCrossing() {
		return crossing;}
}