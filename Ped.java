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
	private ArrayList<Ped> ahead;
	public ArrayList<Turtle> yielders;
	public  NdPoint myLoc;
	private	NdPoint endPt;
	private boolean go, go0, go1, go2, go3, curbed;
	private int goes, goes0, goes1, goes2, goes3;
	public  double[] v, dv, newV, frogger;
	public  double[] zero = new double[] {0,0};
	private	double xLoc, yLoc, maxV, endPtDist, endPtTheta;
	private double gap, gap0, gap1, gap2, gap3, critGap;
	private double side = RoadBuilder.sidewalk;
	private Random rnd = new Random();
	private double gapParamA = 6.2064; //TODO: email Brewer re: value for just convergent sets
	private double gapParamB = 0.942;  //TODO: ditto
	private double gapMin    = 1/(1+Math.exp(gapParamA));
	private double wS, etaS, wV, etaV, sigR;
	public	int age, dir;	// dir = 1 walks up, -1 walks down
	public	int crossing;	// 0=not yet, 1=waiting, 2=yes, 3=done
	public	double xTime;
	public	Turtle nearest, nearest0, nearest1, nearest2, nearest3;
	private double boxL = RoadBuilder.xWalkx - 2/RoadBuilder.spaceScale;
	private double boxR = RoadBuilder.xWalkx + 2/RoadBuilder.spaceScale;
		//2 here is more or less arbitrary, but has to match Turtle.stopBar variable
	
	//3-circle variables - from Helbing, et al (2000)
	//TODO: accT is strange - should it not vary with how far from maxV the ped is?
	private double accT  = 0.5/UserPanel.tStep;							//acceleration time
	private double m     = 80;											//avg ped mass in kg
	private double horiz = 5/RoadBuilder.spaceScale;					//distance at which peds affect each other
	private double A     = 2000*UserPanel.tStep/RoadBuilder.spaceScale;	//ped interaction constant (kg*space units/time units)
	private double B     = 0.08/RoadBuilder.spaceScale;					//ped distance interaction constant (space units)
	public  double r     = 0.3/RoadBuilder.spaceScale;					//ped radius (space units)
	private double k	 = 120000*UserPanel.tStep;						//wall force constant
	//TODO: figure out A & k wrt one or two factors of time (currently one like acceleration)
	
	//TODO: inroad() script/flag
	
	/**
	 * Scheduled method to calculate ped acceleration based on current state
	 */
	public void calc() {
		myLoc = space.getLocation(this);
		xLoc  = myLoc.getX();
		yLoc  = myLoc.getY();
		dv    = accel(myLoc,dir,curbed);
		newV  = sumV(v,dv);
		newV  = limitV(newV);
		
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
				if (xLoc + r + v[0] >= boxR && newV[0] < 0) { //avoid wiggles by box walls
					newV[0] = 0;}
				if (xLoc - r + v[0] <= boxL && newV[0] > 0) {
					newV[0] = 0;}
				break;
		case 1: dv = yield();
				newV = sumV(v,dv);
				if (dir == 1) {
					if (yLoc + newV[1] > side) {
						crossing = 2;}}
				else {
					if (yLoc + newV[1] < side + RoadBuilder.roadW) {
						crossing = 2;}}
				if (xLoc + r + v[0] >= boxR && newV[0] < 0) { //avoid wiggles by box walls
					newV[0] = 0;}
				if (xLoc - r + v[0] <= boxL && newV[0] > 0) {
					newV[0] = 0;}
				break;
		case 2: if (curbed == true) {
					dv = yield();
					newV = sumV(v,dv);}
				break;
		default: break;}
		
		newV = limitV(newV);
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
//						curbed = true;
						crossing = 1;}}
				else {
					if (yLoc - RoadBuilder.roadW - side < r || v[1] == 0) {
//						curbed = true;
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

	
	/**
	 * Determines value of curbed by calling gap() for each lane
	 * TODO: will need fundamental rewriting to take double threat into account
	 * @return output of accel() based on new value of curbed
	 */
	public double[] yield() {
		ArrayList<Turtle> approaching  = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching0 = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching1 = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching2 = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching3 = new ArrayList<Turtle>();		
//		ArrayList<Turtle> near = new ArrayList<Turtle>();
		//TODO: add double threat using theta to oncoming cars. if within range of same value, not seen
		
		nearest = null;
		xLoc = myLoc.getX();
		gap = gap0 = gap1 = gap2 = gap3 = RoadBuilder.roadL/2;
		goes0 = goes1 = goes2 = goes3 = 0;
		for (Turtle m : Scheduler.allCars) {
			double thisGap = xLoc - m.xLoc;
			int threat = m.dir * (int)Math.signum(thisGap);
			if (threat == 1) {
				approaching.add(m);}}
		if (!approaching.isEmpty()) {
			for (Turtle n : approaching) {
				if (n.dir == dir) {
					if (n.lane == 0) {
						approaching0.add(n);}
					else {
						approaching1.add(n);}}
				else {
					if (n.lane == 1) {
						approaching2.add(n);}
					else {
						approaching3.add(n);}}}
			if(!approaching0.isEmpty()) {
				for (Turtle o : approaching0) {
					double thisGap = Math.abs(xLoc - o.xLoc);
					if(thisGap < gap0) {
						gap0 = thisGap;
						nearest0 = o;}}}
			if(!approaching1.isEmpty()) {
				for (Turtle o : approaching1) {
					double thisGap = Math.abs(xLoc - o.xLoc);
					if(thisGap < gap1) {
						gap1 = thisGap;
						nearest1 = o;}}}
			if(!approaching2.isEmpty()) {
				for (Turtle o : approaching2) {
					double thisGap = Math.abs(xLoc - o.xLoc);
					if(thisGap < gap2) {
						gap2 = thisGap;
						nearest2 = o;}}}
			if(!approaching3.isEmpty()) {
				for (Turtle o : approaching3) {
					double thisGap = Math.abs(xLoc - o.xLoc);
					if(thisGap < gap3) {
						gap3 = thisGap;
						nearest3 = o;}}}}
		if (nearest0 != null) {
			while (goes0 == 0) {
				goes0 = lag(nearest0,0);}
			if (goes0 ==  1) {go0 = true;}
			if (goes0 == -1) {go0 = false;}}
		else {
			go0 = true;}
		if (nearest1 != null) {
			while (goes1 == 0) {
				goes1 = lag(nearest1,1);}
			if (goes1 ==  1) {go1 = true;}
			if (goes1 == -1) {go1 = false;}}
		else {
			go1 = true;}
		if (nearest2 != null) {
			while (goes2 == 0) {
				goes2 = lag(nearest2,2);}
			if (goes2 ==  1) {go2 = true;}
			if (goes2 == -1) {go2 = false;}}
		else {
			go2 = true;}
		if (nearest3 != null) {
			while (goes3 == 0) {
				goes3 = lag(nearest3,3);}
			if (goes3 ==  1) {go3 = true;}
			if (goes3 == -1) {go3 = false;}}		
		else {
			go3 = true;}
//		if (nearest != null) {
//			go = gap(nearest);
//			if (go==true) {
//				curbed = false;
//				crossing = 2;}}
		
		curbed = true;
		if (!approaching.isEmpty()) {
			if (go0 && go1 && go2 && go3) {
				curbed = false;
//				crossing = 2;
				}}
		else {
			curbed = false;
//			crossing = 2;
			}
		
		frogger = accel(myLoc,dir,curbed);
		return frogger;
	}
	
	
	//TODO: include estimation errors/delay
	public int lag(Turtle t, int ln) {
		goes = 0;
		double approachV, approachX, dist, xDist, yDist, TTCol, TTClear;
		//TODO: xTime can be calculated more precisely (compare to HCM eq 18-17)
		//TODO: make xTime lane-dependent for some peds
		double threatBeg, threatEnd;
		double maxVY = Math.abs(maxV*Math.cos(endPtTheta));
		xTime = accT + (endPtDist - side) / maxVY; 
		sigR = 0.01*UserPanel.tStep;	//standard deviation of relative approach rate 
										//TODO: should this vary?
		approachX = t.xLoc;
		xDist	  = Math.abs(xLoc - approachX);
		yDist	  = (double)ln*RoadBuilder.laneW + RoadBuilder.laneW/2;
		dist	  = space.getDistance(myLoc, t.myLoc);
		approachV = t.v;
		
		if (UserPanel.estErr == true) {
			etaS = rnd.nextGaussian();
			etaV = rnd.nextGaussian();
			wS = UserPanel.wien1*wS + UserPanel.wien2*etaS;
			wV = UserPanel.wien1*wV + UserPanel.wien2*etaV;
			approachV = t.v - dist*sigR*wV;
			dist  = dist*Math.exp(UserPanel.Vs*wS);
			xDist = Math.sqrt(dist*dist - yDist*yDist);}
		
		if (ln == 0) {
			threatBeg = 0;}
		else {
			threatBeg = accT + (ln*RoadBuilder.laneW/maxVY);}
//		threatEnd = accT + (ln+1)*RoadBuilder.laneW/maxVY;
		
		TTCol	= Math.abs(xDist/approachV);
		TTClear	= TTCol + t.length/approachV; //TODO: add radius of ped to this calculation
		//if (xTime < TTC) go = true;
		
		if (t.ying == 1) {
			goes = 1;}
		else {
			if (threatBeg + critGap < TTCol) {
				goes = 1;}
			else if (threatBeg > TTClear) {
				if (t.follower != null) {
					switch (ln) {
					case 0:	nearest0 = t.follower;
							break;
					case 1: nearest1 = t.follower;
							break;
					case 2: nearest2 = t.follower;
							break;
					case 3: nearest3 = t.follower;
							break;	
					default: break;}}
				else {
					goes = 1;}}
			else {
				goes = -1;}}
		return goes;
	}
	
	/**
	 * Calculates all forces acting on ped
	 * TODO: make peds blind to forces behind
	 * @param location
	 * @param direct
	 * @param curb
	 * @return
	 */
	public double[] accel(NdPoint location, int direct, boolean curb) {
		ahead = new ArrayList<Ped>();
		List<Double>  forcesX = new ArrayList<Double>();
		List<Double>  forcesY = new ArrayList<Double>();
		double xF, yF, curbF;
		double[] acc;
		xF = yF = curbF = 0;
		
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
		
		//calculate interactive forces
		//TODO: write code to make a threshold for interaction instead of the arbitrary horizon
		//TODO: make it so pedestrians only feel forces from things ahead of them
		for (Ped a : Scheduler.allPeds) {
			if (a != this) {
				NdPoint	otherLoc	= space.getLocation(a);
				double	otherY		= otherLoc.getY();
				double	visible		= Math.signum((double)dir*(otherY-yLoc));
				if (visible == 1) {		//peds only affected by those in front of them
					double	absDist		= space.getDistance(location, otherLoc);
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
						forcesY.add(interFy);}}}}
		
		//stop at curb if necessary
		if (curb == true) {
			if (direct == 1) {
				double dCurb = side - yLoc;
				curbF = -(A*Math.exp((r-dCurb)/B) + k*(r-dCurb))/m;}
			else {
				double dCurb = yLoc - side - RoadBuilder.roadW;
				curbF = (A*Math.exp((r-dCurb)/B) + k*(r-dCurb))/m;}
			forcesY.add(curbF);}

		//keep peds within crossing box TODO: make world bigger to hold more peds?
		double dBox, boxF;
		if (xLoc + r + v[0] >= boxR) {
			dBox = boxR - xLoc;
			boxF = -(A*Math.exp((r-dBox)/B) + k*(r-dBox))/m;
			
			forcesX.add(boxF);}
		if (xLoc - r - v[0] <= boxL) {
			dBox = xLoc - boxL;
			boxF = (A*Math.exp((r-dBox)/B) + k*(r-dBox))/m;
			forcesX.add(boxF);}
		
		//sum all forces
		for (Double b : forcesX) {
			xF += b;}
		for (Double c : forcesY) {
			yF += c;}
		acc = new double[] {xF, yF}; 
		if ((double)dir*Math.signum(acc[1]) == 1) {
			double foo = 0;}
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
	 * Keeps velocities within acceptable ranges
	 * @param input
	 * @return
	 */
	public double[] limitV(double[] input) {
		if (this.dir == 1) {
			if (input[1] < 0) {
				input[1] = 0;}}
		else {
			if (input[1] > 0) {
				input[1] = 0;}}
		
		double totalV = Math.sqrt(input[0]*input[0] + input[1]*input[1]);
		if (totalV > this.maxV) {
			double norm = this.maxV/totalV;
			input[0] = input[0]*norm;
			input[1] = input[1]*norm;}
		return input;
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
		if (dir == 1) endPt = new NdPoint(RoadBuilder.xWalkx + 1/RoadBuilder.spaceScale, RoadBuilder.worldW);
		else endPt = new NdPoint(RoadBuilder.xWalkx - 1/RoadBuilder.spaceScale, 0);
		
		double preGap0 = rnd.nextDouble();
		double preGap  = preGap0*(1-gapMin) + gapMin; //avoids negative values
		double critGapS = (gapParamA - Math.log((1/preGap) - 1))/gapParamB; //from Brewer 2006
		critGap = critGapS/UserPanel.tStep; //critical gap in simulation time units
		
		crossing = 0;
		curbed   = false;
		
		wS = etaS = rnd.nextGaussian();
		wV = etaV = rnd.nextGaussian();
		
		yielders = new ArrayList<Turtle>();
		
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
	@Parameter(usageName="v", displayName="Current yVel")
	public double getVel() {
		double vY = v[1];
		return vY;}
	@Parameter(usageName="crossing", displayName="Crossing?")
	public double getCrossing() {
		return crossing;}
	@Parameter(usageName="cg", displayName="Critical gap")
	public double getGap() {
		return critGap*UserPanel.tStep;}
}