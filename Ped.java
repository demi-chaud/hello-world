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
	private List<Double> forcesX, forcesY; 
	private	NdPoint endPt;
	private Random rnd = new Random();
	//TODO: make sure there's a separate Wiener process for every single observation (think through which are related)
	private boolean curbed, boxed;
	private int age;
	private	double endPtDist, endPtTheta, critGap;
	private double side	= RoadBuilder.sidewalk;
	private double wS, etaS, wV, etaV, sigR;	//errors
	private double m, horiz, A, B, k, r;  		//interactive force constants (accT is also)
	private double boxL = RoadBuilder.xWalkx - 2/RoadBuilder.spaceScale;
	private double boxR = RoadBuilder.xWalkx + 2/RoadBuilder.spaceScale;
		//2 here is more or less arbitrary, but has to match Turtle.stopBar variable
	public ArrayList<Turtle> yielders;
	public Turtle nearest, nearest0, nearest1, nearest2, nearest3;
	public NdPoint myLoc;
	public double[] v, dv, newV;
	public double xTime, accT, maxV, xLoc, yLoc, whichSide;
	public int dir;			// dir = 1 walks up, -1 walks down
	public int crossing;	// 0=not yet, 1=waiting, 2=yes, 3=done
	
	/**
	 * Calculates ped acceleration
	 * Sets newV, changes value of crossing and curbed in some instances
	 */
	public void calc() {
		myLoc = space.getLocation(this);
		xLoc  = myLoc.getX();
		yLoc  = myLoc.getY();
		dv    = accel(myLoc,dir);
		newV  = sumV(v,dv);
		if (age > 1) {
			if (xLoc + r + newV[0] > boxR || xLoc - r + newV[0] < boxL){
				boxed = true;}
			else {boxed = false;}}
		else {boxed = false;}
//		newV  = limitV(newV);
		
		switch (crossing) {
		case 0: if (dir == 1) {
					if (curbed == true) {
						crossing = 1;}
					if (yLoc + newV[1] >= side - r) {
						curbed = true;
						dv = yield();
						newV = sumV(v,dv);}}
				else {
					if (curbed == true) {
						crossing = 1;
					}
					if (yLoc + newV[1] <= side + RoadBuilder.roadW + r) {
						curbed = true;
						dv = yield();
						newV = sumV(v,dv);}}
				break;
		case 1: dv = yield();
				newV = sumV(v,dv);
				if (dir == 1) {
					if (yLoc + newV[1] > side) {
						newV[0]  = 0;
						crossing = 2;}}
				else {
					if (yLoc + newV[1] < side + RoadBuilder.roadW) {
						newV[0]  = 0;
						crossing = 2;}}
				break;
		case 2: if (curbed == true) {
					dv	 = yield();
					newV = sumV(v,dv);}
				break;
		default: break;}
		
		if (v[0] != 0) {		//avoid bouncing off walls
			double whichDir = Math.signum(v[0]);
			if (whichSide == -whichDir && Math.signum(newV[0]) == -whichDir) {
				newV[0] = 0;}}
		newV = limitV(newV);
	}
	
	
	/*
	 * Moves peds based on results of calc()
	 * Sets value of crossing in some instances
	 */
	public void walk() {
		v = newV;
		move(myLoc,v);
		xLoc = myLoc.getX();
		yLoc = myLoc.getY();
		switch (crossing) {
		case 0:	if (dir == 1) {
					if (side - yLoc < r) {
						crossing = 1;}}
				else {
					if (yLoc - RoadBuilder.roadW - side < r) {
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
	 * Determines value of curbed by calling lag() and/or gap() for each lane
	 * @return output of accel() based on new value of curbed
	 */
	public double[] yield() {
		ArrayList<Turtle> approaching  = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching0 = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching1 = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching2 = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching3 = new ArrayList<Turtle>();		
		double gap0, gap1, gap2, gap3;
		double[] frogger;
		boolean go0, go1, go2, go3;
		go0 = go1 = go2 = go3 = false;
		int goes0, goes1, goes2, goes3;
		goes0 = goes1 = goes2 = goes3 = 0;
		nearest	= null;
		gap0 = gap1 = gap2 = gap3 = RoadBuilder.roadL/2;
		
		//find nearest car in each lane
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
		
		//decide crossing decision for each lane
		if (nearest0 != null) {
			goes0 = lag(nearest0,0);
			while (goes0 == 0) {
				goes0 = gap(nearest0,0);}
			if (goes0 ==  1) {go0 = true;}
			if (goes0 == -1) {go0 = false;}}
		else {
			go0 = true;}
		if (nearest1 != null) {
			goes1 = lag(nearest1,1);
			while (goes1 == 0) {
				goes1 = gap(nearest1,1);}
			if (goes1 ==  1) {go1 = true;}
			if (goes1 == -1) {go1 = false;}}
		else {
			go1 = true;}
		if (nearest2 != null) {
			goes2 = lag(nearest2,2);
			while (goes2 == 0) {
				goes2 = gap(nearest2,2);}
			if (goes2 ==  1) {go2 = true;}
			if (goes2 == -1) {go2 = false;}}
		else {
			go2 = true;}
		if (nearest3 != null) {
			goes3 = lag(nearest3,3);
			while (goes3 == 0) {
				goes3 = gap(nearest3,3);}
			if (goes3 ==  1) {go3 = true;}
			if (goes3 == -1) {go3 = false;}}		
		else {
			go3 = true;}
		
		//tally crossing decisions to give final result
		curbed = true;
		if (!approaching.isEmpty()) {
			if (go0 && go1 && go2 && go3) {
				curbed = false;}}
		else {
			curbed = false;}
		
		frogger = accel(myLoc,dir);
		return frogger;
	}
	
	/**
	 * Calculates whether or not pedestrian will go before next car
	 * If that car is passed, reassigns to following car
	 * @param t  - next car
	 * @param ln - lane car is in
	 * @return:	-1=don't go; 0=look to next car; 1=go
	 */
	public int lag(Turtle t, int ln) {
		//TODO: include estimation errors/delay
		double approachV, approachX, dist, xDist, yDist, TTCol, TTClear;
		double threatBeg; //, threatEnd;
		double maxVY = Math.abs(maxV*Math.cos(endPtTheta));
		int	goes = 0;
		//TODO: xTime can be calculated more precisely (compare to HCM eq 18-17)
		//TODO: make xTime lane-dependent for some peds
		xTime 	  = accT + (endPtDist - side) / maxVY; 
		sigR	  = 0.01*UserPanel.tStep;	//st. dev. of relative approach rate
		approachX = t.xLoc;
		xDist	  = Math.abs(xLoc - approachX);
		yDist	  = (double)ln*RoadBuilder.laneW + RoadBuilder.laneW/2;
		dist	  = space.getDistance(myLoc, t.myLoc);
		approachV = t.v;
		
		//include errors
		if (UserPanel.estErr == true) {
			etaS = rnd.nextGaussian();
			etaV = rnd.nextGaussian();
			wS = UserPanel.wien1*wS + UserPanel.wien2*etaS;
			wV = UserPanel.wien1*wV + UserPanel.wien2*etaV;
			approachV = t.v - dist*sigR*wV;
			dist  = dist*Math.exp(UserPanel.Vs*wS);
			xDist = Math.sqrt(dist*dist - yDist*yDist);}
		
		//calculate relevant times
		if (ln == 0) {
			threatBeg = 0;}
		else {
			threatBeg = accT + (ln*RoadBuilder.laneW/maxVY);}	//ped enters lane
//		threatEnd = accT + (ln+1)*RoadBuilder.laneW/maxVY;		//ped exits lane
		TTCol	= xDist/approachV;
		TTClear	= TTCol + t.length/approachV; //TODO: add radius of ped to this calculation
		
		//decide if lag is big enough to start crossing
		if (yielders.contains(t)) {
			goes = 1;}
		else {
			if (t.ying == 1) {
				goes = 1;}
			else {
				if (threatBeg + critGap < TTCol) {
					goes = 1;}
				else if (threatBeg > TTClear + 1/UserPanel.tStep && TTCol < (t.decelT-1)) { 
					//TODO: 1 is arbitrary. scale w minGap, find literature
					//TODO: decelT should be based on ped values
					if (t.follower != null) {
						goes = 0;}
					else {
						goes = 1;}}
				else {
					goes = -1;}}}
		return goes;
	}
	
	/**
	 * Calculates if gap is sufficient to go
	 * @param t1 - car that ped will cross in front of
	 * @param ln - lane car is in
	 * @return: -1=don't go; 0=look to next car; 1=go
	 */
	public int gap(Turtle t1, int ln) {		//TODO: peds still fucking up the rolling gap
		int goes = 0;						//TODO: make the perception of necessary stopping speed error-prone
		Turtle t2 = t1.follower;
		double t1x, dist1, t1d, yDist, TTCol, TTClear; //, t1v;
		double t2v, t2x, dist2, t2d;
		double threatBeg; //, threatEnd;
		double maxVY = Math.abs(maxV*Math.cos(endPtTheta));
		xTime = accT + (endPtDist - side) / maxVY; 
		sigR = 0.01*UserPanel.tStep;	//standard deviation of relative approach rate
		t1x		= t1.xLoc;
		t2x 	= t2.xLoc;
		t1d		= Math.abs(xLoc - t1x);
		t2d		= Math.abs(xLoc - t2x);
		yDist	= (double)ln*RoadBuilder.laneW + RoadBuilder.laneW/2;
		dist1	= space.getDistance(myLoc, t1.myLoc);
		dist2	= space.getDistance(myLoc, t2.myLoc);
		t2v		= t2.v;		
		
		//include errors
		if (UserPanel.estErr == true) {
			etaS	= rnd.nextGaussian();
			etaV	= rnd.nextGaussian();
			wS		= UserPanel.wien1*wS + UserPanel.wien2*etaS;
			wV		= UserPanel.wien1*wV + UserPanel.wien2*etaV;
			t2v		= t2v - dist2*sigR*wV;
			dist1	= dist1*Math.exp(UserPanel.Vs*wS);
			dist2	= dist2*Math.exp(UserPanel.Vs*wS);
			t1d		= Math.sqrt(dist1*dist1 - yDist*yDist);
			t2d		= Math.sqrt(dist2*dist2 - yDist*yDist);}
		
		//calculate relevant times
		double thisTail   = Math.abs(t2d - t1d);
		double thisTailT  = thisTail/t2v;
		if (ln == 0) {
			threatBeg = 0;}
		else {
			threatBeg = accT + (ln*RoadBuilder.laneW/maxVY);}
//		threatEnd = accT + (ln+1)*RoadBuilder.laneW/maxVY;
		TTCol	= Math.abs(t2d/t2v);
		TTClear	= TTCol + t2.length/t2v; //TODO: add radius of ped to this calculation
		
		//decide if gap is big enough to start crossing
		if (yielders.contains(t2)) {
			goes = 1;}
		else {
			if (thisTailT > critGap) {
				goes = 1;}
			else if (threatBeg > TTClear) {
				if (t2.follower != null) {	//switch to observing next car
					switch (ln) {
					case 0:	nearest0 = t2;
							break;
					case 1: nearest1 = t2;
							break;
					case 2: nearest2 = t2;
							break;
					case 3: nearest3 = t2;
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
	 * @param location
	 * @param direct
	 * @return
	 */
	public double[] accel(NdPoint location, int direct) {
		forcesX = new ArrayList<Double>();
		forcesY = new ArrayList<Double>();
		double xF, yF, curbF, boxF, dBox;
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
		for (Ped a : Scheduler.allPeds) {
			if (a != this && (a.dir == dir || a.crossing == 2)) {
				NdPoint	otherLoc = space.getLocation(a);
				double	otherY	 = otherLoc.getY();
				double	visible	 = Math.signum((double)dir*(otherY-yLoc));
				if (visible == 1) {		//peds only affected by those in front of them
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
						forcesY.add(interFy);}}}}
		
		//stop at curb if necessary
		if (curbed == true) {
			double dCurb;
			if (direct == 1) {dCurb = side - yLoc;}
			else {dCurb = yLoc - side - RoadBuilder.roadW;}
			curbF = -(double)direct*(A*Math.exp((r-dCurb)/B) + k*(r-dCurb))/m;
			forcesY.add(curbF);}

		//keep peds within crossing box
		if (boxed) {
			boxF = 0;
			whichSide = Math.signum(RoadBuilder.xWalkx - xLoc); //1=left, -1=right
			if (whichSide == 1) {dBox = xLoc - boxL;}
			else {dBox = boxR - xLoc;}
			boxF = whichSide*(A*Math.exp((r-dBox)/B) + k*(r-dBox))/m;
			forcesX.add(boxF);}
		
		//sum all forces
		for (Double b : forcesX) {
			xF += b;}
		for (Double c : forcesY) {
			yF += c;}
		acc = new double[] {xF, yF};
		return acc;
	}

	
	
	/**
	 * Move peds and update context
	 * @param loc
	 * overloaded. optional parameters:
	 * @param displacement
	 * @param destination
	 */
	public void move(NdPoint loc, double[] displacement) {
		double[] zero = new double[] {0,0};
		double yl = loc.getY();
		if (yl + displacement[1] > RoadBuilder.worldW || yl + displacement[1] < 0) {
			Scheduler.killListP.add(this);}
		else if (displacement != zero) {	
			space.moveByDisplacement(this,displacement);
//			grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
			myLoc = space.getLocation(this);}
	}
	public void move(NdPoint loc, NdPoint destination) {
		double xd = destination.getX();
		double yd = destination.getY();
		if (yd > RoadBuilder.worldW || yd < 0) {
			Scheduler.killListP.add(this);}
		else if (loc != destination) {
				space.moveTo(this,xd,yd);
//				grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
				myLoc = space.getLocation(this);}
	}
	
	/**
	 * Keep velocities within acceptable ranges
	 * @param input
	 * @return velocity below maxV
	 */
	public double[] limitV(double[] input) {
		double totalV, norm;
		if (this.dir == 1) {
			if (input[1] < 0) {
				input[1] = 0;}}
		else {
			if (input[1] > 0) {
				input[1] = 0;}}
		totalV = Math.sqrt(input[0]*input[0] + input[1]*input[1]);
		if (totalV > maxV) {
			norm = maxV/totalV;
			input[0] = input[0]*norm;
			input[1] = input[1]*norm;}
		return input;
	}
	
	
	/**
	 * Adder for 2-dimensional arrays
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
	
	
	/**
	 * Creates pedestrian agents and initializes values
	 * Called by scheduler in Agent.java
	 * @param contextSpace
	 * @param contextGrid
	 * @param direction
	 */
//	public Ped(ContinuousSpace<Object> contextSpace, Grid<Object> contextGrid, int direction) {
	public Ped(ContinuousSpace<Object> contextSpace, int direction) {
		yielders = new ArrayList<Turtle>();
		space	 = contextSpace;
		maxV	 = UserPanel.pedVavg;
		dir		 = direction; // 1 moves up, -1 moves down
		v		 = new double[] {0,(double)dir*.5*maxV};
		crossing = 0;
		curbed   = false;
		age		 = 0;
		wS = etaS = rnd.nextGaussian(); //TODO: separate these?
		wV = etaV = rnd.nextGaussian();
		double gapParamA	= 6.2064; //TODO: email Brewer re: value for just convergent sets
		double gapParamB	= 0.942;  //TODO: ditto
		double gapMin		= 1/(1+Math.exp(gapParamA));
		double preGap0		= rnd.nextDouble();
		double preGap		= preGap0*(1-gapMin) + gapMin; //avoids negative values
		double critGapS		= (gapParamA - Math.log((1/preGap) - 1))/gapParamB; //from Brewer 2006
		critGap = critGapS/UserPanel.tStep; //critical gap in simulation time units
		
		//3-circle variables - from Helbing, et al (2000) [r from Rouphail et al 1998]
		//TODO: accT is strange - should it not vary with how far from maxV the ped is?
		accT  = 0.5/UserPanel.tStep;							//acceleration time
		m     = 80;											//avg ped mass in kg
		horiz = 5/RoadBuilder.spaceScale;					//distance at which peds affect each other
		A     = 2000*UserPanel.tStep*UserPanel.tStep/RoadBuilder.spaceScale;	//ped interaction constant (kg*space units/time units^2)
		B     = 0.08/RoadBuilder.spaceScale;					//ped distance interaction constant (space units)
		k	  = 120000*UserPanel.tStep*UserPanel.tStep;			//wall force constant
		r     = 0.275/RoadBuilder.spaceScale;					//ped radius (space units)
		
		//store endpoint
		if (dir == 1) endPt = new NdPoint(RoadBuilder.xWalkx + 1/RoadBuilder.spaceScale, RoadBuilder.worldW);
		else endPt = new NdPoint(RoadBuilder.xWalkx - 1/RoadBuilder.spaceScale, 0);
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
		return vY*UserPanel.vBase;}
	@Parameter(usageName="crossing", displayName="Crossing?")
	public double getCrossing() {
		return crossing;}
	@Parameter(usageName="cg", displayName="Critical gap")
	public double getGap() {
		return critGap*UserPanel.tStep;}
	@Parameter(usageName="xTime",displayName="xTime")
	public double getXtime() {
		return xTime*UserPanel.tStep;}
	@Parameter(usageName="loc",displayName="y")
	public double getYloc() {
		return yLoc;}
}