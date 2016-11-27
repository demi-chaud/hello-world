package driving1;

import java.util.ArrayList;

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
	private	NdPoint myLoc;
	private boolean go;
	public	double v;
	private	double xLoc, yLoc, maxV, disp, side, frogger, gap;
	public	int age, dir;	// dir = 1 walks up, -1 walks down
	public	int crossing;	// 0=not yet, 1=waiting, 2=yes, 3=done
	public	Turtle nearest;
	//TODO: peds currently don't interact with each other. fix that
	//double bubble = 0.5 / RoadBuilder.spaceScale; //gives .5m gap between peds at rest 
	
	//TODO: inroad() script/flag
	public void calc() {
		myLoc = space.getLocation(this);
		xLoc = myLoc.getX();
		yLoc = myLoc.getY();
		disp = dir*v;
		side = RoadBuilder.sidewalk;
		
		switch (crossing) {
		case 0: if (dir == 1) {
					if (yLoc + disp >= side) {
						space.moveTo(this,xLoc,side);
						myLoc = space.getLocation(this);
//						grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
						v = 0;
						crossing = 1;}
					else move(myLoc,disp);}
				else {
					if (yLoc + disp <= side + RoadBuilder.roadW) {
						space.moveTo(this,xLoc, side+RoadBuilder.roadW);
						myLoc = space.getLocation(this);
//						grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
						crossing = 1;}
					else move(myLoc,disp);}
				break;
		case 1: disp = yield();
				v    = Math.abs(disp);
				if (disp != 0) move(myLoc,disp);
				break;
		case 2:	move(myLoc,disp);
				yLoc = myLoc.getY();
				if (dir == 1) {
					if (yLoc >= side + RoadBuilder.roadW) crossing = 3;}
				else if (yLoc <= side) {crossing = 3;}
				break;
		case 3: move(myLoc,disp);
				break;
		default: break;}
	}
	
	/**
	 * Peds call this to determine if there is room to cross
	 * TODO: Currently only works for direction = 1
	 * TODO: will need fundamental rewriting to take double threat into account
	 * TODO: need to add cars coming from other direction
	 * TODO: still needs calc of car position in time
	 */
	
	public double yield() {
		ArrayList<Turtle> approaching = new ArrayList<Turtle>();
//		ArrayList<Turtle> approaching0 = new ArrayList<Turtle>();
//		ArrayList<Turtle> approaching1 = new ArrayList<Turtle>();
//		ArrayList<Turtle> near = new ArrayList<Turtle>();
		nearest = null;
		gap = RoadBuilder.roadL/2;
		for (Turtle m : Scheduler.allCars) {
			if (m.xLoc < RoadBuilder.xWalkx) {
				approaching.add(m);}}
		if (!approaching.isEmpty()) {
//			for (Turtle n : approaching) {
//				if (n.lane == 0) {
//					approaching0.add(n);}
//				else if (n.lane == 1) {
//					approaching1.add(n);}}
//			if(!approaching0.isEmpty()) {
				for (Turtle o : approaching) {
					if(RoadBuilder.xWalkx - o.xLoc < gap) {
						gap = RoadBuilder.xWalkx - o.xLoc;
						nearest = o;}}}
		if (nearest != null) {
			go = gap(nearest);
			if (go==true) {
				frogger = dir * maxV;
				crossing = 2;}
			//TODO: need to add acceleration time
			else frogger = 0;}
		else {
			frogger = dir * maxV;
			crossing = 2;}
		return frogger;
	}
	
	//TODO: include estimation errors
	public boolean gap(Turtle t) {
		go = false;
//		int approachL;
		double approachV, approachX, dist, TTC;
		//TODO: xTime will need to be adapted when acceleration is added
		double xTime = RoadBuilder.roadW / maxV; 
		
		approachX = t.xLoc;
		approachV = t.v;
//		approachL = nearest.lane;
		
		dist = Math.abs(xLoc - approachX);
		TTC  = dist/approachV;
		if (xTime < TTC) go = true;
		
		return go;
	}
	
	/**
	 * Moves peds and updates context
	 * @param loc
	 * @param displacement
	 */
	
	public void move(NdPoint loc, double displacement) {
		double yl = loc.getY();
		if (yl + displacement > RoadBuilder.worldW || yl + displacement < 0) {
			die();}
		else if (displacement != 0) {
				space.moveByDisplacement(this,0,displacement);
				myLoc = space.getLocation(this);
//				grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
		}
	}
	
	//TODO: give x-walk width, distribute peds, add interaction
	
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
		//v     = RoadBuilder.pedVavg * 1000 / 3600;
		maxV  = UserPanel.pedVavg;
		v     = maxV;
		dir   = direction; // 1 moves up, -1 moves down
		crossing = 0;
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
	public double getVel() {
		System.out.println(nearest);
		return v;}
	@Parameter(usageName="crossing", displayName="Crossing?")
	public double getCrossing() {
		return crossing;}
}


//
///**
// * Attempt at using car-follwing to match ped speeds - probably not good approach
// */
//public double crowd(NdPoint loc, int myDir) {
//	double setSpeed = this.maxV;
//	double thisY = loc.getY();
//	ArrayList<Ped> ahead = new ArrayList<Ped>();
//	ArrayList<Ped> leaders = new ArrayList<Ped>();
//	Ped leader = null;
//	double head = RoadBuilder.worldW;
//	if (this.dir == 1) {
//		for (Ped m : Scheduler.allPeds) {
//			if (m.yLoc > thisY) {
//				ahead.add(m);}}}
//	else {for (Ped m : Scheduler.allPeds) {
//			if (m.yLoc < thisY) {
//				ahead.add(m);}}}
//	if (!ahead.isEmpty()) {
//		for (Ped n : ahead) {
//			if (n.dir == myDir) {
//				leaders.add(n);}}
//		if(!leaders.isEmpty()) {
//			for (Ped o : leaders) {
//				if(Math.abs(o.yLoc - thisY) < head) {
//					head = Math.abs(o.yLoc - thisY);
//					leader = o;}}}}
//	
//	//rest of code is not adjusted from car following
//	if (leader != null) {
//		setSpeed = leader.v;
//		double vDiff = setSpeed - this.v;
//		//double check the variables here - is setSpeed supposed to be used again?
//		double safeHead = (jamHead + this.v*(this.tGap/.18) +
//				((this.v*vDiff)/(2*Math.sqrt(this.maxa*this.mina))));
//		a = this.maxa*(1 - Math.pow(this.v/this.maxv,4) - Math.pow(safeHead/head,2));}
//	else {a = maxa*(1 - Math.pow(this.v/this.maxv,4));}
//	return v;
//}
