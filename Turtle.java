package driving;

import java.util.ArrayList;
import java.util.Random;

import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;
import repast.simphony.space.grid.Grid;

/**
* Vehicle agent class. Defines creation and actions.
* Will need more methods for detailed driver vision modeling.
* @author - Darryl Michaud
*/

public class Turtle extends Agent{
	private ContinuousSpace<Object> space;
	private Grid<Object> grid;
	Random rnd = new Random(); //initiates random number generator for vehicle properties
	public double v, acc, xLoc;
	private double tGap, jamHead, maxv, mina, maxa;
	public int lane; // 0 -> bottom lane, 1 -> top lane
	//private double vision;
	
	/**
	* Used to move cars at each tick.
	*/
	@Override
	public void step() {
		double vNew;
		NdPoint myLoc = space.getLocation(this);
		this.xLoc = myLoc.getX();
		int thisLane = this.lane;
		
		this.acc = accel(myLoc, thisLane);
		vNew = this.v + this.acc;
		if (vNew < 0) {vNew = 0;}
		if (vNew > this.maxv) {vNew = this.maxv;}
		
		// Die or move?
		if (myLoc.getX() + vNew >= RoadBuilder.roadLength) {
			die();}
		else if (vNew != 0) {
			space.moveByDisplacement(this,vNew,0);
			myLoc = space.getLocation(this);
			this.xLoc = myLoc.getX();
			grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());}
		this.v = vNew;}

	/**
	 * Used to calculate car-following behavior based on location.
	 */
	public double accel(NdPoint loc, int myLane) {
		double setSpeed = this.maxv;
		double thisX = loc.getX();
		double a;
		ArrayList<Turtle> ahead = new ArrayList<Turtle>();
		ArrayList<Turtle> leaders = new ArrayList<Turtle>();
		Turtle leader = null;
		double head = RoadBuilder.roadLength;
		for (Turtle m : Scheduler.allCars) {
			if (m.xLoc > thisX) {
				ahead.add(m);}}
		if (!ahead.isEmpty()) {
			for (Turtle n : ahead) {
				if (n.lane == myLane) {
					leaders.add(n);}}
			if(!leaders.isEmpty()) {
				for (Turtle o : leaders) {
					if(o.xLoc - thisX < head) {
						head = o.xLoc - thisX;
						leader = o;}}}}
		if (leader != null) {
			setSpeed = leader.v;
			double vDiff = setSpeed - this.v;
			//double check the variables here - is setSpeed supposed to be used again?
			double safeHead = (jamHead + this.v*(this.tGap/.18) +
					((this.v*vDiff)/(2*Math.sqrt(this.maxa*this.mina))));
			a = this.maxa*(1 - Math.pow(this.v/this.maxv,4) - Math.pow(safeHead/head,2));}
		else {a = maxa*(1 - Math.pow(this.v/this.maxv,4));}
		
		return a;}

	/**
	* Used to initialize values in created cars. 
	* Called by scheduler in Agent.java
	*/
	public Turtle(ContinuousSpace<Object> space, Grid<Object> grid) {
		this.space = space;
		this.grid  = grid;
		this.maxa = rnd.nextGaussian()*(RoadBuilder.maxa*.08)+RoadBuilder.maxa;
		this.mina = rnd.nextGaussian()*(RoadBuilder.mina*.08)+RoadBuilder.mina;
		this.maxv = rnd.nextGaussian()*(.2*RoadBuilder.vLimit/RoadBuilder.vBase)+(RoadBuilder.vLimit/RoadBuilder.vBase);
		// stdDev of maxv is stretched to .2*mean to demonstrate car-following behavior
		this.tGap = rnd.nextGaussian()*(RoadBuilder.tGap*.08)+RoadBuilder.tGap;
		this.jamHead = rnd.nextGaussian()*(RoadBuilder.jamHead*.08)+RoadBuilder.jamHead;
		this.v = this.maxv * (1 - .3*rnd.nextDouble());}
	
	/**
	 * Getter for identification
	 */
	@Override
	public int isCar() {
		return 1;}
}
