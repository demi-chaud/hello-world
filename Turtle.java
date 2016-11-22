package driving1;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;

/**
* Vehicle agent class. Defines creation and actions.
* Will need more methods for detailed driver vision modeling.
* @author - Darryl Michaud
 * @param <E>
*/

public class Turtle extends Agent{
	ContinuousSpace<Object> space;
//	Grid<Object> grid;
	Random rnd = new Random(); 			//initiates random number generator for vehicle properties
	public double v, vNew, acc, xLoc;
	private double tGap, jamHead, maxv, mina, maxa, newAcc, wS, etaS, wV, etaV, sigR;
	private double delayT = UserPanel.delayT;
	public int age, lane; 				// lane = 0 -> bottom lane, 1 -> top lane
	//TODO: add int "dir" tag and implement cars going other direction 
	private List<double[]> storage = new ArrayList<double[]>();
	private boolean distracted = false; 		// 1 = yes, figure out how to implement this
	private NdPoint myLoc;
//	double vision;
	
	/**
	* Used to move cars at each tick.
	*/
	@Override
	public void calc() {
		myLoc = space.getLocation(this);
		this.xLoc = myLoc.getX();
		int thisLane = lane;
		
		if (distracted == false) newAcc = accel(myLoc, thisLane);
		else newAcc = acc;
		
		//implement acc calculated delayT ago
		if (UserPanel.delayT > 0) {
			double stamp = RoadBuilder.clock.getTickCount();
			double tStamp = stamp*UserPanel.tStep;
			storage.add(new double[] {tStamp, newAcc, v});
			double delayedT = tStamp - delayT;
			double hiT = 0;
			int foo = 0;
			if (age > 2) {
				while (hiT < delayedT) {
					hiT = storage.get(foo)[0];
					foo++;}
				double hiAcc = storage.get(foo-1)[1];
				if (hiT != delayedT){								//linear interpolation
					double loAcc = storage.get(foo-2)[1];
					double tN = Math.floor(delayT/UserPanel.tStep);
					double tBeta = (delayT/UserPanel.tStep) - tN;
					acc = tBeta*loAcc + (1-tBeta)*hiAcc;}
				else acc = hiAcc;}
			else acc = newAcc;
			age ++;}
		else acc = newAcc;
		
		vNew = v + acc;
		if (vNew < 0) {vNew = 0;}
		if (vNew > maxv) {vNew = maxv;}
	}		

	@Override
	public void drive() {
		if (xLoc + vNew >= RoadBuilder.roadL) {
			die();}
		else if (vNew != 0) {
			space.moveByDisplacement(this,vNew,0);
			myLoc = space.getLocation(this);
			xLoc = myLoc.getX();
//			grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
		}
		v = vNew;
	}
	
	/**
	 * Used to calculate car-following behavior based on location.
	 */
	public double accel(NdPoint loc, int myLane) {
		double setSpeed = maxv;
		double thisX = loc.getX();
		double a;
		ArrayList<Turtle> ahead = new ArrayList<Turtle>();
		ArrayList<Turtle> leaders = new ArrayList<Turtle>();
		Turtle leader = null;
		double head = RoadBuilder.roadL;
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
						head = o.xLoc - thisX; //TODO: adjust this to include car length
						leader = o;}}}}
		if (leader != null) {
			setSpeed = leader.v;
			double vDiff = setSpeed - v;
			//TODO: double-check the variables here - is setSpeed supposed to be used again?
			if (UserPanel.estErr == true) {   // includes estimation error in headway measuring
				etaS = rnd.nextGaussian();
				etaV = rnd.nextGaussian();
				wS = RoadBuilder.wien1*wS + RoadBuilder.wien2*etaS;
				wV = RoadBuilder.wien1*wV + RoadBuilder.wien2*etaV;
				head = head*Math.exp(RoadBuilder.Vs*wS);
				setSpeed = leader.v - head*sigR*wV;}
			double safeHead = (jamHead + v*(tGap/UserPanel.tStep) +
					((v*vDiff)/(2*Math.sqrt(maxa*mina))));
			a = maxa*(1 - Math.pow(v/maxv,4) - Math.pow(safeHead/head,2));}
		else {a = maxa*(1 - Math.pow(v/maxv,4));}
		
		return a;}

	/**
	 * Creates vehicle agents and initializes values
	 * Called by scheduler in Agent.java
	 * @param contextSpace
	 * @param contextGrid
	 */
	//TODO: add similar code to Ped.java to vary ped parameters
//	public Turtle(ContinuousSpace<Object> contextSpace, Grid<Object> contextGrid) {
	public Turtle(ContinuousSpace<Object> contextSpace) {	
		space = contextSpace;
//		grid  = contextGrid;
		maxa = rnd.nextGaussian()*(RoadBuilder.maxa*.08)+RoadBuilder.maxa;
		mina = rnd.nextGaussian()*(RoadBuilder.mina*.08)+RoadBuilder.mina;
		maxv = rnd.nextGaussian()*(.2*UserPanel.sLimit/RoadBuilder.vBase)+(UserPanel.sLimit/RoadBuilder.vBase);
		// TODO: stdDev of maxv is stretched to .2*mean to demonstrate car-following behavior
		tGap = rnd.nextGaussian()*(RoadBuilder.tGap*.08)+RoadBuilder.tGap;
		jamHead = rnd.nextGaussian()*(RoadBuilder.jamHead*.08)+RoadBuilder.jamHead;
		v = maxv * (1 - .3*rnd.nextDouble());
		age = 0;
		wS = etaS = rnd.nextGaussian();
		wV = etaV = rnd.nextGaussian();
		sigR = 0.01;} //standard deviation of relative approach rate TODO: should this vary?
	
	/**
	 * Getter for identification
	 */
	@Override
	public int isCar() {
		return 1;}
}


	/**
	 * Code for psycho-spatial car following behavior
	 * would be called in step() if IDM script is moved to separate method
	 * AX = measured from rear bumper to 
	 * AX
	 */
//	public Psycho() {
//		/**
//		 * all headways measured from front of lead veh to front of following veh (incl. length of lead veh)
//		 */
//		double AX; //jam headway (minimum gap when stationary)
//		double BX; //constant related to speed-determined headway
//		double EX; //between 0.5-1.5, gives distance beyond which leader does not affect behavior 
//		double ABX = AX + BX*Math.sqrt(this.v);    //desired minimum headway threshold
//		double SdelX = AX + BX*Math.sqrt(this.v)*EX; //maximum following distance threshold
//	
//		double CX; //constant related to threshold of perception for visual angle divergence
//		double delX; //headway (needs name change)
//		double CLdelV = -Math.pow(delX,2)/Math.pow(CX,2);}