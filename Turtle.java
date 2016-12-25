package driving1;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import repast.simphony.parameter.Parameter;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;

/**
* Vehicle agent class. Defines creation and actions.
* Will need more methods for detailed driver vision modeling.
* @author - Darryl Michaud
*/

public class Turtle extends Agent{
	ContinuousSpace<Object> space;
//	Grid<Object> grid;
	private ArrayList<Turtle> ahead, leaders;
	public  Turtle leader;
	private List<double[]> storage = new ArrayList<double[]>();
	private Random  rnd = new Random(); 			//initiates random number generator for vehicle properties
	public  double	v, vNew, acc, xLoc;
	private double	tGap, jamHead, maxv, mina, maxa, newAcc, wS, etaS, wV, etaV, sigR;
	private double	delayTs = UserPanel.delayTs;
	private double	stamp, tStamp, delayedT, hiT;
	public  int		age, lane; 				// lane = 0 -> bottom lane, 1 -> top lane
	private int		thisLane;
	//TODO: add int "dir" tag and implement cars going other direction 
	private boolean	distracted = false; 		// 1 = yes, figure out how to implement this
	private NdPoint	myLoc;
	private double	tN = Math.floor(delayTs/UserPanel.tStep);
	private double	tBeta = (delayTs/UserPanel.tStep) - tN;
//	double vision;
	
	/**
	* High-level method of driver action.
	*/
	public void calc() {
		myLoc = space.getLocation(this);
		xLoc = myLoc.getX();
		thisLane = lane;
		
		if (distracted == false) newAcc = accel(myLoc, thisLane);
		else newAcc = acc;
		
		//delayed reaction implements acc calculated and stored delayT ago
		if (UserPanel.delayTs > 0) {
			stamp  = RoadBuilder.clock.getTickCount();
			tStamp = stamp*UserPanel.tStep;
			//TODO: limit size of storage to limit memory use
			storage.add(new double[] {tStamp, newAcc, v});
			delayedT = tStamp - delayTs;
			hiT = 0;
			int foo = 0;
			if (storage.size() > 3 && storage.size() > tN+1) { 
				while (hiT < delayedT) {
					hiT = storage.get(foo)[0];
					foo++;}
				double hiAcc = storage.get(foo-1)[1];
				if (hiT != delayedT){	//linear interpolation (is there a better approx?)
					double loAcc = storage.get(foo-2)[1];
					acc = tBeta*loAcc + (1-tBeta)*hiAcc;}
				else acc = hiAcc;}
			else acc = newAcc;
			age ++;}
		else acc = newAcc;
		
		vNew = v + acc;
		if (vNew < 0) {vNew = 0;}
		if (vNew > maxv) {vNew = maxv;}
	}		

	public void drive() {
		if (xLoc + vNew >= RoadBuilder.roadL) {
			Scheduler.killListC.add(this);}
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
		double thisX = loc.getX();
		double a, head, setSpeed, vDiff, safeHead;
		ahead	= new ArrayList<Turtle>();
		leaders	= new ArrayList<Turtle>();
		leader	= null;
		head	= RoadBuilder.roadL;
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
			if (UserPanel.estErr == true) {   // includes estimation error in headway measuring
				etaS = rnd.nextGaussian();
				etaV = rnd.nextGaussian();
				wS = UserPanel.wien1*wS + UserPanel.wien2*etaS;
				wV = UserPanel.wien1*wV + UserPanel.wien2*etaV;
				head = head*Math.exp(UserPanel.Vs*wS);
				setSpeed = leader.v - head*sigR*wV;}
			else setSpeed = leader.v;
			vDiff = setSpeed - v;
			safeHead = (jamHead + v*(tGap) +
					((v*vDiff)/(2*Math.sqrt(maxa*mina))));
			a = maxa*(1 - Math.pow(v/maxv,4) - Math.pow(safeHead/head,2));}
		else {a = maxa*(1 - Math.pow(v/maxv,4));}
		
		return a;
	}

	/**
	 * Creates vehicle agents and initializes values
	 * Called by scheduler in Agent.java
	 * @param contextSpace
	 * @param contextGrid
	 */
	//TODO: add similar code to Ped.java to vary ped parameters
//	public Turtle(ContinuousSpace<Object> contextSpace, Grid<Object> contextGrid) {
	public Turtle(ContinuousSpace<Object> contextSpace, int whichLane) {	
		space = contextSpace;
//		grid  = contextGrid;
		lane = whichLane;
		//store parameters with heterogeneity (currently s.dev = 8% of mean)
		//TODO: get theory for these numbers
		maxa = rnd.nextGaussian()*(UserPanel.maxa*.08)+UserPanel.maxa;
		mina = rnd.nextGaussian()*(UserPanel.mina*.08)+UserPanel.mina;
		maxv = rnd.nextGaussian()*(.2*UserPanel.sLimit)+(UserPanel.sLimit);
		// TODO: stdDev of maxv is stretched to .2*mean to demonstrate car-following behavior
		tGap = rnd.nextGaussian()*(UserPanel.tGap*.08)+UserPanel.tGap;
		jamHead = rnd.nextGaussian()*(UserPanel.jamHead*.08)+UserPanel.jamHead;
		v = maxv * (1 - .3*rnd.nextDouble());
		age = 0;
		wS = etaS = rnd.nextGaussian();
		wV = etaV = rnd.nextGaussian();
		sigR = 0.01; //standard deviation of relative approach rate TODO: should this vary?
	}
	
	/**
	 * Getter for identification
	 */
	@Override
	public int isCar() {
		return 1;}
	
	/**
	 * Parameter declarations for probe
	 */
	@Parameter(usageName="newAcc", displayName="Calculated acc")
	public double getAcc() {
		return newAcc;}
	@Parameter(usageName="v", displayName="Current vel")
	public double getVel() {
		System.out.println(leader);
		return v;}
	@Parameter(usageName="lane", displayName="Current lane")
	public double getLane() {
		return lane;}
	@Parameter(usageName="leader", displayName="Current leader")
	public Turtle getLead() {
		return leader;}
}


//	/**
//	 * Code for psycho-spatial car following behavior
//	 * would be called in step() if IDM script is moved to separate method
//	 * AX = measured from rear bumper to 
//	 * AX
//	 */
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