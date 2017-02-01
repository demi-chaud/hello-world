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
	private ArrayList<Turtle> sameDir, ahead, leaders, behind, followers;
	private ArrayList<Ped> crossingP, waitingP;
	public  Turtle leader, follower;
	private List<double[]> storage = new ArrayList<double[]>();
	private Random  rnd = new Random(); 			//initiates random number generator for vehicle properties
	public  double	v, vNew, acc, xLoc, length;
	private double	tGap, jamHead, maxv, mina, maxa, newAcc, tail, wS, etaS, wV, etaV, sigR;
	private double	pedDecel, stopBar;
	private double	delayTs = UserPanel.delayTs;
	private double	stamp, tStamp, delayedT, hiT;
	public  int		age, ying, yType, lane;	// lane = 0 -> outer lane,   1 -> inner lane
	public	int		dir;					// dir  = 1 -> going right, -1 -> going left
	private int		thisLane, thisDir;
	private int		pedYield;				// -1 = none, 0 = soft, 1 = hard
	private boolean	distracted = false; 	// 1 = yes, figure out how to implement this
	public  NdPoint	myLoc;
	private double	tN = Math.floor(delayTs/UserPanel.tStep);
	private double	tBeta 	= (delayTs/UserPanel.tStep) - tN;	
//	double vision;
	
	/**
	* High-level method of driver action.
	*/
	public void calc() {
		myLoc = space.getLocation(this);
		xLoc = myLoc.getX();
		thisLane = lane;
		thisDir  = dir;
//		ying = 0;
		
		if (distracted == false) newAcc = accel(myLoc, thisLane, thisDir);
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
				if (hiT != delayedT){	//linear interpolation TODO: is there a better approx?
					double loAcc = storage.get(foo-2)[1];
					acc = tBeta*loAcc + (1-tBeta)*hiAcc;}
				else acc = hiAcc;}
			else acc = newAcc;
			age ++;}
		else acc = newAcc;
		
		vNew = v + acc;
//		vNew = v + (double)dir*acc;
		if (vNew < 0) {vNew = 0;}
		if (vNew > maxv) {vNew = maxv;}
	}		

	public void drive() {
		if (dir == 1) {
			if (xLoc + vNew >= RoadBuilder.roadL - 1) {
				Scheduler.killListC.add(this);}
			else if (vNew != 0) {
				space.moveByDisplacement(this,vNew,0);
				myLoc = space.getLocation(this);
				xLoc = myLoc.getX();}}
		else {
			if (xLoc - vNew <= 0) {
				Scheduler.killListC.add(this);}
			else if (vNew != 0) {
				space.moveByDisplacement(this,-vNew,0);
				myLoc = space.getLocation(this);
				xLoc = myLoc.getX();}}
		v = vNew;
	}
//	grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
	
	/**
	 * Used to calculate car-following behavior based on location.
	 */
	public double accel(NdPoint loc, int myLane, int myDir) {
		double thisX = loc.getX();
		double a, head, setSpeed, vDiff, safeHead, newYield;
		sameDir  = new ArrayList<Turtle>();
		ahead	 = new ArrayList<Turtle>();
		leaders  = new ArrayList<Turtle>();
		head	 = RoadBuilder.roadL;
		//TODO: this if loop will go away if lane-changes are included
		if (follower == null) {
			behind	  = new ArrayList<Turtle>();
			followers = new ArrayList<Turtle>();
			leader	  = null;
			tail	  = RoadBuilder.roadL;}
		for (Turtle p : Scheduler.allCars) {
			if (p.dir == myDir) {
				sameDir.add(p);}}
		if (!sameDir.isEmpty()) {
			for (Turtle m : sameDir) {
				if (myDir == 1) {					if (m.xLoc > thisX) {						ahead.add(m);}
					if (follower == null) {
						if (m.xLoc < thisX) {
							behind.add(m);}}}
				else {
					if (m.xLoc < thisX) {
						ahead.add(m);}
					if (follower == null) {
						if (m.xLoc > thisX) {
							behind.add(m);}}}}}
		if (!ahead.isEmpty()) {
			for (Turtle n : ahead) {
				if (n.lane == myLane) {
					leaders.add(n);}}}
		if (follower == null) {
			if (!behind.isEmpty()) {
				for (Turtle n : behind) {
					if (n.lane == myLane) {
						followers.add(n);}}}}
		if (!leaders.isEmpty()) {
			for (Turtle o : leaders) {
				if(Math.abs(o.xLoc - thisX) < head) {
					head = Math.abs(o.xLoc - thisX); //TODO: adjust this to include car length?
					leader = o;}}}
		if (follower == null) {
			if (!followers.isEmpty()) {
				for (Turtle p : followers) {
					if(Math.abs(p.xLoc - thisX) < tail) {
						tail = Math.abs(p.xLoc - thisX); //TODO: adjust this to include car length?
						follower = p;}}}}
		else {
			tail = Math.abs(follower.xLoc - thisX);}
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
		
		double xwalkD = stopBar - thisX;
		double threat = Math.signum(dir*xwalkD);
		if (threat == 1) {
			yield(xwalkD);
			if (ying == 1) {
				double a0 = -v*v/(2*Math.abs(xwalkD));
//				if (a0 < -mina) {
//					a = -mina;}
				if (a0 < a) {
					a = a0;}}
			else {
				
			}
		}
		return a;
	}

	/**
	 * Determines driver yielding behavior
	 * called within accel()
	 * @return yType: -1 = none, 0 = soft, 1 = hard
	 */
	public void yield(double dist) {
		//TODO: add yielding to crossing=1 peds
		//TODO: add errors in distance and reading of ped v
		//TODO: rewrite error-making code as method in Agent class
		double	decelT		= v/mina;
		double	TTCol		= Math.abs(dist/v);
		double	TTClear		= TTCol + length/v; //TODO: add width of xwalk to this calculation
		pedDecel  = 0;
		waitingP  = new ArrayList<Ped>();
		crossingP = new ArrayList<Ped>();
		
		for (Ped i : Scheduler.allPeds) {
			if (i.crossing == 1) {
				waitingP.add(i);}
			if (i.crossing == 2) {
				crossingP.add(i);}}
		
		//TODO: make this more realistic, include soft yield
		if (crossingP.isEmpty()) {
			ying = -1;}
		else {
			if (ying == -1) {
				for (Ped j : crossingP) {
//					if (!j.yielders.contains(this)) j.yielders.add(this);
//					else yType = 1;
					if (TTCol < j.xTime) {
						ying = 1;}}}}
	}
	
	/**
	 * Creates vehicle agents and initializes values
	 * Called by scheduler in Agent.java
	 * @param contextSpace
	 * @param contextGrid
	 */
	//TODO: add similar code to Ped.java to vary ped parameters
//	public Turtle(ContinuousSpace<Object> contextSpace, Grid<Object> contextGrid) {
	public Turtle(ContinuousSpace<Object> contextSpace, int whichLane, int whichDir) {	
		space	= contextSpace;
//		grid	= contextGrid;
		lane	= whichLane;
		dir		= whichDir;
		length	= UserPanel.carLength;
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
		stopBar	= RoadBuilder.xWalkx - (double)dir*5/RoadBuilder.spaceScale; 
		// 5 here is exaggerated to view effects
		// should match pedbox (?) which is arbitrarily set to 2
		ying = -1;
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
//		System.out.println(v);
		return v;}
	@Parameter(usageName="lane", displayName="Current lane")
	public double getLane() {
		return lane;}
	@Parameter(usageName="leader", displayName="Current leader")
	public Turtle getLead() {
		return leader;}
	@Parameter(usageName="yielding", displayName="yielding?")
	public int getYield() {
		return yType;}
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