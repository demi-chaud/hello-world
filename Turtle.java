package driving1;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.apache.commons.math3.util.FastMath;

import repast.simphony.parameter.Parameter;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;

/**
* Vehicle agent class. Defines creation and actions.
* @author - Darryl Michaud
*/
public class Turtle extends Agent{
	ContinuousSpace<Object> space;
//	Grid<Object> grid;
	private ArrayList<Turtle> sameDir, ahead, leaders, behind, followers;
	private ArrayList<Ped> crossingP, waitingP;
	private ArrayList<ViewAngle> obstructers, obstructees;
	private ArrayList<Yieldage> yieldage = new ArrayList<Yieldage>();
	private List<double[]> storage = new ArrayList<double[]>();
	private Random  rnd = new Random();	//initiates random number generator for vehicle properties
	private boolean	distracted = false; // TODO:figure out how to implement this
	private double	delayTs, tN, tBeta;							//delay
	private double	tGap, jamHead, maxv, mina, maxa, newAcc;	//car-following
	private double	wS, etaS, wV, etaV, sigR;					//errors
	private double	stopBar, TTCol, lnTop, lnBot;				//yielding
	private double	carW = UserPanel.carWidth;
	private int		age;
	public  NdPoint	myLoc;
	public  Turtle	leader, follower;
	public  double	v, vNew, acc, xLoc, yLoc, length, tail, driverX, driverY;
	public  int		lane;	//  0 = outer lane, 1 = inner lane
	public	int		dir;	//  1 = going right, -1 = going left
	public	int		ying;	// -1 = none, 0 = soft, 1 = hard
//	double vision;
	//TODO: combine xLoc and thisX
	
	/**
	* Calculates driver acceleration (if not distracted)
	* Sets value of acc & vNew
	*/
	public void calc() {
		myLoc	= space.getLocation(this);
		xLoc	= myLoc.getX();
		
		if (distracted == false) newAcc = accel(myLoc, lane, dir);
		else newAcc = acc;
		
		//delayed reaction: implements acc calculated and stored delayT ago
		if (UserPanel.delayTs > 0) {
			double stamp, tStamp, delayedT, hiT;
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
			else acc = newAcc;}
		else acc = newAcc;
		age++;
		
		vNew = v + acc;
		if (vNew < 0) {vNew = 0;}
		if (vNew > maxv) {vNew = maxv;}
	}		
	
	/**
	 * Moves the cars based on value created in calc()
	 */
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
	 * Calculates acceleration based on car-following and yielding.
	 * Compares the two and returns the lower value.
	 * @param loc
	 * @param myLane
	 * @param myDir
	 * @return accel
	 */
	public double accel(NdPoint loc, int myLane, int myDir) {
		double thisX = loc.getX();
		double a, head, setSpeed, vDiff, safeHead;
		sameDir  = new ArrayList<Turtle>();
		ahead	 = new ArrayList<Turtle>();
		leaders  = new ArrayList<Turtle>();
		head	 = RoadBuilder.roadL;
		
		//Determine leader and follower
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
		
		//calculate CF acceleration (with errors)
		if (leader != null) {
			if (UserPanel.estErr == true) {   //includes estimation error in headway measurement
				etaS = rnd.nextGaussian();
				etaV = rnd.nextGaussian();
				wS = UserPanel.wien1*wS + UserPanel.wien2*etaS;
				wV = UserPanel.wien1*wV + UserPanel.wien2*etaV;
				head = head*Math.exp(UserPanel.Vs*wS);
				setSpeed = leader.v - head*sigR*wV;}
			else setSpeed = leader.v;
			vDiff = v - setSpeed;
			safeHead = (jamHead + v*(tGap) +
					((v*vDiff)/(2*Math.sqrt(maxa*mina))));
			a = maxa*(1 - Math.pow(v/maxv,4) - Math.pow(safeHead/head,2));}
		else {a = maxa*(1 - Math.pow(v/maxv,4));}
		
		//Calculate yielding acceleration
		
		double stopD	= stopBar - thisX;
		double stopDab	= Math.abs(stopD);
		double xwalkD	= RoadBuilder.xWalkx - thisX;
		double threat	= Math.signum(dir*xwalkD);
		if (threat == 1) {
			double a0 = yield(stopDab);
//			if (a0 < -mina) {
//				a = -mina;}
			if (a0 < a) {
				a = a0;}}
		return a;
	}

	/**
	 * Determines driver yielding behavior, returns deceleration if necessary
	 * called within accel()
	 */
	public double yield(double dist) {
		//TODO: add yielding to crossing=1 peds
		//TODO: add errors in distance and reading of ped v
		//TODO: rewrite error-making code as method in Agent class
//		double	TTClear	= TTCol + length/v; //TODO: add width of xwalk to this calculation
		waitingP  = new ArrayList<Ped>();
		crossingP = new ArrayList<Ped>();
		obstructers = new ArrayList<ViewAngle>();
		obstructees = new ArrayList<ViewAngle>();
		double	threatBeg, threatEnd;
		double	lnW		 = RoadBuilder.laneW;
		double	decelT	 = v/mina;
		double	yieldDec = 1;		//dummy value
		double	hardYield	= -v*v/(2*dist);
		double	tHardYield	= 2*dist/v;
		TTCol = dist/v;
		driverX = xLoc-(double)dir*length/2;
		threatBeg = 0;
		threatEnd = -1;
		ying = -1;
		
		//make list of waiting/crossing peds
		for (Ped i : Scheduler.allPeds) {
			if (i.crossing == 1) {
				waitingP.add(i);}
			if (i.crossing == 2) {
				crossingP.add(i);}}
		
		//double threat
		for (Turtle i : ahead) {				//are there any cars potentially blocking view?
			double otherD	= RoadBuilder.xWalkx - i.xLoc;
			double threat	= Math.signum(dir*otherD);
			ViewAngle thisView = null;
			if (i.lane != lane && threat == 1) {		//TODO: make this only happen if the car is close enough for it to matter
				double front	= i.xLoc - (double)dir*i.length/3;
				double back		= i.xLoc - (double)dir*i.length;
				double inside	= i.yLoc + (double)dir*carW/2;
				double outside	= i.yLoc - (double)dir*carW/2;
				Double thisTheta1, thisTheta2;
				if (lane == 0) {
					thisTheta1 = FastMath.atan2((outside - driverY), (front - driverX));
					thisTheta2 = FastMath.atan2((inside  - driverY), (back  - driverX));}
				else {
					thisTheta1 = FastMath.atan2((outside - driverY), (back  - driverX));
					thisTheta2 = FastMath.atan2((inside  - driverY), (front - driverX));}
				if (thisTheta1 != null) {
					if (thisTheta1 < 0) {
						thisTheta1 = thisTheta1 + 2*Math.PI;}
					if (thisTheta2 < 0) {
						thisTheta2 = thisTheta2 + 2*Math.PI;}
					thisView = new ViewAngle(i,thisTheta1,thisTheta2);
					obstructers.add(thisView);}}}
		if (!obstructers.isEmpty()){
			for (Ped j : crossingP) {				//calculate angle to any relevant peds
				Double thisTheta = null;
				if (dir == j.dir && lane == 1) {
					thisTheta = FastMath.atan2((j.yLoc - driverY), (j.xLoc - driverX));}
				else if (dir != j.dir && lane == 0) {
					thisTheta = FastMath.atan2((j.yLoc - driverY), (j.xLoc - driverX));}
				if (thisTheta != null) {
					if (thisTheta < 0) {
						thisTheta = thisTheta + 2*Math.PI;}
					ViewAngle thisView = new ViewAngle(j,thisTheta);
					obstructees.add(thisView);}}}
		if (!obstructees.isEmpty()) {
			for (ViewAngle i : obstructers) {
				double theta1 = i.theta1;
				double theta2 = i.theta2;
				for (ViewAngle j : obstructees) {
					Ped thisPed = j.ped;
					double pedTheta = j.theta;
					if (pedTheta >= theta1 && pedTheta <= theta2) {
						crossingP.remove(thisPed);}}}}
				
		//TODO: add yielding to waiting peds
		if (!crossingP.isEmpty()) {
			for (Ped k : crossingP) {
				double pedY = k.yLoc;
				double thisDecel = 1;
				double endGauntlet = 0;
				double oldDecel, clearY;
				Yieldage oldVals = null;
				oldDecel = clearY = 0;
				int thisYing = -1;
				
				//calculate relevant times for this ped (note: in OR, cars have to wait until ped is >1 lane away)
				for (Yieldage m : yieldage) {
					if (m.yieldee == k) {			//check if already yielding to ped
						oldDecel = m.calcAcc;
						clearY   = m.endThreat;
//						oldYield = m.yState;
						oldVals  = m;
						break;}}
				
				//bring in old accel value if above is true
				if (oldVals != null) {
					double newDecel;
					if (k.dir == 1 && clearY > pedY) {
						if (v != 0) {
							threatEnd = k.accT*(1-k.v[1]/k.maxV) + (clearY - pedY)/k.maxV;
							if (threatEnd > tHardYield) {
								newDecel = hardYield;
								thisYing = 1;}
							else {
								newDecel = -2*(v*threatEnd - dist) / (threatEnd*threatEnd);
								thisYing = 0;}
							oldVals.yState = thisYing;
							if (newDecel < oldDecel) {
								thisDecel = newDecel;
								oldVals.calcAcc = newDecel;}
							else {
								thisDecel = oldDecel;}}
						else {
							thisDecel = 0;
							thisYing  = 1;}}
					else if (k.dir == -1 && clearY < pedY) {
						if (v != 0) {
							threatEnd = k.accT*(1-Math.abs(k.v[1]/k.maxV)) + (pedY - clearY)/k.maxV;
							if (threatEnd > tHardYield) {
								newDecel = hardYield;
								thisYing = 1;}
							else {
								newDecel = -2*(v*threatEnd - dist) / (threatEnd*threatEnd);
								thisYing = 0;}
							oldVals.yState = thisYing;
							if (newDecel < oldDecel) {
								thisDecel = newDecel;
								oldVals.calcAcc = newDecel;}
							else {
								thisDecel = oldDecel;}}
						else {
							thisDecel = 0;
							thisYing  = 1;}}
					else {
						yieldage.remove(oldVals);
						thisYing = -1;}}
				
				//calculate necessary accel and add ped to list of yieldees
				else {
					if (k.dir == 1) {					//ped walking up
						endGauntlet = lnTop + lnW;
						if (pedY > lnBot) {
							if (pedY > lnTop) {
								if (pedY > lnTop + lnW) {	//ped already out of danger (lane buffer irrelevant in top lane
									threatEnd = 0;}										//bc crossing = 3 once ped is clear)
								else {						//ped out of this lane, not yet clear
									threatBeg = 0;
									threatEnd = (lnTop + lnW - pedY)/k.maxV;}}
							else {							//ped in this lane
								threatBeg = 0;
								threatEnd = k.accT + (lnTop + lnW - pedY)/k.maxV;}}
						else {
							if (lane == 0) {
								if (dir == 1) {				//car in bottom lane, ped hasn't started to cross
									threatBeg = 0;
									threatEnd = k.accT + 2*lnW/k.maxV;}
								else {						//car in top lane, ped clear after just this lane
									threatBeg	= (lnBot - lnW - pedY)/k.maxV;  //one lane buffer
									threatEnd	= (lnTop - pedY)/k.maxV;
									endGauntlet	= lnTop;}}
							else {							//ped in or below same lane
								threatBeg = (lnBot - lnW - pedY)/k.maxV;
								threatEnd = k.accT + (lnTop + lnW - pedY)/k.maxV;}}}
					else {								//ped walking down
						endGauntlet = lnBot - lnW;
						if (pedY < lnTop) {
							if (pedY < lnBot) {
								if (pedY < lnBot - lnW) {	//ped already out of danger (lane buffer irrelevant in bottom lane
									threatEnd = 0;}										//bc crossing = 3 once ped is clear)
								else {						//ped out of this lane, not yet clear
									threatBeg = 0;
									threatEnd = (pedY - lnBot + lnW)/k.maxV;}}
							else {							//ped in this lane
								threatBeg = 0;
								threatEnd = k.accT + (pedY - lnBot + lnW)/k.maxV;}}
						else {
							if (lane == 0) {
								if (dir == -1) {			//car in top lane, ped hasn't started to cross
									threatBeg = 0;			
									threatEnd = k.accT + 2*lnW/k.maxV;}
								else {						//car in bottom lane, ped clear after just this lane
									threatBeg	= (pedY - lnTop - lnW)/k.maxV;  //one lane buffer
									threatEnd	= (pedY - lnBot)/k.maxV;
									endGauntlet	= lnBot;}}
							else {
								threatBeg = (pedY - lnTop - lnW)/k.maxV;  //one lane buffer
								threatEnd = k.accT + (pedY - lnBot + lnW)/k.maxV;}}}
					if (threatBeg < 0) {
						threatBeg = 0;}			//correction for ped within current lane
					
					//decide whether or not to yield for this ped
					if (v != 0) {				//TTCol is undefined if car is already stopped
						if (TTCol < threatBeg) {	//can probably lose this first if
							if (TTCol <= decelT) {
								threatEnd = 0;}}					//driver recognizes ped is accepting rolling gap
						if (TTCol > threatEnd) {
							threatEnd = 0;}	//car reaches xwalk after ped is clear of next lane

						//calculate yielding acceleration
						if (threatEnd > 0) {
							if (threatEnd > tHardYield) {
								thisDecel = hardYield;
								thisYing = 1;}
							else {			//soft yield
								thisDecel = -2*(v*threatEnd - dist) / (threatEnd*threatEnd);
								thisYing  = 0;}
							Yieldage thisYield = new Yieldage(k,thisDecel,endGauntlet,thisYing);
							k.yielders.add(this);
							yieldage.add(thisYield);}
						else {
							thisYing = -1;}}
					else {
						if (age > 1 && threatEnd > 0) {
							thisDecel = 0;
							thisYing = 1;
							Yieldage thisYield = new Yieldage(k,thisDecel,endGauntlet,thisYing);
							k.yielders.add(this);
							yieldage.add(thisYield);}}}

				//update final acceleration value and yielding state
				if (thisDecel < yieldDec) {
					yieldDec = thisDecel;}
				if (thisYing > ying) {
					ying = thisYing;}}}
//			if (TTCol < j.xTime) {
//				ying = 1;}}}
		return yieldDec;
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
		//store parameters with heterogeneity (currently s.dev abitrarily = 8% of mean)
		//TODO: get theory for these numbers
		maxa	= rnd.nextGaussian()*(UserPanel.maxa*.08)+UserPanel.maxa;
		mina	= rnd.nextGaussian()*(UserPanel.mina*.08)+UserPanel.mina;
		maxv	= rnd.nextGaussian()*(.2*UserPanel.sLimit)+(UserPanel.sLimit);
		// TODO: stdDev of maxv is stretched to .2*mean to demonstrate car-following behavior
		tGap	= rnd.nextGaussian()*(UserPanel.tGap*.08)+UserPanel.tGap;
		jamHead	= rnd.nextGaussian()*(UserPanel.jamHead*.08)+UserPanel.jamHead;
		length	= UserPanel.carLength;
		delayTs	= UserPanel.delayTs;
		tN		= Math.floor(delayTs/UserPanel.tStep);
		tBeta	= (delayTs/UserPanel.tStep) - tN;	
		v		= maxv * (1 - .3*rnd.nextDouble());
		wS		= etaS = rnd.nextGaussian();
		wV = etaV = rnd.nextGaussian(); //TODO: separate these?
		sigR 	= 0.01; //standard deviation of relative approach rate TODO: should this vary?
		stopBar	= RoadBuilder.xWalkx - (double)dir*5/RoadBuilder.spaceScale; 
		// 5 here is exaggerated to view effects
		// should match pedbox (?) which is arbitrarily set to 2
		if (dir == 1) {
			if (lane == 0) {
				lnBot = RoadBuilder.sidewalk;}
			else {
				lnBot = RoadBuilder.sidewalk + RoadBuilder.laneW;}}
		else {
			if (lane == 0) {
				lnBot = RoadBuilder.sidewalk + 3*RoadBuilder.laneW;}
			else {
				lnBot = RoadBuilder.sidewalk + 2*RoadBuilder.laneW;}}
		lnTop = lnBot + RoadBuilder.laneW;
		age  = 0;
		ying = -1;
	}
	
	/**
	 * Getter for identification
	 */
	@Override
	public int isCar() {
		return 1;}
	
	/**
	 * Class for storing peds a car is yielding to for later reference
	 * @author Darryl Michaud
	 */
	class Yieldage {
		Ped yieldee;
		double calcAcc;
		double endThreat;
		int yState;
		Yieldage(Ped yieldee, double calcAcc, double endThreat, int yState) {
			this.yieldee = yieldee;
			this.calcAcc = calcAcc;
			this.endThreat = endThreat;
			this.yState = yState;}
	}
	
	/**
	 * Class for storing viewing angle to peds or cars for calculating double threat
	 * @author demi_chaud
	 *
	 */
	class ViewAngle {
		Ped ped;
		Turtle car;
		double theta;
		double theta1;
		double theta2;
		ViewAngle(Ped ped, double theta) {
			this.ped = ped;
			this.theta = theta;}
		ViewAngle(Turtle turtle, double theta1, double theta2) {
			this.car = turtle;
			this.theta1 = theta1;
			this.theta2 = theta2;}
	}
	
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
	@Parameter(usageName="yielding", displayName="yielding?")
	public int getYield() {
		return ying;}
	@Parameter(usageName="age", displayName="age")
	public int getAge() {
		return age;}
	@Parameter(usageName="TTC", displayName="TTC")
	public double getTTC() {
		return TTCol;}
}