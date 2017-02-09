package driving1;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

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
	private ArrayList<Yieldage> yieldage = new ArrayList<Yieldage>();
	private List<double[]> storage = new ArrayList<double[]>();
	private Random  rnd = new Random();	//initiates random number generator for vehicle properties
	private boolean	distracted = false; // TODO:figure out how to implement this
	private double	delayTs, tN, tBeta;							//delay
	private double	tGap, jamHead, maxv, mina, maxa, newAcc;	//car-following
	private double	wS, etaS, wV, etaV, sigR;					//errors
	private double	stopBar, TTCol, lnTop, lnBot;				//yielding
	private int		age;
	public  NdPoint	myLoc;
	public  Turtle	leader, follower;
	public  double	v, vNew, acc, xLoc, length, tail;
	public  int		lane;	//  0 = outer lane, 1 = inner lane
	public	int		dir;	//  1 = going right, -1 = going left
	public	int		ying;	// -1 = none, 0 = soft, 1 = hard
//	double vision;
	
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
			else acc = newAcc;
			age++;}
		else acc = newAcc;
		
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
			vDiff = setSpeed - v;
			safeHead = (jamHead + v*(tGap) +
					((v*vDiff)/(2*Math.sqrt(maxa*mina))));
			a = maxa*(1 - Math.pow(v/maxv,4) - Math.pow(safeHead/head,2));}
		else {a = maxa*(1 - Math.pow(v/maxv,4));}
		
		//Calculate yielding acceleration
		double xwalkD	= stopBar - thisX;
		double xwalkDab	= Math.abs(xwalkD);
		double threat	= Math.signum(dir*xwalkD);
		if (threat == 1) {
			double a0 = yield(xwalkDab);
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
		double	threatBeg, threatEnd;
		double	lnW		 = RoadBuilder.laneW;
		double	decelT	 = v/mina;
		double	yieldDec = 1;		//dummy value
		double	hardYield	= -v*v/(2*dist);
		double	tHardYield	= 2*dist/v;
		TTCol = dist/v;
		threatBeg = 0;
		threatEnd = -1;
		int thisYing = -1;
		
		//make list of waiting/crossing peds
		for (Ped i : Scheduler.allPeds) {
			if (i.crossing == 1) {
				waitingP.add(i);}
			if (i.crossing == 2) {
				crossingP.add(i);}}
		
		//TODO: add yielding to waiting peds
		if (!crossingP.isEmpty()) {
			for (Ped j : crossingP) {
				double pedY = j.yLoc;
				double thisDecel = 1;
				double endGauntlet = 0;
				double oldDecel, clearY;
				Yieldage oldVals = null;
				oldDecel = clearY = 0;
				
				//calculate relevant times for this ped (note: in OR, cars have to wait until ped is >1 lane away)
				for (Yieldage k : yieldage) {
					if (k.yieldee == j) {			//check if already yielding to ped
						oldDecel = k.calcAcc;
						clearY   = k.endThreat;
						oldVals  = k;
						break;}}
				
				//bring in old accel value if above is true
				if (oldVals != null) {
					double newDecel;
					if (j.dir == 1 && clearY > pedY) {
						threatEnd = j.accT*(1-j.v[1]/j.maxV) + (clearY - pedY)/j.maxV;
						if (threatEnd > tHardYield) {
							newDecel = hardYield;
							thisYing = 1;}
						else {
							newDecel = -2*(v*threatEnd - dist) / (threatEnd*threatEnd);
							thisYing = 0;}
						if (newDecel < oldDecel) {
							thisDecel = newDecel;
							oldVals.calcAcc = newDecel;}
						else {
							thisDecel = oldDecel;}}
					else if (j.dir == -1 && clearY < pedY) {
						threatEnd = j.accT*(1-Math.abs(j.v[1]/j.maxV)) + (pedY - clearY)/j.maxV;
						if (threatEnd > tHardYield) {
							newDecel = hardYield;
							thisYing = 1;}
						else {
							newDecel = -2*(v*threatEnd - dist) / (threatEnd*threatEnd);
							thisYing = 0;}
						if (newDecel < oldDecel) {
							thisDecel = newDecel;
							oldVals.calcAcc = newDecel;}
						else {
							thisDecel = oldDecel;}}
					else {
						yieldage.remove(oldVals);
						thisYing = -1;}}
				
				//calculate necessary accel and add ped to list of yieldees
				else {
					if (j.dir == 1) {					//ped walking up
						endGauntlet = lnTop + lnW;
						if (pedY > lnBot) {
							if (pedY > lnTop) {
								if (pedY > lnTop + lnW) {	//ped already out of danger (lane buffer irrelevant in top lane
									threatEnd = 0;}										//bc crossing = 3 once ped is clear)
								else {						//ped out of this lane, not yet clear
									threatBeg = 0;
									threatEnd = (lnTop + lnW - pedY)/j.maxV;}}
							else {							//ped in this lane
								threatBeg = 0;
								threatEnd = j.accT + (lnTop + lnW - pedY)/j.maxV;}}
						else {
							if (lane == 0) {
								if (dir == 1) {				//car in bottom lane, ped hasn't started to cross
									threatBeg = 0;
									threatEnd = j.accT + 2*lnW/j.maxV;}
								else {						//car in top lane, ped clear after just this lane
									threatBeg	= (lnBot - lnW - pedY)/j.maxV;  //one lane buffer
									threatEnd	= (lnTop - pedY)/j.maxV;
									endGauntlet	= lnTop;}}
							else {							//ped in or below same lane
								threatBeg = (lnBot - lnW - pedY)/j.maxV;
								threatEnd = j.accT + (lnTop + lnW - pedY)/j.maxV;}}}
					else {								//ped walking down
						endGauntlet = lnBot - lnW;
						if (pedY < lnTop) {
							if (pedY < lnBot) {
								if (pedY < lnBot - lnW) {	//ped already out of danger (lane buffer irrelevant in bottom lane
									threatEnd = 0;}										//bc crossing = 3 once ped is clear)
								else {						//ped out of this lane, not yet clear
									threatBeg = 0;
									threatEnd = (pedY - lnBot + lnW)/j.maxV;}}
							else {							//ped in this lane
								threatBeg = 0;
								threatEnd = j.accT + (pedY - lnBot + lnW)/j.maxV;}}
						else {
							if (lane == 0) {
								if (dir == -1) {			//car in top lane, ped hasn't started to cross
									threatBeg = 0;			
									threatEnd = j.accT + 2*lnW/j.maxV;}
								else {						//car in bottom lane, ped clear after just this lane
									threatBeg	= (pedY - lnTop - lnW)/j.maxV;  //one lane buffer
									threatEnd	= (pedY - lnBot)/j.maxV;
									endGauntlet	= lnBot;}}
							else {
								threatBeg = (pedY - lnTop - lnW)/j.maxV;  //one lane buffer
								threatEnd = j.accT + (pedY - lnBot + lnW)/j.maxV;}}}
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
							Yieldage thisYield = new Yieldage(j,thisDecel,endGauntlet);
							j.yielders.add(this);
							yieldage.add(thisYield);}
						else {
							thisYing = -1;}}
					else {
						if (age > 1 && threatEnd > 0) {
							thisDecel = 0;
							thisYing = 1;
							Yieldage thisYield = new Yieldage(j,thisDecel,endGauntlet);
							j.yielders.add(this);
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
		Yieldage(Ped yieldee, double calcAcc, double endThreat) {
			this.yieldee = yieldee;
			this.calcAcc = calcAcc;
			this.endThreat = endThreat;}
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
	@Parameter(usageName="lane", displayName="Current lane")
	public double getLane() {
		return lane;}
	@Parameter(usageName="leader", displayName="Current leader")
	public Turtle getLead() {
		return leader;}
//	@Parameter(usageName="yielding", displayName="yielding?")
//	public int getYield() {
//		return ying;}
	@Parameter(usageName="age", displayName="age")
	public int getAge() {
		return age;}
	@Parameter(usageName="TTC", displayName="TTC")
	public double getTTC() {
		return TTCol;}
}