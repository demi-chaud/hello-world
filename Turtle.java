package driving1;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import org.apache.commons.math3.util.FastMath;

import repast.simphony.parameter.Parameter;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;

import driving1.RedLight.state;

/**
* Vehicle agent class. Defines creation and actions.
* @author - Darryl Michaud
*/
public class Turtle extends Agent{
	ContinuousSpace<Object> space;
	private ArrayList<Turtle> sameDir, ahead, leaders, behind, followers;
	private ArrayList<Ped> crossingP, crossingP1, crossingP2, waitingP;
	private ArrayList<ViewAngle> obstructers, obstructees;
	private ArrayList<Yieldage> yieldage = new ArrayList<Yieldage>();
	private List<double[]> storage = new ArrayList<double[]>();
	private List<double[]> shldBrakeStorage = new ArrayList<double[]>();
	private Random  rnd  = new Random();	//initiates random number generator for vehicle properties
	private Random  rndD = new Random();	//ditto for distraction
	private Random	rndS = new Random();
	private Random  rndBRT = new Random();
	private Random	rndADRT = new Random();
	private Random  rndIDM = new Random();
	private boolean	distracted = false;
	private double	timeD, durD, timeSinceD, interD, interDlam;				//distraction
	private double	delayTs, tN, tBeta, BRTs, brt_tN, brtBeta;				//delay
	private double	tGap, jamHead, maxv, mina, maxa, newAcc, head, zIIDM;	//car-following
	private double	wS, etaS, wV, etaV, sigR;								//errors
	private double	confLim, stopBar, ttstopBar, lnTop, lnBot;				//yielding
	private double	hardYield, yieldDec;									//also yielding
	private double	carW = UserPanel.carWidth;
	private double  deltaIDM = 4;
	private double	tick;
	public  int		age, nMaxDecel;
	public  NdPoint	myLoc;
	public  Turtle	leader, follower;
	public	boolean connected, autonomous;
	public  double	v, vNew, acc, xLoc, yLoc, length, tail, driverX, driverY, decelT;
	public	double	stopDab;
	public  int		lane;	//  0 = outer lane, 1 = inner lane
	public	int		dir;	//  1 = going right, -1 = going left
	public	int		ying, oldYing;	// -1 = none, 0 = soft, 1 = hard
	
	/**
	* Calculates driver acceleration (if not distracted)
	* Sets value of acc & vNew
	*/
	public void calc() {
		tick	= Scheduler.thisTick;
		myLoc	= space.getLocation(this);
		xLoc	= myLoc.getX();
		newAcc  = 0;
		if (v > 3 * maxv) {
			int foo = 0;}
		if (xLoc > RoadBuilder.roadL/15 && xLoc < 14*RoadBuilder.roadL/15) {
			if (interD != 0 && timeSinceD >= interD) {
				if (ying == -1) { //don't get distracted if yielding
					distracted = true;
					timeSinceD = 0;
					interD = 0;}}
			if (durD != 0 && timeD >= durD) {
				distracted = false;
				timeD = 0;
				durD = 0;}
			if (autonomous == false && distracted == true) {
				if (timeD == 0) {
					double durD0 = rndD.nextGaussian()*UserPanel.DsigHat + UserPanel.DmuHat;
					durD = Math.exp(durD0);}
				newAcc = accel(myLoc, lane, dir, true);
				//newAcc = acc;
				timeD += 1;
				double xwalkD	= RoadBuilder.xWalkx - xLoc;
				double threat	= Math.signum(dir*xwalkD);
				if (threat == 1 && v != 0) {
					conflict();}}
			else {
				if (timeSinceD == 0) {
					double interD0 = (rndD.nextDouble() + 1E-15)*interDlam; //padded to avoid -inf
					interD = -Math.log(interD0/interDlam)/interDlam;}
				timeSinceD += 1;
				newAcc = accel(myLoc, lane, dir, false);}}
		else {
			newAcc = accel(myLoc, lane, dir, false);}
		
		//delayed CF reaction: implements acc calculated and stored delayT ago
		if (UserPanel.ADRT && autonomous == false) { //TODO: probably give non-zero value for automated
			double stamp, tStamp, delayedT, hiT;
			stamp  = RoadBuilder.clock.getTickCount();
			tStamp = stamp*UserPanel.tStep;
			tN		= Math.floor(1e-14 + delayTs/UserPanel.tStep);
			tBeta	= (delayTs/UserPanel.tStep) - tN;
			storage.add(new double[] {tStamp, newAcc, v});
			delayedT = tStamp - delayTs;
			hiT = 0;
			int foo = 0;
			int storSize = storage.size();
			if (storSize > 3 && storSize > tN+1) { 
				while (hiT < delayedT) {
					hiT = storage.get(foo)[0];
					foo++;}
				double hiAcc = storage.get(foo-1)[1];
				if (Math.abs(hiT - delayedT) > 1e-14) {	//linear interpolation TODO: is there a better approx?
					double loAcc = storage.get(foo-2)[1];
					acc = tBeta*loAcc + (1-tBeta)*hiAcc;}
				else acc = hiAcc;
				if (storSize > 3*tN) {		//TODO: make this less arbitrary
					storage.remove(0);}}
			else acc = newAcc;}
		else acc = newAcc;
		
		//delayed braking reaction
		double newbAccel = brake(myLoc, lane, dir);
		double oldbAccel;
		//if (UserPanel.BRT && autonomous == false) { //TODO: probably give non-zero value for automated
		if (UserPanel.BRT) {
			double stamp, tStamp, delayedT, hiT;
			stamp  = RoadBuilder.clock.getTickCount();
			tStamp = stamp*UserPanel.tStep;
			brt_tN	= Math.floor(1e-14 + BRTs/UserPanel.tStep);
			brtBeta	= (BRTs/UserPanel.tStep) - brt_tN;
			//TODO: limit size of storage to limit memory use
			shldBrakeStorage.add(new double[] {tStamp, newbAccel});
			delayedT = tStamp - BRTs;
			hiT = 0;
			int foo = 0;
			int storSize = shldBrakeStorage.size();
			if (storSize > 3 && storSize > brt_tN+1) { 
				while (hiT < delayedT) {
					hiT = shldBrakeStorage.get(foo)[0];
					foo++;}
				double hiAcc = shldBrakeStorage.get(foo-1)[1];
				if (Math.abs(hiT - delayedT) > 1e-14) {	//linear interpolation TODO: is there a better approx?
					double loAcc = shldBrakeStorage.get(foo-2)[1];
					oldbAccel = brtBeta*loAcc + (1-brtBeta)*hiAcc;}
				else oldbAccel = hiAcc;
				if (storSize > 3*brt_tN) {		//TODO: make this less arbitrary
					shldBrakeStorage.remove(0);}}
			else oldbAccel = newbAccel;}
		else oldbAccel = newbAccel;
		if (autonomous == false && distracted == false) {
			if (oldbAccel < acc && newbAccel < acc) {
				acc = newbAccel;}}
		age++;
		
		vNew = v + acc;
		if (vNew > 3 * maxv) {
			int foo = 0;}
		if (vNew < 0) {vNew = 0;}
	}		
	
	/**
	 * Moves the cars based on value created in calc()
	 */
	public void drive() {
		if (dir == 1) {
			if (xLoc + vNew >= RoadBuilder.roadL - 1) {
				Scheduler.killListC.add(this);
				this.storage.clear();}
			else if (vNew != 0) {
				//double displacement = v + .5*acc;		//TODO: replace this?
				space.moveByDisplacement(this,vNew,0);	// new version from Kesting, Treiber and Helbing 2009
				//space.moveByDisplacement(this,displacement,0);	//"Agents for Traffic Simulation"
				myLoc = space.getLocation(this);
				xLoc = myLoc.getX();}}
		else {
			if (xLoc - vNew <= 0) {
				Scheduler.killListC.add(this);
				this.storage.clear();}
			else if (vNew != 0) {
				//double displacement = -v - .5*acc;		//TODO: ditto
				space.moveByDisplacement(this,-vNew,0);
				//space.moveByDisplacement(this,displacement,0);
				myLoc = space.getLocation(this);
				xLoc = myLoc.getX();}}
		v = vNew;
		decelT = v/UserPanel.emergDec;
	}
	
	/**
	 * Calculates acceleration based on car-following
	 * @param loc, myLane, myDir
	 * @return a
	 */
	public double accel(NdPoint loc, int myLane, int myDir, boolean isDistracted) {
		double a, setSpeed, vDiff, safeHead, aFree;
		confLim	= UserPanel.confLimS/UserPanel.tStep;
		
		sameDir	= new ArrayList<Turtle>();
		ahead	= new ArrayList<Turtle>();
		leaders	= new ArrayList<Turtle>();
		head	= RoadBuilder.roadL;
		//TODO: limit accel to physically possible values
		
		//Determine leader and follower
		behind	  = new ArrayList<Turtle>();
		followers = new ArrayList<Turtle>();
		leader	  = null;
		tail	  = RoadBuilder.roadL;	
		for (Turtle p : Scheduler.allCars) {
			if (p.dir == myDir) {
				sameDir.add(p);}}
		if (!sameDir.isEmpty()) {
			for (Turtle m : sameDir) {
				if (myDir == 1) {
					if (m.xLoc > xLoc) {
						ahead.add(m);}
					if (m.xLoc < xLoc) {
						behind.add(m);}}
				else {
					if (m.xLoc < xLoc) {
						ahead.add(m);}
					if (m.xLoc > xLoc) {
						behind.add(m);}}}}
		if (!ahead.isEmpty()) {
			for (Turtle n : ahead) {
				if (n.lane == myLane) {
					leaders.add(n);}}}
		if (!behind.isEmpty()) {
			for (Turtle n : behind) {
				if (n.lane == myLane) {
					followers.add(n);}}}
		if (!leaders.isEmpty()) {
			for (Turtle o : leaders) {
				if((Math.abs(o.xLoc - xLoc) - o.length) < head) {
					head = Math.abs(o.xLoc - xLoc) - o.length; //TODO: change visualization to move icons to center of car
					if (head < 0) {
						head = 1e-10;}
					leader = o;}}}
		if (!followers.isEmpty()) {
			for (Turtle p : followers) {
				if((Math.abs(p.xLoc - xLoc) - length) < tail) {
					tail = Math.abs(p.xLoc - xLoc) - length;
					follower = p;}}}
		
		//calculate CF acceleration (with errors)
		if (leader != null) {
			if (UserPanel.estErr == true && autonomous == false) {		//includes estimation error in headway measurement
				etaS = rnd.nextGaussian();
				etaV = rnd.nextGaussian();
				wS = UserPanel.wien1*wS + UserPanel.wien2*etaS;
				wV = UserPanel.wien1*wV + UserPanel.wien2*etaV;
				head = head*Math.exp(UserPanel.Vs*wS);
				setSpeed = leader.v - head*sigR*wV;}
			else setSpeed = leader.v;
			vDiff = v - setSpeed;
			double safeHead0;
			safeHead0 = v*(tGap) + (v*vDiff)/(2*Math.sqrt(maxa*mina));
			safeHead = jamHead + Math.max(0,safeHead0);			//avoid negative values
			zIIDM = 1000;
			if (head != 0) {
				if (!isDistracted) {
					zIIDM = safeHead / head;}}
				//implement IIDM
				if (v < maxv) {
					aFree = maxa*(1-Math.pow(v/maxv,deltaIDM));
					if (zIIDM >= 1) {
						a = maxa*(1-zIIDM*zIIDM);}
					else {
						a = aFree*(1-Math.pow(zIIDM, 2*maxa/aFree));}}
				else {
					aFree = -mina*(1-Math.pow(maxv/v, maxa*deltaIDM/mina));
					if (zIIDM >= 1) {
						a = aFree + maxa*(1-zIIDM*zIIDM);}
					else {
						a = aFree;}}}
		else {a = maxa*(1 - Math.pow(v/maxv,deltaIDM));}
		if (xLoc > RoadBuilder.roadL/10 && xLoc < 9*RoadBuilder.roadL/10 && a < -UserPanel.emergDec) {
			a = -UserPanel.emergDec;
			nMaxDecel++;}
		else {
			nMaxDecel = 0;}
		return a;	
	}
	
	/*
	 * Calculates acceleration for yielding and red lights
	 */
	public double brake(NdPoint loc, int myLane, int myDir) {
		//Calculate yielding acceleration
		
		double aBrake = 100;
		double stopD	= stopBar - xLoc;
		stopDab	= dir*stopD;
		double xwalkD	= RoadBuilder.xWalkx - xLoc;
		double threat	= Math.signum(dir*xwalkD);
		if (threat == 1) {
			aBrake = yield(stopDab,xwalkD);}

		double rlAcc = 100;
		if (UserPanel.inclRL) {
			ArrayList<Turtle> queue = new ArrayList<Turtle>();
			ArrayList<Turtle> rlAhead = new ArrayList<Turtle>();
			double rlD;
			RedLight rl1 = RoadBuilder.rl1;
			RedLight rl2 = RoadBuilder.rl2;
			if (dir == 1 && xLoc < rl1.xLoc) {
				rlD = rl1.xLoc - xLoc;
				if (v != 0) {
					for (Turtle t : Scheduler.allCars) {
						if (t != this && t.dir == dir && t.lane == lane && t.xLoc <= rl1.xLoc && t.xLoc > xLoc) {
							rlAhead.add(t);}}
					if (rlAhead.size() < 2) {
						if (rl1.myState == state.RED) {
							rlAcc = -v*v/(2*rlD);}
						else if (rl1.myState == state.AMBER) {
							rlAcc = -v*v/(2*rlD);
							if (rlAcc < -mina) {
								rlAcc = 100;}}}}}
			else if (dir == -1 && xLoc > rl2.xLoc) {
				rlD = xLoc - rl2.xLoc;
				if (v != 0) {
					for (Turtle t : Scheduler.allCars) {
						if (t != this && t.dir == dir && t.lane == lane && t.xLoc >= rl2.xLoc && t.xLoc < xLoc) {
							rlAhead.add(t);}}
					if (rlAhead.size() < 2) {
						if (rl2.myState == state.RED) {
							rlAcc = -v*v/(2*rlD);}
						else if (rl2.myState == state.AMBER) {
							rlAcc = -v*v/(2*rlD);
							if (rlAcc < -mina) {
								rlAcc = 100;}}}}}}
		if (rlAcc < aBrake) {
			aBrake = rlAcc;}
		if (xLoc > RoadBuilder.roadL/10 && xLoc < 9*RoadBuilder.roadL/10 && aBrake < -UserPanel.emergDec) {
			aBrake = -UserPanel.emergDec;}
		return aBrake;
	}

	/**
	 * Determines driver yielding behavior, returns deceleration if necessary
	 * called within accel()
	 */
	public double yield(double stopDist, double conDist) { //dist is abs val, conDist is not (if stopDist is <0, car is passed stopbar)
		//TODO: add yielding to crossing=1 peds
		//TODO: add errors in distance and reading of ped v
		//TODO: rewrite error-making code as method in Agent class
//		waitingP	= new ArrayList<Ped>();
		crossingP	= new ArrayList<Ped>();
		crossingP1	= new ArrayList<Ped>();
		obstructers = new ArrayList<ViewAngle>();
		obstructees = new ArrayList<ViewAngle>();
		double	threatBeg, threatEnd, tHardYield;
		double	tCrash   = 0;
		double	lnW		 = RoadBuilder.laneW;
		yieldDec = 1;		//dummy value
		if (stopDist >=0) {
			if (v != 0) {
				tHardYield = 2*stopDist/v;
				ttstopBar  = stopDist/v;}
			else {
				ttstopBar  = 1e16;		//arbitrarily large
				tHardYield = 1e16;}
			hardYield	= -1000;
			if (stopDist != 0) {
				hardYield	= -v*v/(2*stopDist);}}
		else {							//driver has already passed stopbar (probably distracted)
			ttstopBar = -1;
			tHardYield = decelT;
			hardYield = -UserPanel.emergDec;
			if (v != 0) {
				tCrash = Math.abs(conDist/v);}}
		double tClear = -1;
		if (v != 0) {
			tClear = (Math.abs(conDist) + length)/v;}
		driverX		= xLoc-(double)dir*length/2;
		threatBeg 	= 0;
		threatEnd 	= -1;
		oldYing = ying; 
		ying = -1;
		
		//make list of waiting/crossing peds
		for (Ped i : Scheduler.allPeds) {
//			if (i.crossing == 1) {
//				waitingP.add(i);}
			if (i.crossing == 2) {
				crossingP.add(i);}}
		crossingP1 = crossingP; 		//crossingP will be depleted by double threats
		
		//double threat
		if (connected == false) {		//TODO: find accuracy of passive ped detection, add v2i functionality
			for (Turtle i : ahead) {				//are there any cars potentially blocking view?
				if (i.connected == false) {
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
							obstructers.add(thisView);}}}}
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
							crossingP.remove(thisPed);}}}}}
		
		//yield to crossing peds
		if (!crossingP.isEmpty()) {
			for (Ped k : crossingP) {
				Yieldage   oldVals = null;
				double		  pedY = k.yLoc;
				double	 thisDecel = 1;
				double endGauntlet = 0;
				double	  oldDecel = 0;
				double		clearY = 0;
				int		  thisYing = -1;
				
				//calculate relevant times for this ped (note: in OR, cars have to wait until ped is >1 lane away)
				for (Yieldage m : yieldage) {
					if (m.yieldee == k) {			//check if already yielding to ped
						oldDecel = m.calcAcc;
						clearY   = m.endThreat;
						oldVals  = m;
						break;}}
				//bring in old accel value if above is true
				if (oldVals != null) {
					double newDecel = 0;
					if (k.dir == 1 && clearY > pedY) {
						if (v != 0) {
							threatEnd = k.accT*(1-k.v[1]/k.maxV) + (clearY - pedY)/k.maxV;
							if (stopDist >= 0) {
								if (threatEnd > tHardYield) {
									newDecel = hardYield;
									thisYing = 1;}
								else {			//soft yield
									newDecel = 0;
									if (threatEnd != 0) {
										newDecel = -2*(v*threatEnd - stopDist) / (threatEnd*threatEnd);}
									thisYing = 0;}}
							else {
								newDecel = hardYield;
								thisYing = 1;}}
						else {
							thisDecel = 0;
							thisYing  = 1;}
						if (newDecel < oldDecel) {
							thisDecel = newDecel;
							oldVals.calcAcc = newDecel;}
						else {
							thisDecel = oldDecel;}
						if (thisYing > oldVals.yState) {
							oldVals.yState = thisYing;}}
					else if (k.dir == -1 && clearY < pedY) {
						if (v != 0) {
							threatEnd = k.accT*(1-Math.abs(k.v[1]/k.maxV)) + (pedY - clearY)/k.maxV;
							if (stopDist >= 0) {
								if (threatEnd > tHardYield) {
									newDecel = hardYield;
									thisYing = 1;}
								else {			//soft yield
									newDecel = 0;
									if (threatEnd != 0) {
										newDecel = -2*(v*threatEnd - stopDist) / (threatEnd*threatEnd);}
									thisYing = 0;}}
							else {
								newDecel = hardYield;
								thisYing = 1;}}
						else {
							thisDecel = 0;
							thisYing  = 1;}
						if (newDecel < oldDecel) {
							thisDecel = newDecel;
							oldVals.calcAcc = newDecel;}
						else {
							thisDecel = oldDecel;}
						if (thisYing > oldVals.yState) {
							oldVals.yState = thisYing;}}
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
					if (v != 0) {				//ttstop is undefined if car is already stopped
						if (stopDist >= 0) {
							if (tClear < threatBeg) {	//can probably lose this first if
								//if (ttstopBar <= decelT) { TODO: bring this back?
									threatEnd = 0;}
								//}					//driver recognizes ped is accepting rolling gap
							if (ttstopBar > threatEnd) {
								threatEnd = 0;}	//car reaches xwalk after ped is clear of next lane

							//calculate yielding acceleration
							if (threatEnd > 0) {
								if (threatEnd > tHardYield) {
									thisDecel = hardYield;
									thisYing = 1;}
								else {			//soft yield
									thisDecel = 0;
									if (threatEnd != 0) {
										thisDecel = -2*(v*threatEnd - stopDist) / (threatEnd*threatEnd);}
									thisYing  = 0;}
								Yieldage thisYield = new Yieldage(k,thisDecel,endGauntlet,thisYing);
								k.yielders.add(this);
								yieldage.add(thisYield);}
							else {
								thisYing = -1;}}
						else {
							if (tClear < threatBeg) {
								threatEnd = 0;}							//driver recognizes ped is accepting rolling gap
							if (threatEnd > 0 && threatBeg == 0) {
								thisDecel = hardYield;
								thisYing  = 1;
								Yieldage thisYield = new Yieldage(k,thisDecel,endGauntlet,thisYing);
								k.yielders.add(this);
								yieldage.add(thisYield);}
							else {
								thisYing = -1;}}}
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
		if (yieldDec < -UserPanel.emergDec) {
			yieldDec = -UserPanel.emergDec;}
		//take note of any conflicts
		if (!crossingP1.isEmpty()) {
			if (ttstopBar < confLim) {
				for (Ped n : crossingP1) {
					conflict(n);}}}
		return yieldDec;
	}
	
	/**
	 * store any conflicts with TTC < limit in userPanel
	 * @param p
	 * overloaded
	 */
	public void conflict(Ped p) {
		double pedX = p.xLoc;
		double pedY = p.yLoc;
		double pedTlo = -1;		//time until ped at conflict point
		double pedThi = -1;		//time until ped leaves CP
		double ttc = -1;
		if (v != 0) {
			ttc = (double)dir*(pedX - xLoc)/v;}
		if (ttc >= 0) {
			if (p.dir == 1 && pedY <= (yLoc + carW/2)) {
				if (pedY >= (yLoc - carW/2)) {
					pedTlo = 0;}
				else {
					pedTlo = 1000;
					if (p.v[1] != 0) {
						pedTlo = ((yLoc - carW/2) - pedY)/p.v[1];}}		//TODO: include ped r and make ped calc 2D
				pedThi = 1000;
				if (p.v[1] != 0) {
					pedThi = ((yLoc + carW/2) - pedY)/p.v[1];}}
			else if (p.dir == -1 && pedY >= (yLoc - carW/2)) {
				if (pedY <= (yLoc + carW/2)) {
					pedTlo = 0;}
				else {
					pedTlo = 1000;
					if (p.v[1] != 0) {
						pedTlo = -(pedY - (yLoc + carW/2))/p.v[1];}}		//TODO: include ped r and make ped calc 2D
				pedThi = 1000;
				if (p.v[1] != 0) {
					pedThi = -(pedY - (yLoc - carW/2))/p.v[1];}}
			if (pedTlo != -1) {
				if (ttc >= pedTlo && ttc <= pedThi) {
					if (ttc <= confLim) {
						int init = 1;
						int dup = 0;
						int hasDup = 0;
						double range = (double)dir*(pedX - xLoc);
						Conflict thisConf = new Conflict(this,p,ttc,range,oldYing,yieldDec,timeSinceD,timeD,init,hasDup,connected,autonomous);
						Conflict toAdd = null;
						Conflict toRem = null;
						for (Conflict c : Scheduler.allConf) {
							if (c.car == this && c.ped == p) {
								dup = 1;			//make sure no duplicates added unless lower TTC than first
								if (c.init == 1) {
									if (c.hasDup == 0) {
										if (ttc < c.TTC) {
											c.hasDup = 1;
											thisConf.init = 0;
											toAdd = thisConf;}}
									else {
										for (Conflict c1 : Scheduler.allConf) {
											if (c1.car == this && c1.ped == p) {
												if (c1 != c) {
													if (ttc < c1.TTC) {
														thisConf.init = 0;
														toAdd = thisConf;
														toRem = c1;}}}}}}}}
						if (toAdd != null) {
							Scheduler.allConf.add(thisConf);}
						if (toRem != null) {
							Scheduler.allConf.remove(toRem);}
						if (dup == 0) {
							Scheduler.allConf.add(thisConf);}}}}}
	}
	public void conflict() {
		double ttc	= 0;
		crossingP2	= new ArrayList<Ped>();
		for (Ped i : Scheduler.allPeds) {
			if (i.crossing == 2) {
				crossingP2.add(i);}}
		if (!crossingP2.isEmpty()) {
			if (ttstopBar < confLim) {
				for (Ped n : crossingP2) {
					conflict(n);
//					double pedX = n.xLoc;
//					double pedY	= n.yLoc;
//					double pedTlo = -1;		//time until ped at conflict point
//					double pedThi = -1;		//time until ped leaves CP
//					ttc	= (double)dir*(pedX - xLoc)/v;
//					if (ttc >= 0) {
//						if (n.dir == 1 && pedY <= (yLoc + carW/2)) {
//							if (pedY >= (yLoc - carW/2)) {
//								pedTlo = 0;}
//							else {
//								pedTlo = ((yLoc - carW/2) - n.yLoc)/n.v[1];}		//TODO: include ped r and make ped calc 2D
//							pedThi = ((yLoc + carW/2) - pedY)/n.v[1];}
//						else if (n.dir == -1 && pedY >= (yLoc - carW/2)) {
//							if (pedY <= (yLoc + carW/2)) {
//								pedTlo = 0;}
//							else {
//								pedTlo = -(n.yLoc - (yLoc + carW/2))/n.v[1];}		//TODO: include ped r and make ped calc 2D
//							pedThi = -(pedY - (yLoc - carW/2))/n.v[1];}
//						if (pedTlo != -1) {
//							if (ttc >= pedTlo && ttc <= pedThi) {
//								if (ttc < confLim) {
//									int init = 1;
//									int dup = 0;
//									int hasDup = 0;
//									double range = (double)dir*(pedX - xLoc);
//									Conflict thisConf = new Conflict(this,n,ttc,range,oldYing,acc,timeSinceD,timeD,init,hasDup);
//									Conflict toAdd = null;
//									Conflict toRem = null;
//									for (Conflict c : Scheduler.allConf) {
//										if (c.car == this && c.ped == n) {
//											dup = 1;			//make sure no duplicates added unless lower TTC than first
//											if (c.init == 1) {
//												if (c.hasDup == 0) {
//													if (ttc < c.TTC) {
//														c.hasDup = 1;
//														thisConf.init = 0;
//														toAdd = thisConf;}}
//												else {
//													for (Conflict c1 : Scheduler.allConf) {
//														if (c1.car == this && c1.ped == n) {
//															if (c1 != c) {
//																if (ttc < c1.TTC) {
//																	thisConf.init = 0;
//																	toAdd = thisConf;
//																	toRem = c1;}}}}}}}}
//									if (toAdd != null) {
//										Scheduler.allConf.add(thisConf);}
//									if (toRem != null) {
//										Scheduler.allConf.remove(toRem);}
//									if (dup == 0) {
//										Scheduler.allConf.add(thisConf);}}}}}
					}}}
	}
//	
//	public void crash(Ped p) {
//		Crash newCrash = new Crash(this,p,1,0);
//		Scheduler.crashes.add(newCrash);
//	}

	/**
	 * Creates vehicle agents and initializes values
	 * Called by scheduler in Agent.java
	 * @param contextSpace
	 * @param contextGrid
	 */

	public Turtle(ContinuousSpace<Object> contextSpace, int whichLane, int whichDir,
				  boolean conn, boolean auto) {
		space	= contextSpace;
		lane	= whichLane;
		dir		= whichDir;
		double maxa0 = rndIDM.nextGaussian()*UserPanel.maxaShape + UserPanel.maxaScale;
		maxa	= Math.exp(maxa0);
		double mina0 = rndIDM.nextGaussian()*UserPanel.minaShape + UserPanel.minaScale;
		mina	= Math.exp(mina0);
		maxv	= rnd.nextGaussian()*(UserPanel.sLimit_sd)+(UserPanel.sLimitMu);
		tGap	= rnd.nextGaussian()*(UserPanel.tGap_sd)+UserPanel.tGap;
		double jamHead0 = rndIDM.nextGaussian()*UserPanel.jamHeadShape + UserPanel.jamHeadScale;
		jamHead = Math.exp(jamHead0);
		//jamHead	= rnd.nextGaussian()*(UserPanel.jamHeadShape)+UserPanel.jamHeadScale;
		length	= UserPanel.carLength;
		if (autonomous == false) {
			BRTs = calcBRT() + 0.35;  // + 0.15 mvmt time (Lister 1950) + 0.2 device response time (Grover 2008)
			double delayT0 = rndADRT.nextGaussian() * 1.193759285934727 - 1.60692043370482;
			delayTs	= Math.exp(delayT0) + 0.25;}  
		else {
			BRTs = 0.51;  //seconds (source Grover 2008)
			delayTs = 0.4;}
		tN		= Math.floor(delayTs/UserPanel.tStep);
		tBeta	= (delayTs/UserPanel.tStep) - tN;
		brt_tN	= Math.floor(BRTs/UserPanel.tStep);
		brtBeta	= (BRTs/UserPanel.tStep) - brt_tN;
		v		= maxv * (1 - .3*rnd.nextDouble());
		decelT	= v/UserPanel.emergDec;
		wS		= etaS = rnd.nextGaussian();
		wV		= rnd.nextGaussian();
		etaV	= rnd.nextGaussian();
		sigR 	= 0.01*UserPanel.tStep; //standard deviation of relative approach rate
		stopBar	= calcStopBar(whichDir);
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
		lnTop		= lnBot + RoadBuilder.laneW;
		confLim		= UserPanel.confLimS/UserPanel.tStep;
		age			= 0;
		ying		= -1;
		timeD		= 0;
		durD		= 0;
		timeSinceD	= 0;
		interD		= 0;
		interDlam	= UserPanel.interDlam;
		connected	= conn;
		autonomous	= auto;
	}
	
	/* Fits driver's distance from stopBar to lognormal distribution */
	public double calcStopBar(int dir) {
		double dist0 = rndS.nextGaussian()*UserPanel.SsigHat + UserPanel.SmuHat;
		double stopDistance0 = Math.exp(dist0);
		if (stopDistance0 > 11) {
			stopDistance0 = 11;}
		double stopDistance = RoadBuilder.xWalkx - (double)dir*stopDistance0;
		return stopDistance;
	}
	
	
	/* Fits human driver's brake reaction time to shifted Weibull distribution */
	public double calcBRT() {
		double scaler = 1-(1E-15);
		double brt0 = scaler * rndBRT.nextDouble();
		double brt00 = -Math.log(1-brt0);
		double exp = 1/2.435;
		double brt = 0.25 + 1.2*Math.pow(brt00,exp);
		return brt; 
	}
	
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
	
	public class Conflict {
		Ped ped;
		Turtle car;
		int dirP, dirC, lane, ying, init, hasDup;
		double TTC, range, yDec, vel, timeD, sinceD, tick;
		boolean conn, auto;
		ArrayList<double[]> pedVid;
		ArrayList<Video> video;
		double xWalkx = RoadBuilder.xWalkx;
		double spaceScale = UserPanel.spaceScale;
		Conflict(Turtle car, Ped ped, double ttc, double range, int yieldState, double yieldDec, 
				double timeSinceD, double timeD, int init, int hasDup, boolean conn, boolean auto) {
			this.ped	= ped;
			this.car	= car;
			this.dirP	= ped.dir;
			this.dirC	= car.dir;
			this.lane	= car.lane;
			this.TTC	= ttc;
			this.range	= range;
			this.ying	= yieldState;
			this.vel	= car.v;
			this.yDec	= yieldDec;
			this.tick	= RoadBuilder.clock.getTickCount();
			this.sinceD	= timeSinceD;
			this.timeD	= timeD;
			this.init	= init;
			this.hasDup	= hasDup;
			this.conn	= conn;
			this.auto	= auto;
			this.video	= new ArrayList<Video>();
			this.pedVid = new ArrayList<double[]>();
			double[] thisPedVid = new double[2];
			ped.myLoc.toDoubleArray(thisPedVid);
			this.pedVid.add(thisPedVid);
//			Iterable<Object> snapShot = space.getObjects();
//			Iterator<Object> iterator = snapShot.iterator();
			for (Turtle t : Scheduler.allCars) { 
//			while (iterator.hasNext()) {
//				Object element0 = iterator.next();
//				Agent element = (Agent)element0;
//				if (element.isCar()) {
//					Turtle thisCar = (Turtle)element;
				if (t.myLoc != null && t.dir == car.dir) {
					double[] location = new double[2];
					t.myLoc.toDoubleArray(location);
					double thirty = 30/spaceScale; //30m should be enough to see all relevant cars
					if (location[0] >= xWalkx - thirty && location[0] <= xWalkx + thirty) {
						Video thisVideo = new Video(t,location);
						this.video.add(thisVideo);}}}}
	}
	
	public class Video {
		Turtle car;
		ArrayList<double[]> locs;
		Video(Turtle agent, double[] loc) {
			this.car = agent;
			this.locs = new ArrayList<double[]>();
			this.locs.add(loc);}
	}
	
//	public class Crash {
//		Ped ped;
//		Turtle car;
//		int dirP, dirC, lane, init, hasDup;
//		double v, curDec, sinceDist;
//		boolean distracted;
//		Crash(Turtle car, Ped ped, int init, int hasDup) {
//			this.ped	= ped;
//			this.car	= car;
//			this.dirP	= ped.dir;
//			this.dirC	= car.dir;
//			this.lane	= car.lane;
//			this.v		= car.v;
//			this.init	= init;
//			this.hasDup = hasDup;}
//	}
	
	/**
	 * Getter for identification
	 */
	@Override
	public boolean isCar() {
		return true;}
	
	/**
	 * Parameter declarations for probe
	 */
	@Parameter(usageName="newAcc", displayName="Calculated acc")
	public double getAcc() {
		return newAcc*UserPanel.spaceScale/(UserPanel.tStep*UserPanel.tStep);}
	@Parameter(usageName="v", displayName="Current vel")
	public double getVel() {
		return v*UserPanel.vBase;}
	@Parameter(usageName="maxv", displayName="Max vel")
	public double getMaxV() {
		return maxv*UserPanel.vBase;}
	@Parameter(usageName="yielding", displayName="yielding?")
	public int getYield() {
		return ying;}
	@Parameter(usageName="age", displayName="age")
	public int getAge() {
		return age;}
	@Parameter(usageName="TTC", displayName="TTC")
	public double getTTC() {
		return ttstopBar*UserPanel.tStep;}
	@Parameter(usageName="dir", displayName="dir")
	public int getDir() {
		return dir;}
	@Parameter(usageName="lane", displayName="lane")
	public int getLane() {
		return lane;}
	@Parameter(usageName="xLoc", displayName="xLoc")
	public double getX() {
		return xLoc;}
	@Parameter(usageName="stopD", displayName="stopD")
	public double getStopD() {
		return stopDab;}
	@Parameter(usageName="dist", displayName="distr?")
	public boolean getDistr() {
		return distracted;}
	@Parameter(usageName="headT", displayName="headT")
	public double getHeadT() {
		if (v != 0) {
			return (head/v)*UserPanel.tStep;}
		else return 0;}
}