package driving1;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
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
	private ArrayList<Ped> crossingP, crossingP2;
	public  Map<Ped,Double> blockedPeds;					//most recent tick with blockage
	private ArrayList<ViewAngle> obstructers, obstructees;
//	private ArrayList<Yieldage> yieldage = new ArrayList<Yieldage>();
//	private ArrayList<Yieldage> cYields = new ArrayList<Yieldage>();
//	private ArrayList<Yieldage> pYields = new ArrayList<Yieldage>();
	private Map<Integer,ArrayList<Yieldage>> delayedYields = new HashMap<Integer,ArrayList<Yieldage>>();
	private List<double[]> storage = new ArrayList<double[]>();
	private List<double[]> shldBrakeStorage = new ArrayList<double[]>();
	private Random  rnd  = new Random();	//initiates random number generator for vehicle properties
	private Random  rndD = new Random();	//ditto for distraction
	private Random	rndS = new Random();
	private Random  rndBRT = new Random();
	private Random	rndADRT = new Random();
	private Random  rndIDM = new Random();
	private boolean	distracted = false;
	private double	timeD, durD, timeSinceD, interD, interDlam;						//distraction
	private double	delayTs, tN, tBeta, brt_tN, brtBeta;						//delay
	private double	tGap, jamHead, maxv, mina, maxa, head, zIIDM, accCAH;	//car-following
	private double	wS, etaS, wV, etaV, sigR, wPed, etaPed, wPedV, etaPedV;			//errors
	private double	confLim, stopBar, ttstopBar, realTtStopBar, lnTop, lnBot;		//yielding
	private double	hardYield, cYieldD, yieldDec, percLimit, percV;					//also yielding
	private double	carW = UserPanel.carWidth;
	private double  deltaIDM = 4;
	//private double	tick;
	public  int		age, nMaxDecel;
	public  NdPoint	myLoc;
	public  Turtle	leader, follower;
	public	boolean connected, autonomous;
	public  double	v, vNew, acc, newAcc, xLoc, yLoc, length, tail, driverX, driverY, decelT, BRTs;
	public	double	stopDab;
	public  int		lane;	//  0 = outer lane, 1 = inner lane
	public	int		dir;	//  1 = going right, -1 = going left
	public	int		ying;	// -1 = none, 0 = soft, 1 = hard
	
	/**
	* Calculates driver acceleration (if not distracted)
	* Sets value of acc & vNew
	*/
	public void calc() {
		//tick	= Scheduler.thisTick;
		myLoc	= space.getLocation(this);
		xLoc	= myLoc.getX();
		newAcc  = 0;
		if (xLoc > RoadBuilder.roadL/15 && xLoc < 14*RoadBuilder.roadL/15) {
			if (v > 2 * maxv) {
				int foo = 0;}
			if (interD != 0 && timeSinceD >= interD && !autonomous) {
				if (ying == -1) { //don't get distracted if yielding
					distracted = true;
					timeSinceD = 0;
					interD = 0;}}
			if (durD != 0 && timeD >= durD && !autonomous) {
				distracted = false;
				timeD = 0;
				durD = 0;}
			if (!autonomous && distracted) {
				if (timeD == 0) {
					double durD0 = rndD.nextGaussian()*UserPanel.DsigHat + UserPanel.DmuHat;
					durD = Math.exp(durD0);}
				newAcc = accel(myLoc, lane, dir, true);
				timeD += 1;
//				double xwalkD	= RoadBuilder.xWalkx - xLoc;
//				double threat	= Math.signum(dir*xwalkD);
//				if (threat == 1 && v != 0) {
//					conflict();}
				}
			else {
				if (timeSinceD == 0) {
					double interD0 = (rndD.nextDouble() + 1E-15)*interDlam; //padded to avoid -inf
					interD = -Math.log(interD0/interDlam)/interDlam;}
				timeSinceD += 1;
				newAcc = accel(myLoc, lane, dir, false);}}
		else {
			newAcc = accel(myLoc, lane, dir, false);}
		
		//delayed CF reaction: implements acc calculated and stored delayT ago
		if (UserPanel.ADRT) {
			double[] delayedValuesAcc = delayValue(storage,newAcc,v);
			acc = delayedValuesAcc[1];}
		else {
			acc = newAcc;}
		
		//delayed braking reaction
		int foo = 0;
		double[] brakeOutput = brake(myLoc, lane, dir); 
		double newbAccel = brakeOutput[0];
		double oldbAccel;
//		if (UserPanel.BRT) {
//			double[] delayedValues = delayValue(shldBrakeStorage,newbAccel,brakeOutput[1]);
//			oldbAccel = delayedValues[1];
//			ying = (int)Math.round(delayedValues[2]);}
//		else {
			oldbAccel = newbAccel;
			//yieldDec = newbAccel;
			ying = (int)Math.round(brakeOutput[1]);
//			}
		if (!distracted || autonomous) {
			if (oldbAccel <= 0 && newbAccel <= 0) {
				if (oldbAccel < acc && newbAccel < acc) {
					yieldDec = newbAccel;
					acc = newbAccel;}}}
		age++;
		if (acc == -UserPanel.emergDec) {
			nMaxDecel++;}
		else {
			nMaxDecel = 0;}
		vNew = v + acc;
		if (vNew < 0) {vNew = 0;}
		
		//moved here. TODO: check results
		double xwalkD	= RoadBuilder.xWalkx + ((double)dir * Ped.xWalkHalfWidth) - xLoc;
		double threat	= Math.signum(dir*xwalkD);
		if (threat == 1) {
			conflict();}
	}		
	
	/**
	 * Moves the cars based on value created in calc()
	 */
	public void drive() {
		if (dir == 1) {
			if (xLoc + vNew >= RoadBuilder.roadL - 1) {
				RoadBuilder.flowSource.killListC.add(this);
				this.storage.clear();}
			else if (vNew != 0) {
				//double displacement = v + .5*acc;		//TODO: replace this?
				space.moveByDisplacement(this,vNew,0);	// new version from Kesting, Treiber and Helbing 2009
				//space.moveByDisplacement(this,displacement,0);	//"Agents for Traffic Simulation"
				myLoc = space.getLocation(this);
				xLoc = myLoc.getX();
				driverX = xLoc - (double)dir*length/2;}}
		else {
			if (xLoc - vNew <= 0) {
				RoadBuilder.flowSource.killListC.add(this);
				this.storage.clear();}
			else if (vNew != 0) {
				//double displacement = -v - .5*acc;		//TODO: ditto
				space.moveByDisplacement(this,-vNew,0);
				//space.moveByDisplacement(this,displacement,0);
				myLoc = space.getLocation(this);
				xLoc = myLoc.getX();
				driverX = xLoc - (double)dir*length/2;}}
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
		for (Turtle p : RoadBuilder.flowSource.allCars) {
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
			if (UserPanel.estErr /*&& !autonomous*/) {		//includes estimation error in headway measurement
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
					a = aFree;}}
			
			double cool = 0.99;
			if (head != 0) {
				if (!isDistracted) {
					double effAcc = Math.min(acc, leader.acc);
					accCAH = effAcc;
					if (vDiff > 0) {
						accCAH = effAcc - Math.pow(vDiff, 3)/(2*head);}
					if (leader.v*vDiff <= -2*head*effAcc) {
						if (leader.v*leader.v != 2*head*effAcc) {
							accCAH = v*v*effAcc/(leader.v*leader.v - 2*head*effAcc);}
						else {
							accCAH = effAcc;}}}
				if (a < accCAH) {
					a = (1-cool)*a + cool*(accCAH + mina*Math.tanh((a-accCAH)/mina));}}}
		else {a = maxa*(1 - Math.pow(v/maxv,deltaIDM));}
		if (xLoc > RoadBuilder.roadL/10 && xLoc < 9*RoadBuilder.roadL/10 && a < -UserPanel.emergDec) {
			a = -UserPanel.emergDec;}
		return a;	
	}
	
	/*
	 * Calculates acceleration for yielding and red lights
	 */
	public double[] brake(NdPoint loc, int myLane, int myDir) {
		//Calculate yielding acceleration
		
		double aBrake 	= 100;
		double stopD	= stopBar - xLoc;
		stopDab	= dir*stopD;
		double xwalkD	= RoadBuilder.xWalkx - xLoc;
		double threat	= Math.signum(dir*xwalkD);
		double[] yieldOutput = new double[2];
		if (threat == 1) {
			yieldOutput = yield(stopDab,xwalkD); 
			aBrake = yieldOutput[0];}
		else {
			yieldOutput[1] = -1;}

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
					for (Turtle t : RoadBuilder.flowSource.allCars) {
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
					for (Turtle t : RoadBuilder.flowSource.allCars) {
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
		double[] rv = new double[] {aBrake,yieldOutput[1]};
		return rv;
	}

	/**
	 * Determines driver yielding behavior, returns deceleration if necessary
	 * called within accel()
	 */
	public double[] yield(double stopDist, double conDist) { //dist is abs val, conDist is not (if stopDist is <0, car is passed stopbar)
		//TODO: add errors in distance and reading of ped v
		//TODO: rewrite error-making code as method in Agent class
		crossingP	= new ArrayList<Ped>();
		obstructers = new ArrayList<ViewAngle>();
		obstructees = new ArrayList<ViewAngle>();
		//pYields = new ArrayList<Yieldage>(cYields);
		ArrayList<Yieldage> pYields = new ArrayList<Yieldage>();
		ArrayList<Yieldage> cYields = new ArrayList<Yieldage>();
		ArrayList<Yieldage> reactTo = new ArrayList<Yieldage>();
		double threatBeg, threatEnd, tHardYield, outYing;
		double lnW		 = RoadBuilder.laneW;
		double realStopDist = stopDist;
		double realConDist	= conDist;
		int stamp = (int)RoadBuilder.clock.getTickCount();
		if (UserPanel.estErr /* && !autonomous*/) {		//includes estimation error
			etaPed = rnd.nextGaussian();
			etaPedV = rnd.nextGaussian();
			wPed = UserPanel.wien1*wPed + UserPanel.wien2*etaPed;
			wPedV = UserPanel.wien1*wPedV + UserPanel.wien2*etaPedV;
			stopDist = stopDist*Math.exp(UserPanel.Vs*wPed);
			conDist  = conDist*Math.exp(UserPanel.Vs*wPed);
			if (v != 0) {
				percV = v - conDist*sigR*wPedV;}
			else {
				percV = v;}}
		else percV = v;
		if (percV < 0) {
			percV = 0;}
		if (realStopDist >= 0) {
			if (stopDist < 0) {
				int foo = 0;}
			if (percV != 0) {
				tHardYield = 2*stopDist/percV;
				if (-percV * tHardYield > UserPanel.emergDec) {
					tHardYield = 1e-12;}
				ttstopBar  = stopDist/percV;
				if (v != 0) {
					realTtStopBar = realStopDist/v;}
				else {
					realTtStopBar = 1e12;}}
			else {
				ttstopBar  = 1e12;		//arbitrarily large
				realTtStopBar = 1e12;
				tHardYield = 1e12;}
			hardYield	= -1000;
			if (stopDist != 0) {
				hardYield	= -percV*percV/(2*stopDist);}}
		else {							//driver has already passed stopbar (probably distracted)
			ttstopBar = -1;
			realTtStopBar = -1;
			tHardYield = decelT;
			hardYield = -UserPanel.emergDec;}
		double tClear = -1;
		if (percV != 0) {
			tClear = (Math.abs(conDist) + length)/percV;}
		threatBeg 	= 0;
		threatEnd 	= -1;
		outYing = -1;
		cYieldD = 1;		//dummy value
		//stopDist = realStopDist;
		//conDist = realConDist;
		
		//make list of waiting/crossing peds
		for (Ped i : RoadBuilder.flowSource.allPeds) {
			if (i.crossing == 2) {
				crossingP.add(i);}}
		
		//perception distance
		ArrayList<Ped> tooFar = new ArrayList<Ped>();
		for (Ped d : crossingP) {
			double pDist = Math.abs(xLoc - d.xLoc);
			if (pDist > percLimit) {
				tooFar.add(d);}}
		if (!tooFar.isEmpty()) {
			crossingP.removeAll(tooFar);}
		
		//double threat
		for (Turtle i : ahead) {				//are there any cars potentially blocking view?
			if (!connected || !i.connected) {		//TODO: find accuracy of passive ped detection, add v2i functionality
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
						blockedPeds.put(thisPed, (double)stamp);
						crossingP.remove(thisPed);}}}}
		
		int lastKey;
		if (!delayedYields.isEmpty()) {
			lastKey = Collections.max(delayedYields.keySet());
			pYields = delayedYields.get(lastKey);}
		
		//yield to crossing peds
		if (!crossingP.isEmpty()) {
			for (Ped k : crossingP) {
				
//				if (k.v[1] == 0) {
//					
//				}
				
				Yieldage   oldVals = null;
				double		  pedY = k.yLoc;
				double	 thisDecel = 0;
				double endGauntlet = 0;
				int		  thisYing = -1;
				
				//calculate relevant times for this ped (note: in OR, cars have to wait until ped is >1 lane away)
				for (Yieldage m : pYields) {
					if (m.yieldee == k) {			//check if already yielding to ped
						endGauntlet = m.endThreat;
						oldVals = new Yieldage(m);
						break;}}
				//bring in old accel value if above is true
				if (oldVals != null) {
					double newDecel = 1;
					if (k.dir == 1 && endGauntlet > pedY) {
						if (v > 0) {
							//threatEnd = k.accT*(1-k.v[1]/k.maxV) + (endGauntlet - pedY)/k.maxV;
							threatEnd = (endGauntlet - pedY)/Math.abs(k.v[1]);
							if (k.v[1] == 0) {
								threatEnd = 1e12;}
							if (realStopDist >= 0) {
								if (stopDist < 0) {
									int foo = 0;}
								if (threatEnd > tHardYield) {
									newDecel = hardYield;
									thisYing = 1;}
								else {			//soft yield
									newDecel = 0;
									if (threatEnd != 0) {
										newDecel = -2*(percV*threatEnd - stopDist) / (threatEnd*threatEnd);
										if (newDecel > 0) newDecel = 0;}
									thisYing = 0;}}
							else {
								newDecel = hardYield;
								thisYing = 1;}}
						else {
							newDecel = 0;
							thisYing  = 1;}
						thisDecel = newDecel;
						oldVals.calcAcc = newDecel;
						if (thisYing > oldVals.yState) { //TODO: this seems suspect as well
							oldVals.yState = thisYing;}}
					else if (k.dir == -1 && endGauntlet < pedY) {
						if (percV > 0) {
							//threatEnd = k.accT*(1-Math.abs(k.v[1]/k.maxV)) + (pedY - endGauntlet)/k.maxV;
							threatEnd = (pedY - endGauntlet)/Math.abs(k.v[1]);
							if (k.v[1] == 0) {
								threatEnd = 1e12;}
							if (realStopDist >= 0) {
								if (stopDist < 0) {
									int foo = 0;}
								if (threatEnd > tHardYield) {
									newDecel = hardYield;
									thisYing = 1;}
								else {			//soft yield
									newDecel = 0;
									if (threatEnd != 0) {
										newDecel = -2*(percV*threatEnd - stopDist) / (threatEnd*threatEnd);
										if (newDecel > 0) newDecel = 0;}
									thisYing = 0;}}
							else {
								newDecel = hardYield;
								thisYing = 1;}}
						else {
							newDecel = 0;
							thisYing  = 1;}
						thisDecel = newDecel;
						oldVals.calcAcc = newDecel;
						if (thisYing > oldVals.yState) {
							oldVals.yState = thisYing;}}
					else {
						continue;}
					cYields.add(oldVals);}
				
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
									if (k.v[1] != 0) {
										threatEnd = (lnTop + lnW - pedY)/Math.abs(k.v[1]);}
									else {
										threatEnd = 1e12;}}}
									//threatEnd = (lnTop + lnW - pedY)/k.maxV;}}
							else {							//ped in this lane
								threatBeg = 0;
								if (k.v[1] != 0) {
									threatEnd = (lnTop + lnW - pedY)/Math.abs(k.v[1]);}
								else {
									threatEnd = 1e12;}
								//threatEnd = k.accT + (lnTop + lnW - pedY)/k.maxV;}}
							}}
						else {
							if (lane == 0) {
								if (dir == 1) {				//car in bottom lane, ped hasn't started to cross
									threatBeg = 0;
									if (k.v[1] != 0) {
										threatEnd = 2*lnW/Math.abs(k.v[1]);}
									else {
										threatEnd = k.accT + 2*lnW/k.maxV;}
									//threatEnd = k.accT + 2*lnW/k.maxV;
									}
								else {						//car in top lane, ped clear after just this lane
									if (k.v[1] != 0) {
										threatBeg	= (lnBot - lnW - pedY)/Math.abs(k.v[1]);  //one lane buffer
										threatEnd	= (lnTop + lnW/2 - pedY)/Math.abs(k.v[1]);
										endGauntlet	= lnTop + lnW/2;}
									else {
										threatBeg	= (lnBot - lnW - pedY)/k.maxV;  //one lane buffer
										threatEnd	= 1e12;
										endGauntlet	= lnTop + lnW/2;}
//									threatBeg	= (lnBot - lnW - pedY)/k.maxV;  //one lane buffer
//									threatEnd	= (lnTop - pedY)/k.maxV;
//									endGauntlet	= lnTop;
									}}
							else {							//ped in or below same lane
								if (k.v[1] != 0) {
									threatBeg = (lnBot - lnW - pedY)/Math.abs(k.v[1]);
									threatEnd = k.accT + (lnTop + lnW - pedY)/Math.abs(k.v[1]);}
								else {
									threatBeg = (lnBot - lnW - pedY)/k.maxV;
									threatEnd = 1e12;}
//								threatBeg = (lnBot - lnW - pedY)/k.maxV;
//								threatEnd = k.accT + (lnTop + lnW - pedY)/k.maxV;
								}}}
					else {								//ped walking down
						endGauntlet = lnBot - lnW;
						if (pedY < lnTop) {
							if (pedY < lnBot) {
								if (pedY < lnBot - lnW) {	//ped already out of danger (lane buffer irrelevant in bottom lane
									threatEnd = 0;}										//bc crossing = 3 once ped is clear)
								else {						//ped out of this lane, not yet clear
									threatBeg = 0;
									if (k.v[1] != 0) {
										threatEnd = (pedY - lnBot + lnW)/Math.abs(k.v[1]);}
									else {
										threatEnd = 1e12;}}}
//									threatEnd = (pedY - lnBot + lnW)/k.maxV;}}
							else {							//ped in this lane
								threatBeg = 0;
								if (k.v[1] != 0) {
									threatEnd = (pedY - lnBot + lnW)/Math.abs(k.v[1]);}
								else {
									threatEnd = 1e12;}}}
//								threatEnd = k.accT + (pedY - lnBot + lnW)/k.maxV;}}
						else {
							if (lane == 0) {
								if (dir == -1) {			//car in top lane, ped hasn't started to cross
									threatBeg = 0;	
									if (k.v[1] != 0) {
										threatEnd = 2*lnW/Math.abs(k.v[1]);}
									else {
										threatEnd = k.accT + 2*lnW/k.maxV;}}
//									threatEnd = k.accT + 2*lnW/k.maxV;}
								else {						//car in bottom lane, ped clear after just this lane
									if (k.v[1] != 0) {
										threatBeg	= (pedY - lnTop - lnW)/Math.abs(k.v[1]);  //one lane buffer
										threatEnd	= (pedY - lnBot - lnW/2)/Math.abs(k.v[1]);
										endGauntlet	= lnBot - lnW/2;}
									else {
										threatBeg	= (pedY - lnTop - lnW)/k.maxV;  //one lane buffer
										threatEnd	= 1e12;
										endGauntlet	= lnBot - lnW/2;}}}
									
//									threatBeg	= (pedY - lnTop - lnW)/k.maxV;  //one lane buffer
//									threatEnd	= (pedY - lnBot)/k.maxV;
//									endGauntlet	= lnBot;}}
							else {
								if (k.v[1] != 0) {
									threatBeg = (pedY - lnTop - lnW)/Math.abs(k.v[1]);
									threatEnd = k.accT + (pedY - lnBot + lnW)/Math.abs(k.v[1]);}
								else {
									threatBeg = (pedY - lnTop - lnW)/k.maxV;
									threatEnd = 1e12;}}}}
//								threatBeg = (pedY - lnTop - lnW)/k.maxV;  //one lane buffer
//								threatEnd = k.accT + (pedY - lnBot + lnW)/k.maxV;}}}
					if (threatBeg < 0) {
						threatBeg = 0;}			//correction for ped within current lane
					if (threatBeg == 0 && k.v[1] == 0) {
						threatEnd = 1e12;}
					
					//decide whether or not to yield for this ped
					if (percV != 0) {				//ttstop is undefined if car is already stopped
						if (realStopDist >= 0) {
							if (stopDist < 0) {
								int foo = 0;}
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
										thisDecel = -2*(percV*threatEnd - stopDist) / (threatEnd*threatEnd);
										if (thisDecel > 0) thisDecel = 0;}
									thisYing  = 0;}
								Yieldage thisYield = new Yieldage(k,thisDecel,endGauntlet,thisYing);
								//k.yielders.add(this);
								cYields.add(thisYield);}
							else {
								thisYing = -1;}}
						else {
							if (tClear < threatBeg) {
								threatEnd = 0;}							//driver recognizes ped is accepting rolling gap
							if (threatEnd > 0 && threatBeg == 0) {
								thisDecel = hardYield;
								thisYing  = 1;
								Yieldage thisYield = new Yieldage(k,thisDecel,endGauntlet,thisYing);
								//k.yielders.add(this);
								cYields.add(thisYield);}
							else {
								thisYing = -1;}}}
					else {
						if (age > 1 && threatEnd > 0) {
							thisDecel = 0;
							thisYing = 1;
							Yieldage thisYield = new Yieldage(k,thisDecel,endGauntlet,thisYing);
							//k.yielders.add(this);
							cYields.add(thisYield);}}}
				//update final acceleration value and yielding state
//				if (thisDecel < cYieldD) {
//					cYieldD = thisDecel;}
//				if (thisYing > outYing) {
//					outYing = thisYing;}
				}}
		
		delayedYields.put(stamp,cYields);
		int foo = 0;
		int wayBack = (int)Math.round(BRTs/UserPanel.tStep);
		if (UserPanel.BRT && stamp > wayBack + 3) {
			int thenKey = stamp - wayBack;
			if (delayedYields.size() > wayBack) {
				if (delayedYields.containsKey(thenKey)) {
					reactTo = delayedYields.get(thenKey);}
				else {
					int bar = 0;}}
			else {
				reactTo = cYields;}}
		else {
			reactTo = cYields;}
		
		for (Yieldage yg : reactTo) {
			if (yg.yState > -1) {
				if (!yg.yieldee.yielders.contains(this)) {
					yg.yieldee.yielders.add(this);}
				if (yg.calcAcc < cYieldD) {
					cYieldD = yg.calcAcc;}
				if (yg.yState > outYing) {
					outYing = yg.yState;}}}
		if (cYieldD < -UserPanel.emergDec) {
			cYieldD = -UserPanel.emergDec;}
		
		if (delayedYields.size() > wayBack + 10) {
			int lowKey = Collections.min(delayedYields.keySet());
			delayedYields.remove(lowKey);}
		
		double[] rv = new double[] {cYieldD,outYing};
		return rv;
	}
	
	/* Returns delayed value of inList */
	public double[] delayValue(List<double[]> inList, double inVal1, double inVal2) {
		double stamp, tStamp;
		stamp  = RoadBuilder.clock.getTickCount();
		tStamp = stamp*UserPanel.tStep;
		inList.add(new double[] {tStamp, inVal1, inVal2});
		double hiT = 0;
		double backMod = 0;
		double delayedT = 0;
		double backN = 0;
		double rv1 = 0;
		double rv2 = -1;
		int foo = 0;
		if (inList == shldBrakeStorage) {
			delayedT = tStamp - BRTs;
			backN = brt_tN;
			backMod = brtBeta;}
		else if (inList == storage) {
			delayedT = tStamp - delayTs;
			backN = tN;
			backMod = tBeta;}
		
		int storSize = inList.size();
		if (storSize > 3 && storSize > backN+1) { 
			while (hiT < delayedT) {
				hiT = inList.get(foo)[0];
				foo++;}
			double hiVal1 = inList.get(foo-1)[1];
			double hiVal2 = inList.get(foo-1)[2];
			if (Math.abs(hiT - delayedT) > 1e-14) {	//linear interpolation TODO: is there a better approx?
				double loVal1 = inList.get(foo-2)[1];
				double loVal2 = inList.get(foo-2)[2];
				rv1 = backMod*loVal1 + (1-backMod)*hiVal1;
				//rv2 = backMod*loVal2 + (1-backMod)*hiVal2;
				rv2 = Math.max(loVal2, hiVal2);}
			else {
				rv1 = hiVal1;
				rv2 = hiVal2;}
			if (storSize > backN + 10) {
				inList.remove(0);}}
		else {
			rv1 = inVal1;
			rv2 = inVal2;}
		return new double[] {tStamp,rv1,rv2};
	}
	
	/**
	 * store any conflicts with TTC < limit in userPanel
	 * @param p
	 * overloaded
	 */
	public void conflict() {
		double ttc	= 0;
		crossingP2	= new ArrayList<Ped>();
		if (realTtStopBar < confLim) {
			for (Ped i : RoadBuilder.flowSource.allPeds) {
				if (i.crossing == 2 && !i.curbed) {
					crossingP2.add(i);}}
			if (!crossingP2.isEmpty()) {
				for (Ped n : crossingP2) {
					conflict(n);}}}}
	public void conflict(Ped p) {
		double sinceBlocked = -1;
		double stamp = RoadBuilder.clock.getTickCount();
		if (blockedPeds.containsKey(p)) {
			sinceBlocked = stamp - blockedPeds.get(p);}
		double pedX = p.xLoc;
		double pedY = p.yLoc;
		double pedTlo = -1;		//time until ped at conflict point
		double pedThi = -1;		//time until ped leaves CP
		double ttc = -1;
		double lw = RoadBuilder.laneW;
		double worstV = Math.max(v, vNew);
		if (worstV != 0) {
			ttc = ((double)dir*(pedX - xLoc) - p.r)/worstV;}
//		if (ttc < 0 && ttc >= -length/worstV) {
//			int foo = 0;}
		if (ttc >= 0 && ttc <= confLim) {
			if (p.dir == 1 && pedY <= (lnTop)) {
			//if (p.dir == 1 && pedY <= (lnTop + lw)) {
			//if (p.dir == 1 && pedY <= (yLoc + carW/2)) {
				if (pedY >= (lnBot)) {
				//if (pedY >= (lnBot - lw)) {
				//if (pedY >= (yLoc - carW/2)) {
					pedTlo = 0;}
				else {
					pedTlo = 1000;
					if (p.v[1] != 0) {
						pedTlo = ((lnBot) - pedY)/p.v[1];}}
						//pedTlo = ((lnBot - lw) - pedY)/p.v[1];}}
						//pedTlo = ((yLoc - carW/2) - pedY)/p.v[1];}}		//TODO: include ped r and make ped calc 2D
				pedThi = 1000;
				if (p.v[1] != 0) {
					pedThi = ((lnTop) - pedY)/p.v[1];}}
					//pedThi = ((lnTop + lw) - pedY)/p.v[1];}}
					//pedThi = ((yLoc + carW/2) - pedY)/p.v[1];}}
			else if (p.dir == -1 && pedY >= (lnBot)) {
			//else if (p.dir == -1 && pedY >= (lnBot - lw)) {
			//else if (p.dir == -1 && pedY >= (yLoc - carW/2)) {
				//if (pedY <= (yLoc + carW/2)) {
				//if (pedY <= (lnTop + lw)) {
				if (pedY <= (lnTop)) {
					pedTlo = 0;}
				else {
					pedTlo = 1000;
					if (p.v[1] != 0) {
						pedTlo = -(pedY - (lnTop))/p.v[1];}}
						//pedTlo = -(pedY - (lnTop + lw))/p.v[1];}}
						//pedTlo = -(pedY - (yLoc + carW/2))/p.v[1];}}		//TODO: include ped r and make ped calc 2D
				pedThi = 1000;
				if (p.v[1] != 0) {
					pedThi = -(pedY - (lnBot))/p.v[1];}}
					//pedThi = -(pedY - (lnBot - lw))/p.v[1];}}
					//pedThi = -(pedY - (yLoc - carW/2))/p.v[1];}}
			if (pedTlo != -1) {
				if (ttc >= pedTlo && ttc <= pedThi) {
					int init = 1;
					int dup = 0;
					int hasDup = 0;
					double range = (double)dir*(pedX - xLoc);
					Conflict thisConf = new Conflict(this,p,ttc,range,init,hasDup,sinceBlocked);
					Conflict toAdd = null;
					Conflict toRem = null;
					for (Conflict c : RoadBuilder.flowSource.allConf) {
						if (c.ped == p && c.car != this) {
							Turtle otherCar = c.car;	//make sure not counting cars behind as extra conflicts
							if (otherCar.dir == this.dir && otherCar.lane == this.lane) {
								if (c.TTC < ttc) {
									int foo = 0;}}}
						if (c.car == this && c.ped == p) {
							dup = 1;			//make sure no duplicates added unless lower TTC than first
							if (c.init == 1) {
								if (c.hasDup == 0) {
									if (ttc < c.TTC) {
										c.hasDup = 1;
										thisConf.init = 0;
										toAdd = thisConf;}}
								else {
									for (Conflict c1 : RoadBuilder.flowSource.allConf) {
										if (c1.car == this && c1.ped == p) {
											if (c1 != c) {
												if (ttc < c1.TTC) {
													thisConf.init = 0;
													toAdd = thisConf;
													toRem = c1;}}}}}}}}
					if (dup == 0) {
						RoadBuilder.flowSource.allConf.add(thisConf);}
					else {
						if (toAdd != null) {
							RoadBuilder.flowSource.allConf.add(toAdd);}
						if (toRem != null) {
							RoadBuilder.flowSource.allConf.remove(toRem);}}}}}
		
		//check for impending crashes
		double crashPedTlo = -1e15;
		double crashPedThi = -1;
		if (worstV != 0 && ttc >= -length/worstV && ttc <= 1) {
			if (p.dir == 1 && pedY <= (yLoc + carW/2 + p.r)) {
				if (pedY >= (yLoc - carW/2 - p.r)) {
					crashPedTlo = -1e5;}
				else {
					crashPedTlo = 1000;
					if (p.v[1] != 0) {
						crashPedTlo = ((yLoc - carW/2 - p.r) - pedY)/p.v[1];}}		//TODO: make ped calc 2D
				crashPedThi = 1000;
				if (p.v[1] != 0) {
					crashPedThi = ((yLoc + carW/2 + p.r) - pedY)/p.v[1];}}
			else if (p.dir == -1 && pedY >= (yLoc - carW/2 - p.r)) {
				if (pedY <= (yLoc + carW/2 + p.r)) {
					crashPedTlo = -1e5;}
				else {
					crashPedTlo = 1000;
					if (p.v[1] != 0) {
						crashPedTlo = -(pedY - (yLoc + carW/2 + p.r))/p.v[1];}}		//TODO: make ped calc 2D
				crashPedThi = 1000;
				if (p.v[1] != 0) {
					crashPedThi = -(pedY - (yLoc - carW/2 - p.r))/p.v[1];}}
			if (crashPedTlo != -1e15) {
//				if (ttc < 0 && crashPedTlo == -1e5) {
//					int foo = 0;}
				if (ttc >= crashPedTlo && ttc <= crashPedThi) {
					int init = 1;
					int dup = 0;
					int hasDup = 0;
					Conflict thisCrash = new Conflict(this,p,0,0,init,hasDup,sinceBlocked);
					Conflict toAdd = null;
					Conflict toRem = null;
					
					for (Conflict c : RoadBuilder.flowSource.allConf) {
						if (c.car == this && c.ped == p) {
							dup = 1;			//make sure no duplicates added unless lower TTC than first
							if (c.init == 1) {
								if (c.hasDup == 0) {
									if (ttc < c.TTC) {
										c.hasDup = 1;
										thisCrash.init = 0;
										toAdd = thisCrash;}}
								else {
									for (Conflict c1 : RoadBuilder.flowSource.allConf) {
										if (c1.car == this && c1.ped == p) {
											if (c1 != c) {
												if (ttc < c1.TTC) {
													thisCrash.init = 0;
													toAdd = thisCrash;
													toRem = c1;
													break;}}}}}
								break;}}}
					if (dup == 0) {
						RoadBuilder.flowSource.allConf.add(thisCrash);}
					else {
						if (toAdd != null) {
							RoadBuilder.flowSource.allConf.add(toAdd);}
						if (toRem != null) {
							RoadBuilder.flowSource.allConf.remove(toRem);}}}}}
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
	
	public Turtle(Turtle src) {
		v = src.v;
		newAcc = src.newAcc;
		vNew = src.vNew;
		acc = src.acc;
		xLoc = src.xLoc;
		yLoc = src.yLoc;
	}
	public Turtle(ContinuousSpace<Object> contextSpace, int whichLane, int whichDir,
				  boolean conn, boolean auto) {
		space	= contextSpace;
		lane	= whichLane;
		dir		= whichDir;
		connected	= conn;
		autonomous	= auto;
		double maxa0 = rndIDM.nextGaussian()*UserPanel.maxaShape + UserPanel.maxaScale;
		maxa	= Math.exp(maxa0);
		if (maxa > RoadBuilder.panel.maxMaxA) maxa = RoadBuilder.panel.maxMaxA;
		if (maxa < RoadBuilder.panel.minMaxA) maxa = RoadBuilder.panel.minMaxA;
		double mina0 = rndIDM.nextGaussian()*UserPanel.minaShape + UserPanel.minaScale;
		mina	= Math.exp(mina0);
		if (mina > RoadBuilder.panel.maxMinA) mina = RoadBuilder.panel.maxMinA;
		if (mina < RoadBuilder.panel.minMinA) mina = RoadBuilder.panel.minMinA;
		maxv	= rndIDM.nextGaussian()*(RoadBuilder.panel.sLimit_sd)+(RoadBuilder.panel.sLimitMu);
		if (maxv > RoadBuilder.panel.maxMaxV) maxv = RoadBuilder.panel.maxMaxV;
		if (maxv < RoadBuilder.panel.minMaxV) maxv = RoadBuilder.panel.minMaxV;
		tGap	= rndIDM.nextGaussian()*(UserPanel.tGap_sd)+UserPanel.tGap;
		if (tGap > RoadBuilder.panel.maxHeadT) tGap = RoadBuilder.panel.maxHeadT;
		if (tGap < RoadBuilder.panel.minHeadT) tGap = RoadBuilder.panel.minHeadT;
		double jamHead0 = rndIDM.nextGaussian()*UserPanel.jamHeadShape + UserPanel.jamHeadScale;
		jamHead = Math.exp(jamHead0);
		if (jamHead > RoadBuilder.panel.maxJamHead) jamHead = RoadBuilder.panel.maxJamHead;
		if (jamHead < RoadBuilder.panel.minJamHead) jamHead = RoadBuilder.panel.minJamHead;
		//jamHead	= rnd.nextGaussian()*(UserPanel.jamHeadShape)+UserPanel.jamHeadScale;
		length	= UserPanel.carLength;
//		if (!autonomous) {
			percLimit = RoadBuilder.panel.hPercLim;
			BRTs = calcBRT() + 0.35;  // + 0.15 mvmt time (Lister 1950) + 0.2 device response time (Grover 2008)
			double delayT0 = rndADRT.nextGaussian() * 1.193759285934727 - 1.60692043370482;
			delayTs	= Math.exp(delayT0) + 0.25;
			if (delayTs > 2.5) {
				delayTs = 2.5;}
//			}
//		else {
//			percLimit = RoadBuilder.panel.aPercLim;
//			BRTs = 0.51;  //seconds (source Grover 2008)
//			delayTs = 0.4;}
		tN		= Math.floor(delayTs/UserPanel.tStep);
		tBeta	= (delayTs/UserPanel.tStep) - tN;
		brt_tN	= Math.floor(BRTs/UserPanel.tStep);
		brtBeta	= (BRTs/UserPanel.tStep) - brt_tN;
		v		= maxv * (1 - .3*rnd.nextDouble());
		decelT	= v/UserPanel.emergDec;
		wS		= rnd.nextGaussian();
		etaS	= rnd.nextGaussian();
		wV		= rnd.nextGaussian();
		etaV	= rnd.nextGaussian();
		sigR 	= 0.01*UserPanel.tStep; //standard deviation of relative approach rate
		wPed	= rnd.nextGaussian();
		etaPed	= rnd.nextGaussian();
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
		zIIDM 		= 1000;
		accCAH		= -1000;
		interDlam	= UserPanel.interDlam;
		nMaxDecel	= 0;
		blockedPeds = new HashMap<Ped, Double>();
	}
	
	/* Fits driver's distance from stopBar to lognormal distribution in model units */
	public double calcStopBar(int dir) {
		double dist0 = rndS.nextGaussian()*UserPanel.SsigHat + UserPanel.SmuHat;
		double stopDistance0 = Math.exp(dist0);
		if (stopDistance0 > RoadBuilder.panel.stopBarDistance + 1/RoadBuilder.spaceScale) {
			stopDistance0 = RoadBuilder.panel.stopBarDistance + 1/RoadBuilder.spaceScale;}
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
		double time;
		Yieldage(Ped yieldee, double calcAcc, double endThreat, int yState) {
			this.yieldee = yieldee;
			this.calcAcc = calcAcc;
			this.endThreat = endThreat;
			this.yState = yState;
			this.time = RoadBuilder.clock.getTickCount();}
		Yieldage(Yieldage src) {
			this.yieldee = src.yieldee;
			this.calcAcc = src.calcAcc;
			this.endThreat = src.endThreat;
			this.yState = src.yState;
			this.time = RoadBuilder.clock.getTickCount();
		}
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
		int dirP, dirC, lane, yingVal, nMaxD, init, hasDup;
		double TTC, range, yDec, vel, timeD, sinceD, tick, yDist, minGap, brt, sinceBlocked;
		boolean conn, auto;
		ArrayList<double[]> pedVid;
		ArrayList<Video> video;
		double xWalkx = RoadBuilder.xWalkx;
		double spaceScale = UserPanel.spaceScale;
//		Conflict(Turtle car, Ped ped, double ttc, double range, int yieldState, double yieldD, int nMaxD,
//				double timeSinceD, double timeD, int init, int hasDup, boolean conn, boolean auto) {
		Conflict(Turtle car, Ped ped, double ttc, double range, int init, int hasDup, double _sinceBlocked) {
			this.ped	= ped;
			this.car	= car;
			this.dirP	= ped.dir;
			this.dirC	= car.dir;
			this.lane	= car.lane;
			this.TTC	= ttc;
			this.range	= range;
			this.tick	= RoadBuilder.clock.getTickCount();
			this.vel	= car.v;
			this.yingVal= car.ying;
			this.yDec	= car.yieldDec;
			this.nMaxD	= car.nMaxDecel;
			this.sinceD	= car.timeSinceD;
			this.timeD	= car.timeD;
			this.conn	= car.connected;
			this.auto	= car.autonomous;
			this.init	= init;
			this.hasDup	= hasDup;
			this.minGap = ped.critGap;
			this.brt	= car.BRTs;
			this.sinceBlocked = _sinceBlocked;
			double yDistCenters = ped.yLoc - car.yLoc;
			double above = Math.signum(yDistCenters);
			if (above == 0) this.yDist = 0;
			else {
				double combinedWidth = ped.r + car.carW/2;
				if (Math.abs(yDistCenters) <= combinedWidth) {
					this.yDist = 0;}
				else {
					this.yDist = yDistCenters - above*(combinedWidth);}}
//			this.yingVal= yieldState;
//			this.yDec	= yieldD;
//			this.nMaxD	= nMaxD;
//			this.sinceD	= timeSinceD;
//			this.timeD	= timeD;
//			this.conn	= conn;
//			this.auto	= auto;
//			this.init	= init;
//			this.hasDup	= hasDup;
		}
//			this.video	= new ArrayList<Video>();
//			this.pedVid = new ArrayList<double[]>();
//			double[] thisPedVid = new double[2];
//			ped.myLoc.toDoubleArray(thisPedVid);
//			this.pedVid.add(thisPedVid);
//			Iterable<Object> snapShot = space.getObjects();
//			Iterator<Object> iterator = snapShot.iterator();
//			for (Turtle t : Scheduler.allCars) { 
//			while (iterator.hasNext()) {
//				Object element0 = iterator.next();
//				Agent element = (Agent)element0;
//				if (element.isCar()) {
//					Turtle thisCar = (Turtle)element;
//				if (t.myLoc != null && t.dir == car.dir) {
//					double[] location = new double[2];
//					t.myLoc.toDoubleArray(location);
//					double thirty = 30/spaceScale; //30m should be enough to see all relevant cars
//					if (location[0] >= xWalkx - thirty && location[0] <= xWalkx + thirty) {
//						Video thisVideo = new Video(t,location);
//						this.video.add(thisVideo);}}}
	}
	
	public class Video {
		Turtle car;
		ArrayList<double[]> locs;
		Video(Turtle agent, double[] loc) {
			this.car = agent;
			this.locs = new ArrayList<double[]>();
			this.locs.add(loc);}
	}
	
	public class Crash {
		Ped ped;
		Turtle car;
		int dirP, dirC, lane, yielding, nMaxDec;
		double v, dec, sinceDist, timeDist, tick;
		boolean conn, auto;
		Crash(Turtle car, Ped ped) {
			this.ped = ped;
			this.car = car;
			this.dirP = ped.dir;
			this.dirC = car.dir;
			this.lane = car.lane;
			this.v = car.v;
			this.yielding = car.ying;
			this.dec = car.yieldDec;
			this.nMaxDec = car.nMaxDecel;
			this.tick = RoadBuilder.clock.getTickCount();
			this.sinceDist = car.timeSinceD;
			this.timeDist = car.timeD;
			this.conn = car.connected;
			this.auto = car.autonomous;}
	}
	
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
		return acc*UserPanel.spaceScale/(UserPanel.tStep*UserPanel.tStep);}
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