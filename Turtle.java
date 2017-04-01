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
	private ArrayList<Turtle> sameDir, ahead, leaders, behind, followers;
	private ArrayList<Ped> crossingP, crossingP1, crossingP2, waitingP;
	private ArrayList<ViewAngle> obstructers, obstructees;
	private ArrayList<Yieldage> yieldage = new ArrayList<Yieldage>();
	private List<double[]> storage = new ArrayList<double[]>();
	private Random  rnd  = new Random();	//initiates random number generator for vehicle properties
	private Random  rndD = new Random();	//ditto for distraction
	private boolean	distracted = false;
	private double	timeD, durD, timeSinceD, interD, interDlam;		//distraction
	private double	delayTs, tN, tBeta;							//delay
	private double	tGap, jamHead, maxv, mina, maxa, newAcc;	//car-following
	private double	wS, etaS, wV, etaV, sigR;					//errors
	private double	confLim, stopBar, ttstopBar, lnTop, lnBot, hardYield;		//yielding
	private double	carW = UserPanel.carWidth;
	private double  deltaIDM = 4;
	private int		age;
	public  NdPoint	myLoc;
	public  Turtle	leader, follower;
	public	boolean connected, autonomous;
	public  double	v, vNew, acc, xLoc, yLoc, length, tail, driverX, driverY, decelT;
	public  int		lane;	//  0 = outer lane, 1 = inner lane
	public	int		dir;	//  1 = going right, -1 = going left
	public	int		ying, oldYing;	// -1 = none, 0 = soft, 1 = hard
	
	/**
	* Calculates driver acceleration (if not distracted)
	* Sets value of acc & vNew
	*/
	public void calc() {
		myLoc	= space.getLocation(this);
		xLoc	= myLoc.getX();
		newAcc  = 0;
		if (interD != 0 && timeSinceD >= interD) {
			distracted = true;
			timeSinceD = 0;
			interD = 0;}
		if (durD != 0 && timeD >= durD) {
			distracted = false;
			timeD = 0;
			durD = 0;}
		if (autonomous == false && distracted == true) {
			if (timeD == 0) {
				double durD0 = rndD.nextGaussian()*UserPanel.DsigHat + UserPanel.DmuHat;
				durD = Math.exp(durD0);}
			newAcc = acc;
			timeD += 1;
			double xwalkD	= RoadBuilder.xWalkx - xLoc;
			double threat	= Math.signum(dir*xwalkD);
			if (threat == 1 && v != 0) {
				conflict();}}
		else {
			if (timeSinceD == 0) {
				double interD0 = (rndD.nextDouble() + Double.MIN_VALUE)*interDlam; //padded to avoid -inf
				interD = -Math.log(interD0/interDlam)/interDlam;}
			timeSinceD += 1;
			newAcc = accel(myLoc, lane, dir);}
		//delayed reaction: implements acc calculated and stored delayT ago
		if (UserPanel.delayTs > 0 && autonomous == false) { //TODO: probably give non-zero value for automated
			double stamp, tStamp, delayedT, hiT;
			stamp  = RoadBuilder.clock.getTickCount();
			tStamp = stamp*UserPanel.tStep;
			delayTs = UserPanel.delayTs;
			tN		= Math.floor(delayTs/UserPanel.tStep);
			tBeta	= (delayTs/UserPanel.tStep) - tN;
			//TODO: limit size of storage to limit memory use
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
				if (hiT != delayedT){	//linear interpolation TODO: is there a better approx?
					double loAcc = storage.get(foo-2)[1];
					acc = tBeta*loAcc + (1-tBeta)*hiAcc;}
				else acc = hiAcc;
				if (storSize > 5*tN) {		//TODO: make this less arbitrary
					storage.remove(storSize - 1);}}
			else acc = newAcc;}
		else acc = newAcc;
		age++;
		
		vNew = v + acc;
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
				space.moveByDisplacement(this,vNew,0);
				myLoc = space.getLocation(this);
				xLoc = myLoc.getX();}}
		else {
			if (xLoc - vNew <= 0) {
				Scheduler.killListC.add(this);
				this.storage.clear();}
			else if (vNew != 0) {
				space.moveByDisplacement(this,-vNew,0);
				myLoc = space.getLocation(this);
				xLoc = myLoc.getX();}}
		v = vNew;
		decelT = v/UserPanel.emergDec;
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
		double a, head, setSpeed, vDiff, safeHead, aFree;
		confLim	= UserPanel.confLimS/UserPanel.tStep;
		sameDir	= new ArrayList<Turtle>();
		ahead	= new ArrayList<Turtle>();
		leaders	= new ArrayList<Turtle>();
		head	= RoadBuilder.roadL;
		//TODO: limit accel to physically possible values
		
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
				if (myDir == 1) {					if (m.xLoc > xLoc) {						ahead.add(m);}
					if (follower == null) {
						if (m.xLoc < xLoc) {
							behind.add(m);}}}
				else {
					if (m.xLoc < xLoc) {
						ahead.add(m);}
					if (follower == null) {
						if (m.xLoc > xLoc) {
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
				if(Math.abs(o.xLoc - xLoc) < head) {
					head = Math.abs(o.xLoc - xLoc) - o.length; //TODO: change visualization to move icons to center of car
					if (head < 0) {
						head = 1e-5;}
					leader = o;}}}
		if (follower == null) {
			if (!followers.isEmpty()) {
				for (Turtle p : followers) {
					if((Math.abs(p.xLoc - xLoc) - length) < tail) {
						tail = Math.abs(p.xLoc - xLoc) - length;
						follower = p;}}}}
		else {
			tail = Math.abs(follower.xLoc - xLoc) - length;}
		
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
			double z = safeHead / head;
//			if (UserPanel.IIDM == true) {
				if (v < maxv) {
					aFree = maxa*(1-Math.pow(v/maxv,deltaIDM));
					if (z >= 1) {
						a = maxa*(1-z*z);}
					else {
						a = aFree*(1-Math.pow(z, 2*maxa/aFree));}}
				else {
					aFree = -mina*(1-Math.pow(maxv/v, maxa*deltaIDM/mina));
					if (z >= 1) {
						a = aFree + maxa*(1-z*z);}
					else {
						a = aFree;}}
//				}
//			else {
//				a = maxa*(1 - Math.pow(v/maxv,deltaIDM) - Math.pow(safeHead/head,2));}
		}
		else {a = maxa*(1 - Math.pow(v/maxv,deltaIDM));}

		//Calculate yielding acceleration
		double stopD	= stopBar - xLoc;
		double stopDab	= dir*stopD;
		double xwalkD	= RoadBuilder.xWalkx - xLoc;
		double threat	= Math.signum(dir*xwalkD);
		double aYield	= 0;
		if (threat == 1) {
			aYield = yield(stopDab,xwalkD);
			if (aYield < a) {
				a = aYield;}}
		if (age > 20 && a < -UserPanel.emergDec) {
			a = -UserPanel.emergDec;}
		if(a < -UserPanel.emergDec) {
			a = -UserPanel.emergDec;}
		return a;
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
		double	yieldDec = 1;		//dummy value
		if (stopDist >=0) {
			if (v != 0) {
				tHardYield = 2*stopDist/v;
				ttstopBar  = stopDist/v;}
			else {
				ttstopBar  = 1e16;		//arbitrarily large
				tHardYield = 1e16;}
			hardYield	= -v*v/(2*stopDist);}
		else {							//driver has already passed stopbar (probably distracted)
			 ttstopBar = -1;
			tHardYield = decelT;
			 hardYield = -UserPanel.emergDec;
			 if (v != 0) {
				 tCrash = conDist/v;}}
		double tClear = (conDist + length)/v;
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
		}
		
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
					double newDecel;
					if (k.dir == 1 && clearY > pedY) {
						if (v != 0) {
							threatBeg = 
							threatEnd = k.accT*(1-k.v[1]/k.maxV) + (clearY - pedY)/k.maxV;
							if (threatEnd > tHardYield) {
								newDecel = hardYield;
								thisYing = 1;
								if (stopDist < 0) {
									crash(k);}}
							else {			//soft yield
								if (stopDist >= 0) {
									newDecel = -2*(v*threatEnd - stopDist) / (threatEnd*threatEnd);
									thisYing = 0;}
								else {
									newDecel = hardYield;
									thisYing = 1;
									crash(k);}}
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
								thisYing = 1;
								if (stopDist < 0) {
									crash(k);}}
							else {			//soft yield
								if (stopDist >= 0) {
									newDecel = -2*(v*threatEnd - stopDist) / (threatEnd*threatEnd);
									thisYing = 0;}
								else {
									newDecel = hardYield;
									crash(k);}}
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
					if (v != 0) {				//ttstop is undefined if car is already stopped
						if (stopDist >= 0) {
							if (tClear < threatBeg) {	//can probably lose this first if
								//if (ttstopBar <= decelT) {
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
									thisDecel = -2*(v*threatEnd - stopDist) / (threatEnd*threatEnd);
									thisYing  = 0;}
								Yieldage thisYield = new Yieldage(k,thisDecel,endGauntlet,thisYing);
								k.yielders.add(this);
								yieldage.add(thisYield);}
							else {
								thisYing = -1;}}
						else {
							if (tClear < threatBeg) {
								threatEnd = 0;}							//driver recognizes ped is accepting rolling gap
							if (threatEnd > 0) {
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
		double ttc = Math.abs(pedX - xLoc)/v;
		
		if (p.dir == 1 && pedY <= (yLoc + carW/2)) {
			if (pedY >= (yLoc - carW/2)) {
				pedTlo = 0;}
			else {
				pedTlo = ((yLoc - carW/2) - pedY)/p.v[1];}		//TODO: include ped r and make ped calc 2D
			pedThi = ((yLoc + carW/2) - pedY)/p.v[1];}
		else if (p.dir == -1 && pedY >= (yLoc - carW/2)) {
			if (pedY <= (yLoc + carW/2)) {
				pedTlo = 0;}
			else {
				pedTlo = -(pedY - (yLoc + carW/2))/p.v[1];}		//TODO: include ped r and make ped calc 2D
			pedThi = -(pedY - (yLoc - carW/2))/p.v[1];}
		if (pedTlo != -1) {
			if (ttc >= pedTlo && ttc <= pedThi) {
				if (ttc <= confLim) {
					int init = 1;
					int dup = 0;
					int hasDup = 0;
					Conflict thisConf = new Conflict(this,p,ttc,oldYing,hardYield,init,hasDup);
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
						Scheduler.allConf.add(thisConf);}}}}
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
					double pedX = n.xLoc;
					double pedY	= n.yLoc;
					double pedTlo = -1;		//time until ped at conflict point
					double pedThi = -1;		//time until ped leaves CP
					ttc	= Math.abs(pedX - xLoc)/v;
					
					if (n.dir == 1 && pedY <= (yLoc + carW/2)) {
						if (pedY >= (yLoc - carW/2)) {
							pedTlo = 0;}
						else {
							pedTlo = ((yLoc - carW/2) - n.yLoc)/n.v[1];}		//TODO: include ped r and make ped calc 2D
						pedThi = ((yLoc + carW/2) - pedY)/n.v[1];}
					else if (n.dir == -1 && pedY >= (yLoc - carW/2)) {
						if (pedY <= (yLoc + carW/2)) {
							pedTlo = 0;}
						else {
							pedTlo = -(n.yLoc - (yLoc + carW/2))/n.v[1];}		//TODO: include ped r and make ped calc 2D
						pedThi = -(pedY - (yLoc - carW/2))/n.v[1];}
					if (pedTlo != -1) {
						if (ttc >= pedTlo && ttc <= pedThi) {
							if (ttc < confLim) {
								int init = 1;
								int dup = 0;
								int hasDup = 0;
								Conflict thisConf = new Conflict(this,n,ttc,oldYing,hardYield,init,hasDup);
								Conflict toAdd = null;
								Conflict toRem = null;
								for (Conflict c : Scheduler.allConf) {
									if (c.car == this && c.ped == n) {
										dup = 1;			//make sure no duplicates added unless lower TTC than first
										if (c.init == 1) {
											if (c.hasDup == 0) {
												if (ttc < c.TTC) {
													c.hasDup = 1;
													thisConf.init = 0;
													toAdd = thisConf;}}
											else {
												for (Conflict c1 : Scheduler.allConf) {
													if (c1.car == this && c1.ped == n) {
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
									Scheduler.allConf.add(thisConf);}}}}}}}
	}
	
	public void crash(Ped p) {
		Crash newCrash = new Crash(this,p,1,0);
		Scheduler.crashes.add(newCrash);
	}

	/**
	 * Creates vehicle agents and initializes values
	 * Called by scheduler in Agent.java
	 * @param contextSpace
	 * @param contextGrid
	 */
	//TODO: add similar code to Ped.java to vary ped parameters
//	public Turtle(ContinuousSpace<Object> contextSpace, Grid<Object> contextGrid) {
	public Turtle(ContinuousSpace<Object> contextSpace, int whichLane, int whichDir,
			boolean conn, boolean auto) {
		space	= contextSpace;
//		grid	= contextGrid;
		lane	= whichLane;
		dir		= whichDir;
		//store parameters with heterogeneity (currently s.dev abitrarily = 8% of mean)
		//TODO: get theory for these numbers
		maxa	= rnd.nextGaussian()*(UserPanel.maxa*.08)+UserPanel.maxa;
		mina	= rnd.nextGaussian()*(UserPanel.mina*.08)+UserPanel.mina;
		maxv	= rnd.nextGaussian()*(.08*UserPanel.sLimit)+(UserPanel.sLimit);
		tGap	= rnd.nextGaussian()*(UserPanel.tGap*.08)+UserPanel.tGap;
		jamHead	= rnd.nextGaussian()*(UserPanel.jamHead*.08)+UserPanel.jamHead;
		length	= UserPanel.carLength;
		delayTs	= UserPanel.delayTs;
		tN		= Math.floor(delayTs/UserPanel.tStep);
		tBeta	= (delayTs/UserPanel.tStep) - tN;	
		v		= maxv * (1 - .3*rnd.nextDouble());
		decelT	= v/UserPanel.emergDec;
		wS		= etaS = rnd.nextGaussian();
		wV		= rnd.nextGaussian();
		etaV	= rnd.nextGaussian();
		sigR 	= 0.01*UserPanel.tStep; //standard deviation of relative approach rate
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
		lnTop	= lnBot + RoadBuilder.laneW;
		confLim	= UserPanel.confLimS/UserPanel.tStep;
		age		= 0;
		ying	= -1;
		timeD		= 0;
		durD		= 0;
		timeSinceD	= 0;
		interD		= 0;
		interDlam	= UserPanel.interDlam;
		connected	= conn;
		autonomous	= auto;
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
		double TTC, yDec, vel;
		Conflict(Turtle car, Ped ped, double ttc, int yieldState, double yieldDec, int init, int hasDup) {
			this.ped	= ped;
			this.car	= car;
			this.dirP	= ped.dir;
			this.dirC	= car.dir;
			this.lane	= car.lane;
			this.TTC	= ttc;
			this.ying	= yieldState;
			this.vel	= car.v;
			this.yDec	= yieldDec;
			this.init	= init;
			this.hasDup	= hasDup;}
	}
	
	public class Crash {
		Ped ped;
		Turtle car;
		int dirP, dirC, lane, init, hasDup;
		double v, curDec, sinceDist;
		boolean distracted;
		Crash(Turtle car, Ped ped, int init, int hasDup) {
			this.ped	= ped;
			this.car	= car;
			this.dirP	= ped.dir;
			this.dirC	= car.dir;
			this.lane	= car.lane;
			this.v		= car.v;
			this.init	= init;
			this.hasDup = hasDup;}
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
}