package driving1;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.apache.commons.math3.util.FastMath;

import repast.simphony.parameter.Parameter;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;

/**
 * Pedestrian agent class
 * @author Darryl Michaud
 */
public class Ped extends Agent{
	private boolean debug = false;
	private	ContinuousSpace<Object> space;
//	private	Grid<Object> grid;
	private List<Double> forcesX, forcesY; 
	private	NdPoint endPt;
	private Random rnd = new Random();
	//TODO: make sure there's a separate Wiener process for every single observation (think through which are related)
	public boolean curbed, boxed;
	private int age;
	private	double endPtDist, endPtTheta;
	private double side	= RoadBuilder.sidewalk;
	private double wS1, wSn1, wV1, wVn1, etaS, etaV, sigR;	//errors
	private double m, horiz, A, B, k;  					//interactive force constants (accT is also)
	static double xWalkHalfWidth = RoadBuilder.panel.xWalkWidth/2;
	private double boxL = RoadBuilder.xWalkx - xWalkHalfWidth;
	private double boxR = RoadBuilder.xWalkx + xWalkHalfWidth;
		//2 here is more or less arbitrary, but has to match Turtle.stopBar variable
	public ArrayList<Turtle> yielders;
	public Turtle nearest0, nearest1, nearest2, nearest3;
	public double pGap0, pGap1, pGap2, pGap3;
	public NdPoint myLoc;
	public double[] v, dv, newV;
	public double xTime, accT, maxV, xLoc, yLoc, whichSide, r, critGap, tickAtCrossDecision;
	public int dir;			// dir = 1 walks up, -1 walks down
	public int crossing;	// 0=not yet, 1=waiting, 2=yes, 3=done
	public int front;		// 1=xing in front of car, -1=behind; for testing
	public ArrayList<Turtle> blockedCars;
	public Map<Integer,Double> dictThtBegAtDecision, dictTTcolAtDecision, dictRealTTcolAtDecision, 
			dictTTclearAtDecision, dictRealTTClearAtDecision, dictTailTAtDecision, dictRealTailTAtDecision;
	public Map<Integer,Turtle> dictNearestTurtleAtDecision = new HashMap<Integer,Turtle>();
	public Map<Integer,Turtle> dictFollowingTurtleAtDecision = new HashMap<Integer,Turtle>();
	public Map<Integer,ArrayList<Double>> dictDistMSinceDecision;
	public Map<Integer,ArrayList<Double>> dictSpeedsSinceDecision;
	public Map<Integer,ArrayList<Double>> dictAccelsSinceDecision;
	public Map<Integer,ArrayList<Double>> dictFollowingCarSpeedsSinceDecision;
	
//	double realXDist = Math.sqrt(dist * dist - yDist*yDist);
//	if (approachV != 0) realTTCol = realXDist / t.v;
//	else realTTCol = 1e5;
//	thtBegAtDecision.put(ln, threatBeg); 
//	ttcolAtDecision.put(ln, TTCol);
//	realTTcolAtDecision.put(ln, realTTCol);
	
	/**
	 * Calculates ped acceleration
	 * Sets newV, changes value of crossing and curbed in some instances
	 */
	public void calc() {
		myLoc = space.getLocation(this);
		xLoc  = myLoc.getX();
		yLoc  = myLoc.getY();
		dv    = accel(myLoc,dir);
		newV  = sumV(v,dv);
		if (age > 1) {
			if (xLoc + r + newV[0] > boxR || xLoc - r + newV[0] < boxL){
				boxed = true;}
			else {boxed = false;}}
		else {boxed = false;}
//		newV  = limitV(newV);
		
		Observation currentScene = null;
		ArrayList<Turtle> approaching  = new ArrayList<Turtle>();
		
		if (crossing < 2 || curbed) {	
			for (Turtle p : RoadBuilder.flowSource.allCars) {
				double thisGap = xLoc - (p.xLoc - (double)p.dir*p.length);
				int threat = p.dir * (int)Math.signum(thisGap);
				if (threat == 1) {
					approaching.add(p);}}
			
			if (!approaching.isEmpty()) {
				currentScene = LookForApproachingCars(approaching);}}
		
		if (currentScene != null) {
			if (!RoadBuilder.panel.inclObstruct) {
				nearest0 = currentScene.nearest0;
				nearest1 = currentScene.nearest1;
				nearest2 = currentScene.nearest2;
				nearest3 = currentScene.nearest3;
				blockedCars = new ArrayList<Turtle>();}
			else {			
				blockedCars = new ArrayList<Turtle>(currentScene.blockedCars);
				if (nearest0 == null) {
					nearest0 = currentScene.nearest0;}
				else {
					if (Math.abs(currentScene.gap0) < Math.abs(pGap0) ) {
						nearest0 = currentScene.nearest0;}
					else {
						if (!approaching.contains(nearest0)) {
							if (nearest0.follower != null) {
								Turtle back1 = nearest0.follower;
								if (!currentScene.blockedCars.contains(back1)) {
									nearest0 = back1;}
								else {
									if (back1.follower != null) {
										if (!currentScene.blockedCars.contains(back1.follower)) {
												nearest0 = back1.follower;}
										else {
											nearest0 = null;}}
									else {
										nearest0 = null;}}}
							else {
								nearest0 = null;}}}}
				if (nearest1 == null) {
					nearest1 = currentScene.nearest1;}
				else {
					if (Math.abs(currentScene.gap1) < Math.abs(pGap1) ) {
						nearest1 = currentScene.nearest1;}
					else {
						if (!approaching.contains(nearest1)) {
							if (nearest1.follower != null) {
								Turtle back1 = nearest1.follower;
								if (!currentScene.blockedCars.contains(back1)) {
									nearest1 = back1;}
								else {
									if (back1.follower != null) {
										if (!currentScene.blockedCars.contains(back1.follower)) {
												nearest1 = back1.follower;}
										else {
											nearest1 = null;}}
									else {
										nearest1 = null;}}}
							else {
								nearest1 = null;}}}}
				if (nearest2 == null) {
					nearest2 = currentScene.nearest2;}
				else {
					if (Math.abs(currentScene.gap2) < Math.abs(pGap2) ) {
						nearest2 = currentScene.nearest2;}
					else {
						if (!approaching.contains(nearest2)) {
							if (nearest2.follower != null) {
								Turtle back1 = nearest2.follower;
								if (!currentScene.blockedCars.contains(back1)) {
									nearest2 = back1;}
								else {
									if (back1.follower != null) {
										if (!currentScene.blockedCars.contains(back1.follower)) {
												nearest2 = back1.follower;}
										else {
											nearest2 = null;}}
									else {
										nearest2 = null;}}}
							else {
								nearest2 = null;}}}}
				if (nearest3 == null) {
					nearest3 = currentScene.nearest3;}
				else {
					if (Math.abs(currentScene.gap3) < Math.abs(pGap3) ) {
						nearest3 = currentScene.nearest3;}
					else {
						if (!approaching.contains(nearest3)) {
							if (nearest3.follower != null) {
								Turtle back1 = nearest3.follower;
								if (!currentScene.blockedCars.contains(back1)) {
									nearest3 = back1;}
								else {
									if (back1.follower != null) {
										if (!currentScene.blockedCars.contains(back1.follower)) {
												nearest3 = back1.follower;}
										else {
											nearest3 = null;}}
									else {
										nearest3 = null;}}}
							else {
								nearest3 = null;}}}}}}
		
		switch (crossing) {
		case 0: if (dir == 1) {
					if (curbed == true) {
						crossing = 1;}
					if (yLoc + newV[1] >= side - r) {
						curbed = true;
						dv = yield(approaching,currentScene);
						newV = sumV(v,dv);}}
				else {
					if (curbed == true) {
						crossing = 1;}
					if (yLoc + newV[1] <= side + RoadBuilder.panel.roadW + r) {
						curbed = true;
						dv = yield(approaching,currentScene);
						newV = sumV(v,dv);}}
				break;
		case 1: dv = yield(approaching,currentScene);
				newV = sumV(v,dv);
				if (dir == 1) {
					if (yLoc + newV[1] > side) {
						newV[0]  = 0;
						crossing = 2;}}
				else {
					if (yLoc + newV[1] < side + RoadBuilder.panel.roadW) {
						newV[0]  = 0;
						crossing = 2;}}
				break;
		case 2: if (curbed == true) {
					dv	 = yield(approaching,currentScene);
					newV = sumV(v,dv);}
				break;
		default: break;}
		
		if (v[0] != 0) {		//avoid bouncing off walls
			double whichDir = Math.signum(v[0]);
			if (whichSide == -whichDir && Math.signum(newV[0]) == -whichDir) {
				newV[0] = 0;}}
		newV = limitV(newV);
		
		if (currentScene != null) {
			pGap0 = currentScene.gap0;
			pGap1 = currentScene.gap1;
			pGap2 = currentScene.gap2;
			pGap3 = currentScene.gap3;}
		
		if (!curbed && debug) {
			for (Integer key : dictNearestTurtleAtDecision.keySet()) {
				double cLocat = dictNearestTurtleAtDecision.get(key).xLoc;
				double cM = Math.abs(cLocat - this.xLoc) * RoadBuilder.spaceScale;
				double cV = dictNearestTurtleAtDecision.get(key).v;
				double cA = dictNearestTurtleAtDecision.get(key).acc;
				ArrayList<Double> tempList = dictSpeedsSinceDecision.get(key);
				ArrayList<Double> tempListA = dictAccelsSinceDecision.get(key);
				ArrayList<Double> tempListM = dictDistMSinceDecision.get(key);
				tempList.add(cV);
				tempListA.add(cA);
				tempListM.add(cM);
				dictSpeedsSinceDecision.put(key, tempList);
				dictAccelsSinceDecision.put(key, tempListA);
				dictDistMSinceDecision.put(key, tempListM);}
			for (Integer key : dictFollowingTurtleAtDecision.keySet()) {
				if (dictFollowingTurtleAtDecision.get(key) != null) {
					double cV = dictFollowingTurtleAtDecision.get(key).v;
					ArrayList<Double> tempList = dictFollowingCarSpeedsSinceDecision.get(key);
					tempList.add(cV);
					dictFollowingCarSpeedsSinceDecision.put(key, tempList);}}}
	}
	
	
	/*
	 * Moves peds based on results of calc()
	 * Sets value of crossing in some instances
	 */
	public void walk() {
		v = newV;
		move(myLoc,v);
		xLoc = myLoc.getX();
		yLoc = myLoc.getY();
		switch (crossing) {
		case 0:	if (dir == 1) {
					if (side - yLoc < r) {
						crossing = 1;}}
				else {
					if (yLoc - RoadBuilder.panel.roadW - side < r) {
						crossing = 1;}}
				break;
		case 2: if (dir == 1) {
					if (yLoc >= side + RoadBuilder.panel.roadW) crossing = 3;}
				else {
					if (yLoc <= side) crossing = 3;}
				break;
		default: break;}
		age ++;
	}
	
	public class Observation {
		Turtle nearest0, nearest1, nearest2, nearest3;
		ArrayList<Integer> lanesWithoutEyeContact = new ArrayList<Integer>();
		ArrayList<Turtle> blockedCars = new ArrayList<Turtle>();
		double gap0, gap1, gap2, gap3;
	}
	
	public Observation LookForApproachingCars(ArrayList<Turtle> approaching) {
		Observation cScene = new Observation();
		cScene.blockedCars = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching0 = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching1 = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching2 = new ArrayList<Turtle>();
		ArrayList<Turtle> approaching3 = new ArrayList<Turtle>();	
		cScene.nearest0 = null;
		cScene.nearest1 = null;
		cScene.nearest2 = null;
		cScene.nearest3 = null;
		
		cScene.gap0 = cScene.gap1 = cScene.gap2 = cScene.gap3 = RoadBuilder.roadL/2;
		
		//find nearest car in each lane
		if (!approaching.isEmpty()) {
			for (Turtle n : approaching) {
				if (n.dir == dir) {
					if (n.lane == 0) {
						approaching0.add(n);}
					else {
						approaching1.add(n);}}
				else {
					if (n.lane == 1) {
						approaching2.add(n);}
					else {
						approaching3.add(n);}}}
			if(!approaching0.isEmpty()) {
				for (Turtle o : approaching0) {
					double thisGap = o.xLoc - xLoc;
					if(Math.abs(thisGap) < Math.abs(cScene.gap0)) {
						cScene.gap0 = thisGap;
						cScene.nearest0 = o;}}}
			if(!approaching1.isEmpty()) {
				for (Turtle o : approaching1) {
					double thisGap = o.xLoc - xLoc;
					if(Math.abs(thisGap) < Math.abs(cScene.gap1)) {
						cScene.gap1 = thisGap;
						cScene.nearest1 = o;}}}
			if(!approaching2.isEmpty()) {
				for (Turtle o : approaching2) {
					double thisGap = o.xLoc - xLoc;
					if(Math.abs(thisGap) < Math.abs(cScene.gap2)) {
						cScene.gap2 = thisGap;
						cScene.nearest2 = o;}}}
			if(!approaching3.isEmpty()) {
				for (Turtle o : approaching3) {
					double thisGap = o.xLoc - xLoc;
					if(Math.abs(thisGap) < Math.abs(cScene.gap3)) {
						cScene.gap3 = thisGap;
						cScene.nearest3 = o;}}}}
		
		if (RoadBuilder.panel.inclObstruct) {
			//double threat
			double carW = UserPanel.carWidth;
			double carL = UserPanel.carLength;
			double[] gaps = new double[] {cScene.gap0, cScene.gap1, cScene.gap2, cScene.gap3};
			for (double cGap : gaps) {
				if (cGap == RoadBuilder.roadL/2) {
					cGap = 0;}}
			ArrayList<Turtle> closests = new ArrayList<Turtle>();
			if (cScene.nearest0 != null) {
				closests.add(cScene.nearest0);}
			else {
				closests.add(null);}
			if (cScene.nearest1 != null) {
				closests.add(cScene.nearest1);}
			else {
				closests.add(null);}
			if (cScene.nearest2 != null) {
				closests.add(cScene.nearest2);}
			else {
				closests.add(null);}
			if (cScene.nearest3 != null) {
				closests.add(cScene.nearest3);}
			else {
				closests.add(null);}
			double maxRelevantXDiff = Math.max(gaps[0], Math.max(gaps[1], Math.max(gaps[2], gaps[3])));
			double minRelevantXDiff = Math.min(gaps[0], Math.min(gaps[1], Math.min(gaps[2], gaps[3])));
			if (maxRelevantXDiff > 200/RoadBuilder.spaceScale) maxRelevantXDiff = 200/RoadBuilder.spaceScale;
			if (minRelevantXDiff < -200/RoadBuilder.spaceScale) minRelevantXDiff = -200/RoadBuilder.spaceScale;
			double maxRelevantX = xLoc + maxRelevantXDiff;
			double minRelevantX = xLoc + minRelevantXDiff;
			ArrayList<ViewAngle> obstructers = new ArrayList<ViewAngle>();
			ArrayList<Turtle> nearEnoughInX0 = new ArrayList<Turtle>();
			ArrayList<Turtle> nearEnoughInX1 = new ArrayList<Turtle>();
			ArrayList<Turtle> nearEnoughInX2 = new ArrayList<Turtle>();
			ArrayList<Turtle> nearEnoughInX3 = new ArrayList<Turtle>();
			
			for (Turtle i : RoadBuilder.flowSource.allCars) {
				if (i.xLoc > minRelevantX && i.xLoc < maxRelevantX) {
					if (dir == 1) {
						if (i.dir == 1) {
							if (i.lane == 0) {
								nearEnoughInX0.add(i);}
							else {
								nearEnoughInX1.add(i);}}
						else {
							if (i.lane == 1) {
								nearEnoughInX2.add(i);}
							else {
								nearEnoughInX3.add(i);}}}
					else {
						if (i.dir == -1) {
							if (i.lane == 0) {
								nearEnoughInX0.add(i);}
							else {
								nearEnoughInX1.add(i);}}
						else {
							if (i.lane == 1) {
								nearEnoughInX2.add(i);}
							else {
								nearEnoughInX3.add(i);}}}}}
			for (Turtle cClose : closests) {
				if (cClose == null) continue;
				ArrayList<Turtle> nearEnoughInY = new ArrayList<Turtle>();
				ArrayList<Turtle> nearEnough = new ArrayList<Turtle>();
				if (dir == cClose.dir) {
					if (cClose.lane == 0) {
						continue;}
					else {
						nearEnoughInY = new ArrayList<Turtle>(nearEnoughInX0);}}
				else {
					nearEnoughInY = new ArrayList<Turtle>(nearEnoughInX0);
					nearEnoughInY.addAll(nearEnoughInX1);
					if (cClose.lane == 0) {
						nearEnoughInY.addAll(nearEnoughInX2);}}
				
				if (dir == 1) {
					if (cClose.dir == 1) {
						for (Turtle j : nearEnoughInY) {
							if (j.dir == 1) {
								if (j.xLoc > cClose.xLoc && j.xLoc < xLoc + carL) {
									nearEnough.add(j);}}}}
					else {
						for (Turtle j : nearEnoughInY) {
							if (j.dir == 1) {
								if (j.xLoc > xLoc && j.xLoc < cClose.xLoc + carL) {
									nearEnough.add(j);}}
							else {
								if (j.xLoc > xLoc - carL && j.xLoc < cClose.xLoc) {
									nearEnough.add(j);}}}}}
				else {
					if (cClose.dir == 1) {
						for (Turtle j : nearEnoughInY) {
							if (j.dir == 1) {
								if (j.xLoc > cClose.xLoc && j.xLoc < xLoc + carL) {
									nearEnough.add(j);}}
							else {
								if (j.xLoc > cClose.xLoc - carL && j.xLoc < xLoc) {
									nearEnough.add(j);}}}}
					else {
						for (Turtle j : nearEnoughInY) {
							if (j.dir == -1) {
								if (j.xLoc > xLoc - carL && j.xLoc < cClose.xLoc) {
									nearEnough.add(j);}}}}}
				for (Turtle k : nearEnough) {
					double front	= k.xLoc - (double)k.dir*carL/3;
					double back		= k.xLoc - (double)k.dir*carL;
					double inside	= k.yLoc - (double)dir*carW/2;
					double outside	= k.yLoc + (double)dir*carW/2;
					Double thisTheta1, thisTheta2;
					ViewAngle thisView = null;
					if (dir == 1) {
						if (k.dir == 1) {
							if (back < xLoc) {
								thisTheta1 = FastMath.atan2(back - xLoc, inside - yLoc);}
							else {
								thisTheta1 = FastMath.atan2(back - xLoc, outside - yLoc);}
							if (front > xLoc) {
								thisTheta2 = FastMath.atan2(front - xLoc, inside - yLoc);}
							else {
								thisTheta2 = FastMath.atan2(front - xLoc, outside - yLoc);}}
						else {
							if (front < xLoc) {
								thisTheta1 = FastMath.atan2(front - xLoc, inside - yLoc);}
							else {
								thisTheta1 = FastMath.atan2(front - xLoc, outside - yLoc);}
							if (back > xLoc) {
								thisTheta2 = FastMath.atan2(back - xLoc, inside - yLoc);}
							else {
								thisTheta2 = FastMath.atan2(back - xLoc, outside - yLoc);}}}
					else {
						if (k.dir == 1) {
							if (back < xLoc) {
								thisTheta2 = FastMath.atan2(back - xLoc, inside - yLoc);}
							else {
								thisTheta2 = FastMath.atan2(back - xLoc, outside - yLoc);}
							if (front > xLoc) {
								thisTheta1 = FastMath.atan2(front - xLoc, inside - yLoc);}
							else {
								thisTheta1 = FastMath.atan2(front - xLoc, outside - yLoc);}}
						else {
							if (front < xLoc) {
								thisTheta2 = FastMath.atan2(front - xLoc, inside - yLoc);}
							else {
								thisTheta2 = FastMath.atan2(front - xLoc, outside - yLoc);}
							if (back > xLoc) {
								thisTheta1 = FastMath.atan2(back - xLoc, inside - yLoc);}
							else {
								thisTheta1 = FastMath.atan2(back - xLoc, outside - yLoc);}}}
					if (thisTheta1 != null) {
						thisTheta1 += Math.PI;
						thisTheta2 += Math.PI;
						thisView = new ViewAngle(k,thisTheta1,thisTheta2);
						obstructers.add(thisView);}}
				if (!obstructers.isEmpty()) {
					double front	= cClose.xLoc - (double)cClose.dir*carL/3;
					double back		= cClose.xLoc - (double)cClose.dir*carL;
					double inside	= cClose.yLoc - (double)dir*carW/2;
					double outside	= cClose.yLoc + (double)dir*carW/2;
					Double blockeeTheta1, blockeeTheta2;
					Double thetaDriver = FastMath.atan2(cClose.driverX - xLoc, cClose.driverY - yLoc);
					thetaDriver += Math.PI;
					if (dir == 1) {
						if (cClose.dir == 1) {
							if (back < xLoc) {
								blockeeTheta1 = FastMath.atan2(back - xLoc, inside - yLoc);}
							else {
								blockeeTheta1 = FastMath.atan2(back - xLoc, outside - yLoc);}
							if (front > xLoc) {
								blockeeTheta2 = FastMath.atan2(front - xLoc, inside - yLoc);}
							else {
								blockeeTheta2 = FastMath.atan2(front - xLoc, outside - yLoc);}}
						else {
							if (front < xLoc) {
								blockeeTheta1 = FastMath.atan2(front - xLoc, inside - yLoc);}
							else {
								blockeeTheta1 = FastMath.atan2(front - xLoc, outside - yLoc);}
							if (back > xLoc) {
								blockeeTheta2 = FastMath.atan2(back - xLoc, inside - yLoc);}
							else {
								blockeeTheta2 = FastMath.atan2(back - xLoc, outside - yLoc);}}}
					else {
						if (cClose.dir == 1) {
							if (back < xLoc) {
								blockeeTheta2 = FastMath.atan2(back - xLoc, inside - yLoc);}
							else {
								blockeeTheta2 = FastMath.atan2(back - xLoc, outside - yLoc);}
							if (front > xLoc) {
								blockeeTheta1 = FastMath.atan2(front - xLoc, inside - yLoc);}
							else {
								blockeeTheta1 = FastMath.atan2(front - xLoc, outside - yLoc);}}
						else {
							if (front < xLoc) {
								blockeeTheta2 = FastMath.atan2(front - xLoc, inside - yLoc);}
							else {
								blockeeTheta2 = FastMath.atan2(front - xLoc, outside - yLoc);}
							if (back > xLoc) {
								blockeeTheta1 = FastMath.atan2(back - xLoc, inside - yLoc);}
							else {
								blockeeTheta1 = FastMath.atan2(back - xLoc, outside - yLoc);}}}
					if (blockeeTheta1 != null) {
						blockeeTheta1 += Math.PI;
						blockeeTheta2 += Math.PI;
						ArrayList<Double[]> blockedAngles = new ArrayList<Double[]>();
						blockedAngles.add(new Double[] {obstructers.get(0).theta1, obstructers.get(0).theta2});
						for (ViewAngle ang0 : obstructers) {
							double lowAngle0 = ang0.theta1;
							double hiAngle0 = ang0.theta2;
							boolean included = false;
							for (Double[] ang1 : blockedAngles) {
								if ((lowAngle0 >= ang1[0] && lowAngle0 <= ang1[1] ) && (hiAngle0 > ang1[1])) {
									ang1[1] = hiAngle0;
									included = true;}
								if ((hiAngle0 >= ang1[0] && hiAngle0 <= ang1[1]) && (lowAngle0 < ang1[0])) {
									ang1[0] = lowAngle0;
									included = true;}
								if (lowAngle0 < ang1[0] && hiAngle0 > ang1[1]) {
									ang1[0] = lowAngle0;
									ang1[1] = hiAngle0;
									included = true;}
								if (lowAngle0 == ang1[0] && hiAngle0 == ang1[1]) {
									included = true;}}
							if (!included) {
								blockedAngles.add(new Double[] {lowAngle0, hiAngle0});}}
						for (Double[] blockedThetas : blockedAngles) {
							if (blockedThetas[0] < blockeeTheta1 && blockedThetas[1] > blockeeTheta2) {
								if (!cScene.blockedCars.contains(cClose)) {
									cScene.blockedCars.add(cClose);}
								int laneWithNewTurtle = closests.indexOf(cClose);
								switch (laneWithNewTurtle) {
								case 0:
									cScene.nearest0 = cClose.follower;
									if (cClose.follower != null) {
										cScene.gap0 = cClose.follower.xLoc - xLoc;}
									else {
										cScene.gap0 = RoadBuilder.roadL/2;}
									break;
								case 1:
									cScene.nearest1 = cClose.follower;
									if (cClose.follower != null) {
										cScene.gap1 = cClose.follower.xLoc - xLoc;}
									else {
										cScene.gap1 = RoadBuilder.roadL/2;}
									break;
								case 2:
									cScene.nearest2 = cClose.follower;
									if (cClose.follower != null) {
										cScene.gap2 = cClose.follower.xLoc - xLoc;}
									else {
										cScene.gap2 = RoadBuilder.roadL/2;}
									break;
								case 3:
									cScene.nearest3 = cClose.follower;
									if (cClose.follower != null) {
										cScene.gap3 = cClose.follower.xLoc - xLoc;}
									else {
										cScene.gap3 = RoadBuilder.roadL/2;}
									break;}}
//							if (blockedThetas[0] < thetaDriver && blockedThetas[1] > thetaDriver) {
//							cScene.lanesWithoutEyeContact.add(closests.indexOf(cClose));}
							}}}}
		}
		return cScene;
	}
	
	
	/**
	 * Determines value of curbed by calling lag() and/or gap() for each lane
	 * @return output of accel() based on new value of curbed
	 */
	public double[] yield(ArrayList<Turtle> approaching, Observation currentScene) {
		if (debug) {
			dictThtBegAtDecision = new HashMap<Integer,Double>();
			dictTTcolAtDecision = new HashMap<Integer,Double>();
			dictRealTTcolAtDecision = new HashMap<Integer,Double>();
			dictTailTAtDecision = new HashMap<Integer,Double>();
			dictRealTailTAtDecision = new HashMap<Integer,Double>();
			dictTTclearAtDecision = new HashMap<Integer,Double>();
			dictRealTTClearAtDecision = new HashMap<Integer,Double>();
			dictNearestTurtleAtDecision = new HashMap<Integer, Turtle>();
			dictDistMSinceDecision = new HashMap<Integer, ArrayList<Double>>();
			dictSpeedsSinceDecision = new HashMap<Integer, ArrayList<Double>>();
			dictAccelsSinceDecision = new HashMap<Integer, ArrayList<Double>>();
			dictFollowingTurtleAtDecision = new HashMap<Integer, Turtle>();
			dictFollowingCarSpeedsSinceDecision = new HashMap<Integer, ArrayList<Double>>();}
		
		double[] frogger;
		boolean go0, go1, go2, go3;
		go0 = go1 = go2 = go3 = false;
		int goes0, goes1, goes2, goes3;
		goes0 = goes1 = goes2 = goes3 = 0;
		
//		ArrayList<Integer> lanesWithoutEyeContact = new ArrayList<Integer>();
		
		//decide crossing decision for each lane
		int count0 = 0;
		int count1 = 0;
		int count2 = 0;
		int count3 = 0;
		HashMap<Integer,Integer> waits = new HashMap<Integer,Integer>();
		for (int i = 0; i < 4; i++) {
			waits.put(i, -1);}
		if (nearest0 != null) {
			int[] lagOut = lag(nearest0,0);
			goes0 = lagOut[0];
			waits.put(0, lagOut[1]);
			while (goes0 == 0) {
				int[] gapOut = gap(nearest0,0); 
				goes0 = gapOut[0];
				waits.put(0, gapOut[1]);
				count0++;
				if (count0 > 5) {
					break;}}
			if (goes0 ==  1) {go0 = true;}
			if (goes0 == -1) {go0 = false;}}
		else {
			waits.put(0, 0);
			go0 = true;}
		if (nearest1 != null) {
			int[] lagOut = lag(nearest1,1);
			goes1 = lagOut[0];
			waits.put(1, lagOut[1]);
			while (goes1 == 0) {
				int[] gapOut = gap(nearest1,1);
				goes1 = gapOut[0];
				waits.put(1, gapOut[1]);
				count1++;
				if (count1 > 5) {
					break;}}
			if (goes1 ==  1) {go1 = true;}
			if (goes1 == -1) {go1 = false;}}
		else {
			waits.put(1, 0);
			go1 = true;}
		if (nearest2 != null) {
			int[] lagOut = lag(nearest2,2);
			goes2 = lagOut[0];
			waits.put(2, lagOut[1]);
			while (goes2 == 0) {
				int[] gapOut = gap(nearest2,2);
				goes2 = gapOut[0];
				waits.put(2, gapOut[1]);
				count2++;
				if (count2 > 5) {
					break;}}
			if (goes2 ==  1) {go2 = true;}
			if (goes2 == -1) {go2 = false;}}
		else {
			waits.put(2, 0);
			go2 = true;}
		if (nearest3 != null) {
			int[] lagOut = lag(nearest3,3);
			goes3 = lagOut[0];
			waits.put(3, lagOut[1]);
			while (goes3 == 0) {
				int[] gapOut = gap(nearest3,3);
				goes3 = gapOut[0];
				waits.put(3, gapOut[1]);
				count3++;
				if (count3 > 5) {
					break;}}
			if (goes3 ==  1) {go3 = true;}
			if (goes3 == -1) {go3 = false;}}		
		else {
			waits.put(3, 0);
			go3 = true;}
		
//		if (currentScene != null) {
//			for (Integer blind : currentScene.lanesWithoutEyeContact) {
//				waits.put(blind, 0);}}
		
		//tally crossing decisions to give final result
		curbed = true;
		if (!approaching.isEmpty()) {
			if (go0 && go1 && go2 && go3) {
				crossing = 2;
				if ((waits.get(0) == 0 || (nearest0.v < 1e-8 && nearest0.ying == 1)) && 
						(waits.get(1) == 0 || (nearest1.v < 1e-8 && nearest1.ying == 1)) &&
						(waits.get(2) == 0 || (nearest2.v < 1e-8 && nearest2.ying == 1)) && 
						(waits.get(3) == 0 || (nearest3.v < 1e-8 && nearest3.ying == 1))) {
					tickAtCrossDecision = RoadBuilder.clock.getTickCount();
					curbed = false;}}}
		else {
			tickAtCrossDecision = RoadBuilder.clock.getTickCount();
			curbed = false;}
		
		frogger = accel(myLoc,dir);
		return frogger;
	}
	
	/**
	 * Calculates whether or not pedestrian will go before next car
	 * If that car is passed, reassigns to following car
	 * @param t  - next car
	 * @param ln - lane car is in relative to ped
	 * @return:	[goes, waitForYield]
	 * goes: -1=don't go; 0=look to next car; 1=go
	 * waitForYield: 0=just go, 1=wait
	 */
	public int[] lag(Turtle t, int ln) {
		//TODO: include estimation errors/delay
		double approachV, approachX, approachA, dist, xDist, yDist, TTCol, TTClear, realTTCol, realTTClear;
		double threatBeg, confBeg; //, threatEnd;
		double maxVY = Math.abs(maxV*Math.cos(endPtTheta));
		int	goes = 0;
		//TODO: make xTime lane-dependent for some peds
		if (maxVY != 0) {
			xTime = accT + (endPtDist - side) / maxVY;}
		else {
			xTime = accT + (endPtDist - side) / maxV;}
		sigR	  = 0.01*UserPanel.tStep;	//st. dev. of relative approach rate
		approachX = t.xLoc;
		xDist	  = Math.abs(xLoc - approachX);
		yDist	  = (double)ln*RoadBuilder.panel.laneWidth + RoadBuilder.panel.laneWidth/2;
		dist	  = space.getDistance(myLoc, t.myLoc);
		approachV = t.v;
		//approachA = Math.max(t.acc, t.newAcc);
		approachA = t.acc;
		
		double realXDist = Math.sqrt(dist * dist - yDist*yDist);
		if (approachV != 0) {
			realTTCol = realXDist / t.v;
			realTTClear = realTTCol + t.length/approachV + 1/UserPanel.tStep;}
		else {
			realTTCol = 1000;
			realTTClear = 1000;}
		
		//include errors
		if (RoadBuilder.panel.estErr == true && approachV != 0) {
			etaS = rnd.nextGaussian();
			etaV = rnd.nextGaussian();
			if (t.dir == 1) {
				wS1 = UserPanel.wien1*wS1 + UserPanel.wien2*etaS;
				wV1 = UserPanel.wien1*wV1 + UserPanel.wien2*etaV;
				approachV = t.v - dist*sigR*wV1;
				dist  = dist*Math.exp(UserPanel.Vs*wS1);
				xDist = Math.sqrt(dist*dist - yDist*yDist);}
			else {
				wSn1 = UserPanel.wien1*wSn1 + UserPanel.wien2*etaS;
				wVn1 = UserPanel.wien1*wVn1 + UserPanel.wien2*etaV;
				approachV = t.v - dist*sigR*wVn1;
				dist  = dist*Math.exp(UserPanel.Vs*wSn1);
				xDist = Math.sqrt(dist*dist - yDist*yDist);}}
		
		//calculate relevant times
		if (ln == 0) {
			threatBeg = 0;
			confBeg = 0;}
		else {
			if (maxVY != 0) {
				//threatBeg = accT + (ln-.25)*RoadBuilder.laneW/maxVY;}	//ped enters lane //TODO:mention this 0.25 and 1 below in writeup
				threatBeg = accT + ln*RoadBuilder.panel.laneWidth/maxVY;
				if (ln == 1) {
					confBeg = 0;}
				else {
					confBeg = accT + (ln - 1)*RoadBuilder.panel.laneWidth/maxVY;}}
			else {
				//threatBeg = accT + (ln-.25)*RoadBuilder.laneW/maxV;}}
				threatBeg = accT + ln*RoadBuilder.panel.laneWidth/maxV;
				if (ln == 1) {
					confBeg = 0;}
				else {
					confBeg = accT + (ln - 1)*RoadBuilder.panel.laneWidth/maxV;}}}
//		threatEnd = accT + (ln+1)*RoadBuilder.laneW/maxVY;		//ped exits lane
		TTCol = 1000;
		TTClear = 1000;
		//double relevantAcc = Math.max(t.acc, t.newAcc);
		double relevantAcc = t.acc;
		if (approachV != 0) {
			if (approachA <= 1e-5) {
				TTCol	= xDist/approachV;}
			else {
				TTCol = -(approachV/approachA) + Math.sqrt((approachV*approachV) + 2*approachA*xDist)/approachA;}
			TTClear	= TTCol + t.length/approachV + 1/UserPanel.tStep;} //TODO: add radius of ped to this calculation
		else if (relevantAcc > 0) {
			TTCol = Math.sqrt(2*relevantAcc*xDist)/relevantAcc;}
		
		if (debug) {
			dictThtBegAtDecision.put(ln, threatBeg); 
			dictTTcolAtDecision.put(ln, TTCol);
			dictRealTTcolAtDecision.put(ln, realTTCol);
			dictTTclearAtDecision.put(ln, TTClear);
			dictRealTTClearAtDecision.put(ln, realTTClear);
			dictNearestTurtleAtDecision.put(ln, t);
			dictDistMSinceDecision.put(ln, new ArrayList<Double>());
			dictSpeedsSinceDecision.put(ln, new ArrayList<Double>());
			dictAccelsSinceDecision.put(ln, new ArrayList<Double>());
			dictFollowingTurtleAtDecision.put(ln, t.follower);
			dictFollowingCarSpeedsSinceDecision.put(ln, new ArrayList<Double>());}
		
		//decide if lag is big enough to start crossing
		if (threatBeg + critGap < TTCol) {
			goes = 1;
			front = 1;}
		//else if (threatBeg > TTClear + 1/UserPanel.tStep && TTCol < (t.decelT-1)) {
		else if (confBeg > TTClear && TTCol < (t.decelT - 1)) {
			//TODO: 1 is arbitrary. scale w minGap, find literature
			//TODO: decelT should be based on ped values
			if (t.follower != null) {
				goes = 0;}
			else {
				goes = 1;
				front = -1;}}
		else {
			goes = -1;}
		
		int toWait = 0;
		double stopBarX = 0;
		if (t.dir == 1) {
			stopBarX = RoadBuilder.xWalkx - 2*RoadBuilder.panel.stopBarDistance;}
		else {
			stopBarX = RoadBuilder.xWalkx + 2*RoadBuilder.panel.stopBarDistance;}
		boolean withinStopBar = (t.dir * (int)Math.signum(t.xLoc - stopBarX)) == 1;
		
		if (withinStopBar && goes == 1 && front == 1 && !yielders.contains(t)) {
			toWait = 1;}
		
		int[] outGoes = new int[] {goes, toWait};
		return outGoes;
	}
	
	/**
	 * Calculates if gap is sufficient to go
	 * @param t1 - car that ped will cross in front of
	 * @param ln - lane car is in
	 * @return: -1=don't go; 0=look to next car; 1=go
	 */
	public int[] gap(Turtle t1, int ln) {		//TODO: peds still fucking up the rolling gap
		int goes = 0;						//TODO: make the perception of necessary stopping speed error-prone
		Turtle t2 = t1.follower;
		double t1x, dist1, t1d, yDist, TTCol, TTClear, realTTCol, realTTClear, realTailT, t1v;
		double t2v, t2x, dist2, t2d;
		double threatBeg, confBeg; //, threatEnd;
		double maxVY = Math.abs(maxV*Math.cos(endPtTheta));
		if (maxVY != 0) {
			xTime = accT + (endPtDist - side) / maxVY;}
		else {
			xTime = accT + (endPtDist - side) / maxV;}
		sigR = 0.01*UserPanel.tStep;	//standard deviation of relative approach rate
		t1x		= t1.xLoc;
		t2x 	= t2.xLoc;
		t1d		= Math.abs(xLoc - t1x);
		t2d		= Math.abs(xLoc - t2x);
		yDist	= (double)ln*RoadBuilder.panel.laneWidth + RoadBuilder.panel.laneWidth/2;
		dist1	= space.getDistance(myLoc, t1.myLoc);
		dist2	= space.getDistance(myLoc, t2.myLoc);
		t1v		= t1.v;
		t2v		= t2.v;		
		
		double realXDist = t1d;
		double realXDist2 = t2d;
		double realTail = Math.abs(t2d - t1d) - t1.length;
		if (t2v != 0) {
			realTTCol = realXDist2 / t2v;
			realTTClear = realTTCol + t2.length/t2v + 1/UserPanel.tStep;
			realTailT = realTail/t2v;}
		else {
			realTTCol = 1000;
			realTTClear = 1000;
			realTailT = 1000;}
		
		//include errors
		if (RoadBuilder.panel.estErr == true && t2v != 0) {
			etaS	= rnd.nextGaussian();
			etaV	= rnd.nextGaussian();
			if (t1.dir == 1) {
				wS1		= UserPanel.wien1*wS1 + UserPanel.wien2*etaS;
				wV1		= UserPanel.wien1*wV1 + UserPanel.wien2*etaV;
				t1v		= t1v - dist1*sigR*wV1;
				t2v		= t2v - dist2*sigR*wV1;
				dist1	= dist1*Math.exp(UserPanel.Vs*wS1);
				dist2	= dist2*Math.exp(UserPanel.Vs*wS1);
				t1d		= Math.sqrt(dist1*dist1 - yDist*yDist);
				t2d		= Math.sqrt(dist2*dist2 - yDist*yDist);}
			else {
				wSn1	= UserPanel.wien1*wSn1 + UserPanel.wien2*etaS;
				wVn1	= UserPanel.wien1*wVn1 + UserPanel.wien2*etaV;
				t1v		= t1v - dist1*sigR*wVn1;
				t2v		= t2v - dist2*sigR*wVn1;
				dist1	= dist1*Math.exp(UserPanel.Vs*wSn1);
				dist2	= dist2*Math.exp(UserPanel.Vs*wSn1);
				t1d		= Math.sqrt(dist1*dist1 - yDist*yDist);
				t2d		= Math.sqrt(dist2*dist2 - yDist*yDist);}}
		
		//calculate relevant times
		double thisTail   = Math.abs(t2d - t1d) - t1.length;
		double thisTailT  = 1000;
		double tailTWithFollowerV = 1000;
		double tailTWithLeaderV = 1000;
		if (t2v != 0) {
			tailTWithFollowerV = thisTail/t2v;}
		if (t1v != 0) {
			tailTWithLeaderV = 1000;}
//		if (tailTWithLeaderV < tailTWithFollowerV) {
//			thisTailT = (tailTWithLeaderV + tailTWithFollowerV)/2;}
//		else {
//			thisTailT = tailTWithFollowerV;}								
		thisTailT = Math.min(tailTWithLeaderV, tailTWithFollowerV);		//TODO: add this to writeup
		
		if (ln == 0) {
			threatBeg = 0;
			confBeg = 0;}
		else {
			if (maxVY != 0) {
				threatBeg = accT + (ln*RoadBuilder.panel.laneWidth/maxVY);
				if (ln == 1) {
					confBeg = 0;}
				else {
					 confBeg = accT + (ln - 1)*RoadBuilder.panel.laneWidth/maxVY;}}
			else {
				threatBeg = accT + (ln*RoadBuilder.panel.laneWidth/maxV);
				if (ln == 1) {
					confBeg = 0;}
				else {
					 confBeg = accT + (ln - 1)*RoadBuilder.panel.laneWidth/maxV;}}}
//		threatEnd = accT + (ln+1)*RoadBuilder.laneW/maxVY;
		TTCol	= 1000;
		TTClear = 1000;
		//double relevantAcc = Math.max(t2.acc, t2.newAcc);
		double relevantAcc = t2.acc;
		if (t2v != 0) {
			TTCol	= Math.abs(t2d/t2v);
			TTClear	= TTCol + t2.length/t2v + 1/UserPanel.tStep;} //TODO: add radius of ped to this calculation
		else if (relevantAcc > 0) {
			TTCol = Math.sqrt(2*relevantAcc*t2d)/relevantAcc;}
		
		if (debug) {
			dictThtBegAtDecision.put(ln, threatBeg); 
			dictTTcolAtDecision.put(ln, TTCol);
			dictRealTTcolAtDecision.put(ln, realTTCol);
			dictTTclearAtDecision.put(ln, TTClear);
			dictRealTTClearAtDecision.put(ln, realTTClear);
			dictTailTAtDecision.put(ln, thisTailT);
			dictRealTailTAtDecision.put(ln, realTailT);
	//		dictFollowingTurtleAtDecision.put(ln, t2);
			dictDistMSinceDecision.put(ln, new ArrayList<Double>());
			dictSpeedsSinceDecision.put(ln, new ArrayList<Double>());
			dictAccelsSinceDecision.put(ln, new ArrayList<Double>());
	//		dictFollowingTurtleAtDecision.put(ln, t1);
			dictFollowingCarSpeedsSinceDecision.put(ln, new ArrayList<Double>());}
		
		//decide if gap is big enough to start crossing
		if (thisTailT > critGap && threatBeg + critGap < TTCol) {
			goes = 1;
			front = 2;}
		//else if (threatBeg > TTClear) {
		else if (confBeg > TTClear) {
			if (t2.follower != null) {	//switch to observing next car
				switch (ln) {
				case 0:	nearest0 = t2;
						break;
				case 1: nearest1 = t2;
						break;
				case 2: nearest2 = t2;
						break;
				case 3: nearest3 = t2;
						break;	
				default: break;}}
			else {
				goes = 1;
				front = -2;}}
		else {
			goes = -1;}

		int toWait = 0;
		double stopBarX = 0;
		if (t2.dir == 1) {
			stopBarX = RoadBuilder.xWalkx - RoadBuilder.panel.stopBarDistance;}
		else {
			stopBarX = RoadBuilder.xWalkx + RoadBuilder.panel.stopBarDistance;}
		boolean withinStopBar = (t2.dir * (int)Math.signum(t2.xLoc - stopBarX)) == 1;
		
		if (withinStopBar && goes == 1 && !yielders.contains(t2)) {
			toWait = 1;}
		
		int[] outGoes = new int[] {goes, toWait};
		return outGoes;
	}
	
	/**
	 * Calculates all forces acting on ped
	 * @param location
	 * @param direct
	 * @return
	 */
	public double[] accel(NdPoint location, int direct) {
		forcesX = new ArrayList<Double>();
		forcesY = new ArrayList<Double>();
		double xF, yF, curbF, boxF, dBox;
		double[] acc;
		xF = yF = curbF = 0;
		
		double carW = UserPanel.carWidth;
		double carL = UserPanel.carLength;
		boolean carInPath = false;
		Turtle theCar = null;
		NdPoint cornerOfCar = new NdPoint();
		double absDistCar = 0;
		if (crossing == 2) {
			for (Turtle b : RoadBuilder.flowSource.allCars) {
				if (yielders.contains(b) && b.v == 0) continue;
				boolean carAhead = false;
				double carY = 0;
				if (dir == 1) {
					carY = b.yLoc - carW/2;
					if (carY > yLoc) carAhead = true;
					absDistCar = Math.abs(carY - yLoc) - r;}
				else {
					carY = b.yLoc + carW/2;
					if (carY < yLoc) carAhead = true;
					absDistCar = Math.abs(carY - yLoc) - r;}
				if (!carAhead) continue;
				if (absDistCar > RoadBuilder.panel.laneWidth) continue;
				double backOfCar = 0;
				double frontOfCar = b.xLoc;
				if (b.dir == 1) {
					frontOfCar += 1/RoadBuilder.spaceScale;
					if (b.xLoc > (xLoc - r) && (b.xLoc - carL) < (xLoc + r)) {
						backOfCar = b.xLoc - carL - 1/RoadBuilder.spaceScale;
						if (yielders.contains(b)) {
							cornerOfCar = new NdPoint(frontOfCar, carY);}
						else {
							cornerOfCar = new NdPoint(backOfCar, carY);}
						carInPath = true;
						theCar = b;
						break;}}
				else {
					frontOfCar -= 1/RoadBuilder.spaceScale;
					if (b.xLoc < (xLoc + r) && (b.xLoc + carL) > (xLoc - r)) {
						backOfCar = b.xLoc + carL + 1/RoadBuilder.spaceScale;
						if (yielders.contains(b)) {
							cornerOfCar = new NdPoint(frontOfCar, carY);}
						else {
							cornerOfCar = new NdPoint(backOfCar, carY);}
						carInPath = true;
						theCar = b;
						break;}}}}
		
		if (carInPath) {
			double cornerTheta;
			double cornerDist  = space.getDistance(location, cornerOfCar);
			double cornerDelX  = cornerOfCar.getX() - location.getX();
			if (cornerDist != 0) {
				cornerTheta = FastMath.asin((double)direct*cornerDelX/cornerDist);}
			else {
				cornerTheta = 0;}
			if (direct == -1) {
				cornerTheta += Math.PI;}
			
			Double aroundCarFx = (maxV*Math.sin(cornerTheta) - v[0])/accT;
			Double aroundCarFy = (maxV*Math.cos(cornerTheta) - v[1])/accT;
			forcesX.add(aroundCarFx);
			forcesY.add(aroundCarFy);
			
			Double forceCar = -(double)dir*A*Math.exp((-absDistCar)/B)/m;
			forcesY.add(forceCar);}
		
			
			
		//calculate heading to endpoint (or around car)
		endPtDist  = space.getDistance(location, endPt); 
		double endPtDelX  = endPt.getX()-location.getX();
		if (endPtDist != 0) {
			endPtTheta = FastMath.asin((double)direct*endPtDelX/endPtDist);}
		else {
			endPtTheta = 0;}
		if (direct == -1) {
			endPtTheta += Math.PI;}
		
		//calculate motive force
		Double motFx = (maxV*Math.sin(endPtTheta) - v[0])/accT;
		Double motFy = (maxV*Math.cos(endPtTheta) - v[1])/accT;
		forcesX.add(motFx);
		forcesY.add(motFy);
			
		
		
		//calculate interactive forces
		//TODO: write code to make a threshold for interaction instead of the arbitrary horizon		
		if (RoadBuilder.flowSource.nXing < 20) {
			for (Ped a : RoadBuilder.flowSource.allPeds) {
				if (a != this && (a.dir == dir || a.crossing == 2)) {
					NdPoint	otherLoc = space.getLocation(a);
					double	otherY	 = otherLoc.getY();
					double	visible	 = Math.signum((double)dir*(otherY-yLoc));
					if (visible == 1) {		//peds only affected by those in front of them
						double absDist = space.getDistance(location, otherLoc);
						if (absDist < horiz) {
							double delX    = location.getX()-otherLoc.getX();
							double delY    = location.getY()-otherLoc.getY();
							double delXabs = Math.abs(delX);
							double signFx  = Math.signum(delX);
							double signFy  = Math.signum(delY);
							double theta;
							if (absDist != 0) {
								theta = FastMath.asin(delXabs/absDist);}
							else {
								theta = 0;}
							double rij     = r + a.r;
							Double interFx = signFx*A*Math.exp((rij-absDist)/B)*Math.sin(theta)/m;
							Double interFy = signFy*A*Math.exp((rij-absDist)/B)*Math.cos(theta)/m;
							forcesX.add(interFx);
							forcesY.add(interFy);}}}}}
		
		//stop at curb if necessary
		if (curbed == true /*&& !UserPanel.calcFun*/) {
			double dCurb;
			if (direct == 1) {dCurb = side - yLoc;}
			else {dCurb = yLoc - side - RoadBuilder.panel.roadW;}
			curbF = -(double)direct*(A*Math.exp((r-dCurb)/B) + k*(r-dCurb))/m;
			forcesY.add(curbF);}

		//keep peds within crossing box
		if (boxed) {
			boxF = 0;
			whichSide = Math.signum(RoadBuilder.xWalkx - xLoc); //1=left, -1=right
			if (whichSide == 1) {dBox = xLoc - boxL;}
			else {dBox = boxR - xLoc;}
			boxF = whichSide*(A*Math.exp((r-dBox)/B) + k*(r-dBox))/m;
			forcesX.add(boxF);}
		
		//sum all forces
		for (Double b : forcesX) {
			xF += b;}
		for (Double c : forcesY) {
			yF += c;}
		acc = new double[] {xF, yF};
		return acc;
	}

	/**
	 * Move peds and update context
	 * @param loc
	 * overloaded. optional parameters:
	 * @param displacement
	 * @param destination
	 */
	public void move(NdPoint loc, double[] displacement) {
		double[] zero = new double[] {0,0};
		double yl = loc.getY();
		if (yl + displacement[1] > RoadBuilder.panel.worldW || yl + displacement[1] < 0) {
			RoadBuilder.flowSource.killListP.add(this);}
		else if (displacement != zero) {	
			space.moveByDisplacement(this,displacement);
//			grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
			myLoc = space.getLocation(this);}
	}
	public void move(NdPoint loc, NdPoint destination) {
		double xd = destination.getX();
		double yd = destination.getY();
		if (yd > RoadBuilder.panel.worldW || yd < 0) {
			RoadBuilder.flowSource.killListP.add(this);}
		else if (loc != destination) {
				space.moveTo(this,xd,yd);
//				grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
				myLoc = space.getLocation(this);}
	}
	
	/**
	 * Keep velocities within acceptable ranges
	 * @param input
	 * @return velocity below maxV
	 */
	public double[] limitV(double[] input) {
		double totalV, norm;
		if (this.dir == 1) {
			if (input[1] < 0) {
				input[1] = 0;}}
		else {
			if (input[1] > 0) {
				input[1] = 0;}}
		totalV = Math.sqrt(input[0]*input[0] + input[1]*input[1]);
		if (totalV > maxV) {
			norm = maxV/totalV;
			input[0] = input[0]*norm;
			input[1] = input[1]*norm;}
		return input;
	}
	
	/**
	 * Adder for 2-dimensional arrays
	 * @param  a = double[2]
	 * @param  b = double[2]
	 * @return c = double[2]
	 */
	public double[] sumV(double[] a, double[] b) {
		double[] c = new double[2];
		for (int i = 0; i < 2; i++) {
			c[i] = a[i] + b[i];}
		return c;
	}
	
	/**
	 * Creates pedestrian agents and initializes values
	 * Called by scheduler in Agent.java
	 * @param contextSpace
	 * @param contextGrid
	 * @param direction
	 */
	public Ped(ContinuousSpace<Object> contextSpace, int direction) {
		yielders = new ArrayList<Turtle>();
		space	 = contextSpace;
		maxV	 = rnd.nextGaussian() * UserPanel.pedVsd + UserPanel.pedVavg;
		dir		 = direction; // 1 moves up, -1 moves down
		v		 = new double[] {0,(double)dir*.5*maxV};
		crossing = 0;
		curbed   = false;
		age		 = 0;
		front	 = 0;
		wS1  = rnd.nextGaussian();
		wSn1 = rnd.nextGaussian();
		etaS = rnd.nextGaussian(); 
		wV1  = rnd.nextGaussian();
		wVn1 = rnd.nextGaussian();
		etaV = rnd.nextGaussian();
		double gapParamA	= UserPanel.pedGapParamA; //TODO: email Brewer re: value for just convergent sets
		double gapParamB	= 0.942;  //TODO: ditto
		double gapMin		= 1/(1+Math.exp(gapParamA));
		double preGap0		= rnd.nextDouble();
		double preGap		= preGap0*(1-gapMin) + gapMin; //avoids negative values
		double critGapS		= (gapParamA - Math.log((1/preGap) - 1))/gapParamB; //from Brewer 2006
		critGap = critGapS/UserPanel.tStep; //critical gap in simulation time units
		
		//3-circle variables - from Helbing, et al (2000) [r from Rouphail et al 1998]
		//TODO: accT is strange - should it not vary with how far from maxV the ped is?
		accT  = 0.5/UserPanel.tStep;						//acceleration time
		m     = 80;											//avg ped mass in kg
		horiz = 5/RoadBuilder.spaceScale;					//distance at which peds affect each other
		A     = 2000*UserPanel.tStep*UserPanel.tStep/RoadBuilder.spaceScale;	//ped interaction constant (kg*space units/time units^2)
		B     = 0.08/RoadBuilder.spaceScale;					//ped distance interaction constant (space units)
		k	  = 120000*UserPanel.tStep*UserPanel.tStep;			//wall force constant
		r     = 0.275/RoadBuilder.spaceScale;					//ped radius (space units)
		
		//store endpoint
		if (dir == 1) endPt = new NdPoint(RoadBuilder.xWalkx + 1/RoadBuilder.spaceScale, RoadBuilder.panel.worldW + 1);
		else endPt = new NdPoint(RoadBuilder.xWalkx - 1/RoadBuilder.spaceScale, -1);
	}
	
	public void Clear() {
		yielders = new ArrayList<Turtle>();
	}
	
	class ViewAngle {
		Turtle blocker;
		Turtle blockee;
		double theta;
		double theta1;
		double theta2;
		ViewAngle(Turtle _blockee, double theta) {
			this.blockee = _blockee;
			this.theta = theta;}
		ViewAngle(Turtle _blocker, double theta1, double theta2) {
			this.blocker = _blocker;
			this.theta1 = theta1;
			this.theta2 = theta2;}
	}
	
	/**
	 * Getter for identification
	 */
	@Override
	public boolean isPed() {
		return true;}
	
	/**
	 * Parameter declarations for probe
	 */
	@Parameter(usageName="v", displayName="Current yVel")
	public double getVel() {
		double vY = v[1];
		return vY*UserPanel.vBase;}
	@Parameter(usageName="crossing", displayName="Crossing?")
	public double getCrossing() {
		return crossing;}
	@Parameter(usageName="cg", displayName="Critical gap")
	public double getGap() {
		return critGap*UserPanel.tStep;}
	@Parameter(usageName="xTime",displayName="xTime")
	public double getXtime() {
		return xTime*UserPanel.tStep;}
	@Parameter(usageName="loc",displayName="y")
	public double getYloc() {
		return yLoc;}
	@Parameter(usageName="front", displayName="front")
	public int getFront() {
		return front;}
}