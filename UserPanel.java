package driving1;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.*;

import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.parameter.Parameter;
import repast.simphony.parameter.Parameters;
import repast.simphony.ui.RSApplication;

/*
 * Class to implement the user panel and initialize model variables
 * author: Darryl Michaud
 */

public class UserPanel implements ActionListener{
	static Parameters param = RunEnvironment.getInstance().getParameters();
	public boolean deathKnell = false;
	// declare model parameters
	static final  double spaceScale	= RoadBuilder.spaceScale;
	static public double tStep		= 0.05;		// duration of one tick in seconds	
	static final  double vBase 		= (spaceScale/tStep)*3600/1000;
						// vBase is the natural speed of the model (one cell per tick), converted to km/hr
	static final  double simHours	= .01;				//hours in sim (currently 5)
	static final  double simLength  = simHours*60*60/tStep;	//in ticks
	//static public double percV2X	= 0;
	//static public double percAuto	= 0;
	//static public double percBoth	= 100;
	//static public int    vehRho		= 600;			// veh/hr each dir 	default: 600
	//static public int    pedRho		= 60;			// ppl/hr each dir	default:  60		//should these by total or each (would add factor of two in calc)
//	static public double percV2X	= (double)param.getValue("percV2X");
//	static public double percAuto	= (double)param.getValue("percAuto");
//	static public double percBoth	= (double)param.getValue("percBoth");
	public double percV2X;
	public double percAuto;
	public double percBoth;
	
	static public int    vehRho		= (int)param.getValue("vehRho");
	static public int    pedRho		= (int)param.getValue("pedRho");
	static public double confLimS	= 1.7;			// seconds			default:   1.7		//kinda arbitrary
	static public double hPercLimM	= 100;			// human pedestrian perception (meters)	//kinda arbitrary
	static public double aPercLimM	= 60;			// automated ped perception (meters)
	
	static public boolean estErr 	= true;		// estimation errors
	static public boolean BRT		= true;
	static public boolean ADRT		= true;
	static public boolean inclRL	= false;  //include red lights?
	static public boolean calcFun	= false;  //build fundamental diagram
	static public boolean bothCar	= true;   //cars both directions?
	static public boolean IIDM		= true;   //include Improved IDM?
//	static public boolean pedsUp	= true;
//	static public boolean pedsDn	= true;
	
	// declare variables in real world units
	static public double sLimitKH		= 45;			// km/hr			default:  45
	static final  double pedVavgKH		= 5;			// km/hr			default:   5 	 (source Zebala 2012)
	static final  double pedVsdKH		= 0.936;		// km/hr			default:   0.936 (source Still 2000)
	static final  double maxaScaleMS	= 0.132;		// m/s2				default:   0.132  \
	static final  double maxaShape		= 0.6461;		// model units		default:   0.6461  |
	static final  double minaScaleMS	= 0.5372;		// m/s2				default:   0.5372 /
	static final  double minaShape		= 0.7246;		// model units		default:   0.7246 \  source: kim and
	static final  double jamHeadScaleM	= 0.6517;		// m				default:   0.6517 /    mahmassani 2011
	static final  double jamHeadShape	= 0.4979;		// model units		default:   0.4979 \
	static final  double tGapS			= 1.266;		// s				default:   1.266   |
	static final  double tGapS_sd		= 0.507;		// s				default:   0.507  /
	static public double pedGapParamA   = 6.2064;
	
	static public double sLimitMuKH = sLimitKH + 2;	// km/hr			source:	Fitzpatrick et al 2003
	static public double sLimitSDKH = 4.5;			// km/hr					ditto
	static final  double emergDecMS	= 7.4;			// m/s2				default:   7.4 (source Greibe 2007)
	static final  double carLengthM	= 5.28;			// m			source: http://usatoday30.usatoday.com/money/autos/2007-07-15-little-big-cars_N.htm
	static final  double carWidthM	= 1.89;			// m					avg of lg sedan 1990 & 2007
	static public double DmuHatS	= -0.4552452;	// distraction dist scale param (in seconds)
	static public double DsigHat	= 0.6108071;	// distraction dist shape param (already in model units)
	static public double interDlamS	= 0.3524525;	// interdistraction rate (in seconds^-1)
	static public double SmuHatM	= 1.419;		// stopBar dist scale param (in meters)
	static public double SsigHat	= 0.486;		// stopBar dist shape param (already in model units)
	static public int	 cycleTimeS	= 90;
	static public int	 greenDurS	= 60;			// duration of green light in sec
	static public int	 amberDurS	= RedLight.amberT(sLimitKH);
	static public int	 redDurS	= cycleTimeS - greenDurS - amberDurS;
	static public double calcTSpanS	= 15;			// timespan for fundamental diagram calculations (in seconds)
	//TODO: add ped gap coefficients
	
	// calculate population range constants
	public double V2Xlo;
	public double V2Xhi;
	public double autLo;
	public double autHi;
	public double bothLo;
	public double bothHi;
		
	// convert variables to model units
	static public double sLimit 		= sLimitKH/vBase;
	static final  double sLimitMu		= sLimitMuKH/vBase;
	static final  double sLimit_sd		= sLimitSDKH/vBase;
	static final  double pedVavg		= pedVavgKH/vBase;
	static final  double pedVsd			= pedVsdKH/vBase;
	//static final  double maxa			= maxaScaleMS*tStep*tStep/spaceScale;
	static final  double maxaScale		= maxaScaleMS + 2*Math.log(tStep) - Math.log(spaceScale);
	static final  double minaScale		= minaScaleMS + 2*Math.log(tStep) - Math.log(spaceScale);
	static final  double jamHeadScale	= jamHeadScaleM - Math.log(spaceScale);
	static final  double tGap			= tGapS/tStep;
	static final  double tGap_sd		= tGapS_sd/tStep;	
	
	static final  double emergDec	= emergDecMS*tStep*tStep/spaceScale;
	static final  double carLength	= carLengthM/spaceScale;
	static final  double carWidth	= carWidthM/spaceScale;
	static public double lambdaCar	= vehRho * tStep / 3600;
	static public double lambdaPed	= pedRho * tStep / 3600;
	static public double poisExpV	= Math.exp(-lambdaCar);
	static public double poisExpP	= Math.exp(-lambdaPed);
	static public double Pof1Car	= lambdaCar * poisExpV;		// Poisson probability of 1 arrival in a tick (higher counts are negligible)
	static public double Pof1Ped	= lambdaPed * poisExpP;		//  ditto for peds
	static public double Pof2Car	= Math.pow(lambdaCar,2)*poisExpV/2;
	static public double Pof2Ped	= Math.pow(lambdaPed,2)*poisExpP/2;
	static public double DmuHat		= DmuHatS - Math.log(tStep);
	static public double interDlam	= interDlamS * tStep;
	static public double SmuHat		= SmuHatM - Math.log(RoadBuilder.spaceScale);
	static public int	 cycleTime  = (int)(cycleTimeS / tStep);
	static public int	 greenDur	= (int)(greenDurS / tStep);
	static public int	 amberDur	= (int)(amberDurS / tStep);
	static public int	 redDur		= (int)(redDurS / tStep);
	static public int	 calcTSpan	= (int)(calcTSpanS / tStep);
	static public double hPercLim	= hPercLimM/spaceScale;
	static public double aPercLim	= aPercLimM/spaceScale;
//	static public ArrayList<Double> poisStoreV, poisStoreP;
	//TODO: place agents created in later terms of Poisson approximation
	
	// declare parameters of error-making
	static final  double  Vs		= 0.1;		// relative standard deviation of headEst from head
	static final  double  errPers	= 20;		// persistence time of estimation errors in seconds
	static final  double  wien1		= Math.exp(-tStep/errPers);		// constants in the calculation
	static final  double  wien2		= Math.sqrt(2*tStep/errPers);	//	of perception errors
	
	// max/min vals for generated CF distributions (somewhat arbitrary)
	static final double maxHeadT	= 3.294 / tStep;
	static final double minHeadT	= 0.252 / tStep;
	static final double maxJamHead	= 4.476 / spaceScale;
	static final double minJamHead	= 0.444 / spaceScale;
	static final double maxMaxA		= 5.454 * tStep * tStep / spaceScale;
	static final double minMaxA		= 0.394 * tStep * tStep / spaceScale;
	static final double maxMinA		= emergDec;
	static final double minMinA		= 0.376 * tStep * tStep / spaceScale;
	static final double maxMaxV		= sLimitMu + 4*sLimit_sd;
	static final double minMaxV		= sLimitMu - 4*sLimit_sd;
	
	// convert initial values to strings for JPanel
	private String sLimits	= String.valueOf(sLimitKH);
	private String vehRhos	= String.valueOf(vehRho);
	private String pedRhos	= String.valueOf(pedRho);
	private JTextField pRho;
//	private String delTs	= String.valueOf(delayTs);
//	private String tSteps	= String.valueOf(tStep);
	private String confTs	= String.valueOf(confLimS);
	private String pV2Xs;
//	private String pV2Is	= String.valueOf(percV2I);
	private String pAutoS;
	private String pBothS;
	
	/*
	 * Builds the GUI user panel
	 */
	public UserPanel(RoadBuilder src) {
		percV2X = src.percV2X;
		percAuto = src.percAuto;
		percBoth = src.percBoth;
		V2Xlo	= 0;
		V2Xhi	= percV2X/100;
		autLo	= V2Xhi;
		autHi	= autLo + percAuto/100;
		bothLo	= autHi;
		bothHi = bothLo + percBoth/100;
		pV2Xs	= String.valueOf(percV2X);
		pAutoS	= String.valueOf(percAuto);
		pBothS	= String.valueOf(percBoth);
		
		JPanel newPanel = new JPanel();
		if(percV2X + percAuto + percBoth > 100.001) {
			deathKnell = true;}
		JButton carBtn = new JButton("car");
		carBtn.addActionListener(this);
		newPanel.add(carBtn);
		
		JCheckBox rlOn	  = new JCheckBox("Red Lights?",	inclRL);
		JCheckBox funDia  = new JCheckBox("Calc funDia?",	calcFun);
		JCheckBox errOn   = new JCheckBox("Est Errors?",	estErr);
		JCheckBox brtOn   = new JCheckBox("BRT?",			BRT);
		JCheckBox adrtOn  = new JCheckBox("ADRT?",			ADRT);
//		JCheckBox iidmOn  = new JCheckBox("IIDM?",			IIDM);
//		JCheckBox car2way = new JCheckBox("Cars both dir?", bothCar);
//		JCheckBox pedUp   = new JCheckBox("Peds up?",		pedsUp);
//		JCheckBox pedDown = new JCheckBox("Peds down?",		pedsDn);
		
		JLabel   sLabel = new JLabel("Speed limit (km/hr)");
		JTextField sLim = new JTextField(sLimits, 6);
			sLim.setActionCommand("sLim");
		JLabel   vLabel = new JLabel("Vehicles/hr");
		JTextField vRho = new JTextField(vehRhos, 6);
			vRho.setActionCommand("vRho");
		JLabel   pLabel = new JLabel("Pedestrians/hr");
		pRho = new JTextField(pedRhos, 6);
			pRho.setActionCommand("pRho");
//		JLabel   tLabel = new JLabel("Reaction time (sec)");
//		JTextField delT = new JTextField(delTs,   6);
//			delT.setActionCommand("delT");
		JLabel	 cLabel = new JLabel("Conflict limit (sec)");
		JTextField conf = new JTextField(confTs,  6);
			conf.setActionCommand("conf");
		JLabel	 v2xLab = new JLabel("Percent V2X");
		JTextField v2xF = new JTextField(pV2Xs,   3);
			v2xF.setActionCommand("V2X");
//		JLabel	 v2iLab = new JLabel("Percent V2I");
//		JTextField v2iF = new JTextField(pV2Is,   3);
//			v2iF.setActionCommand("V2I");
		JLabel	 autLab = new JLabel("Percent automated");
		JTextField autF = new JTextField(pAutoS,  3);
			autF.setActionCommand("Automated");
		JLabel	 bthLab = new JLabel("Percent CAV");
		JTextField bthF = new JTextField(pBothS,  3);
			bthF.setActionCommand("CAV");
//		JLabel   tckLab = new JLabel("Tick duration in sec:");
//		JTextField tckT = new JTextField(tSteps,  6);
//			tckT.setActionCommand("tckT");
		
		funDia.addActionListener(this);
		rlOn.addActionListener(this);
		errOn.addActionListener(this);
		brtOn.addActionListener(this);
		adrtOn.addActionListener(this);
		
//		iidmOn.addActionListener(this);
//		car2way.addActionListener(this);
//		pedUp.addActionListener(this);
//		pedDown.addActionListener(this);
		sLim.addActionListener(this);
		pRho.addActionListener(this);
		vRho.addActionListener(this);
//		delT.addActionListener(this);
		conf.addActionListener(this);
//		tckT.addActionListener(this);
		v2xF.addActionListener(this);
//		v2iF.addActionListener(this);
		autF.addActionListener(this);
		bthF.addActionListener(this);
		
		newPanel.add(funDia);
		newPanel.add(rlOn);
		newPanel.add(errOn);
		newPanel.add(brtOn);
		newPanel.add(adrtOn);
//		newPanel.add(iidmOn);
//		newPanel.add(car2way);
//		newPanel.add(pedUp);
//		newPanel.add(pedDown);
		newPanel.add(sLabel);
		newPanel.add(sLim);
		newPanel.add(vLabel);
		newPanel.add(vRho);
		newPanel.add(pLabel);
		newPanel.add(pRho);
//		newPanel.add(tLabel);
//		newPanel.add(delT);
//		newPanel.add(tckLab);
//		newPanel.add(tckT);
		newPanel.add(cLabel);
		newPanel.add(conf);
		newPanel.add(v2xLab);
		newPanel.add(v2xF);
//		newPanel.add(v2iLab);
//		newPanel.add(v2iF);
		newPanel.add(autLab);
		newPanel.add(autF);
		newPanel.add(bthLab);
		newPanel.add(bthF);

		//TODO: figure out how to change appearance
//		newPanel.setSize(50,100);
		if (!RunEnvironment.getInstance().isBatch()) {
			RSApplication.getRSApplicationInstance().addCustomUserPanel(newPanel);}
	}

	public void actionPerformed(ActionEvent ae) {
		JComponent which = (JComponent)ae.getSource();
		String type = which.getUIClassID();
		String name = ae.getActionCommand();
		switch (type) {
		case "ButtonUI":
			JButton btnSource = (JButton)which;
			RoadBuilder.flowSource.forceCar();
			break;
		case "CheckBoxUI":
			JCheckBox checkSource = (JCheckBox)which;
			switch (name) {
			case "Calc funDia?":
				if (checkSource.isSelected()) {
					calcFun = true;
					bothCar = false;
					pedRho = 100;
					calcPeds();}
				else {
					calcFun = false;
					bothCar = true;
					JTextField pedInput = pRho;
					pedRho = Integer.parseInt(pRho.getText());}
				break;
			case "Red Lights?":
				if (checkSource.isSelected()) inclRL = true;
				else inclRL = false;
				break;
			case "Est Errors?":
				if (checkSource.isSelected()) estErr = true;
				else estErr = false;
				break;
			case "BRT?":
				if (checkSource.isSelected()) BRT = true;
				else BRT = false;
				break;
			case "ADRT?":
				if (checkSource.isSelected()) ADRT = true;
				else ADRT = false;
				break;
//			case "IIDM?":
//				if (checkSource.isSelected()) IIDM = true;
//				else IIDM = false;
//				break;
//			case "Cars both dir?":
//				if (checkSource.isSelected()) bothCar = true;
//				else bothCar = false;
//				break;
//			case "Peds up?":
//				if (checkSource.isSelected()) pedsUp = true;
//				else pedsUp = false;
//				break;
//			case "Peds down?":
//				if (checkSource.isSelected()) pedsDn = true;
//				else pedsDn = false;
//				break;
			default: break;}
			break;
		case "TextFieldUI":
			JTextField textSource = (JTextField)which;
			switch (name) {
			case "sLim":
				String newSVal = textSource.getText();
				sLimitKH = Integer.parseInt(newSVal);
				sLimit = sLimitKH/vBase;
				break;
			case "vRho":
				String newVVal = textSource.getText();
				vehRho = Integer.parseInt(newVVal);
				calcCars();
				break;
			case "pRho":
				String newPVal = textSource.getText();
				pedRho = Integer.parseInt(newPVal);
				calcPeds();
				break;
//			case "delT":
//				String newTVal = textSource.getText();
//				delayTs = Double.parseDouble(newTVal);
//				break;
//			case "tckT":
//				String newTick = textSource.getText();
//				tStep = Double.parseDouble(newTick);
//				calcCars();
//				calcPeds();
//				break;
			case "conf":
				String newConfLim = textSource.getText();
				confLimS = Double.parseDouble(newConfLim);
				break;
			case "V2X":
				String newPv2x = textSource.getText();
				percV2X = Double.parseDouble(newPv2x);
				calcCars();
				break;
//			case "V2I":
//				String newPv2i = textSource.getText();
//				percV2I = Double.parseDouble(newPv2i);
//				break;
			case "Automated":
				String newPauto = textSource.getText();
				percAuto = Double.parseDouble(newPauto);
				calcCars();
				break;
			case "CAV":
				String newPboth = textSource.getText();
				percBoth = Double.parseDouble(newPboth);
				calcCars();
				break;}
			break;
		default: break;}
	}
	
	/**
	 * Calculates new values for car creation
	 */
	public void calcCars(){
		lambdaCar = vehRho * tStep / 3600;		// veh arrivals per tick
		poisExpV  = Math.exp(-lambdaCar);
		Pof1Car = lambdaCar * poisExpV;
		Pof2Car	= Math.pow(lambdaCar,2)*poisExpV/2;
		V2Xlo	= 0;
		V2Xhi	= percV2X/100;
		autLo	= V2Xhi;
		autHi	= autLo + percAuto/100;
		bothLo	= autHi;
		bothHi  = bothLo + percBoth/100;
//		poisStoreV = new ArrayList<Double>();
//		double pNew = 1;
//		int i = 1;
//		while (pNew > 1e-9) {
//			pNew = Math.pow(lambdaCar,i+1)*poisExpV/factorial(i+1);
//			poisStoreV.add(pNew);}
	}
	
	/**
	 * Calculates new values for ped creation
	 */
	public void calcPeds(){
		lambdaPed = pedRho * tStep / 3600;		// ped arrivals per tick
		poisExpP  = Math.exp(-lambdaPed);
		Pof1Ped = lambdaPed * poisExpP;
		Pof2Ped	= Math.pow(lambdaPed,2)*poisExpP/2;
		
//		poisStoreP = new ArrayList<Double>();
//		double pNew = 1;
//		int i = 1;
//		while (pNew > 1e-9) {
//			pNew = Math.pow(lambdaPed,i+1)*poisExpP/factorial(i+1);
//			poisStoreP.add(pNew);}
	}
	/**
	 * Returns factorial value for small integers
	 * @param n
	 * @return
	 * author: The Guava Authors, (C) 2011
	 */
	public static int factorial(int n) {
	    return (n < factorials.length) ? factorials[n] : Integer.MAX_VALUE;}

	private static final int[] factorials = {
		1,
		1,
		1 * 2,
		1 * 2 * 3,
		1 * 2 * 3 * 4,
		1 * 2 * 3 * 4 * 5,
		1 * 2 * 3 * 4 * 5 * 6,
		1 * 2 * 3 * 4 * 5 * 6 * 7,
		1 * 2 * 3 * 4 * 5 * 6 * 7 * 8,
		1 * 2 * 3 * 4 * 5 * 6 * 7 * 8 * 9,
		1 * 2 * 3 * 4 * 5 * 6 * 7 * 8 * 9 * 10,
		1 * 2 * 3 * 4 * 5 * 6 * 7 * 8 * 9 * 10 * 11,
		1 * 2 * 3 * 4 * 5 * 6 * 7 * 8 * 9 * 10 * 11 * 12};

	@Parameter(usageName="sLimit", displayName="SpeedLimit (kph)")
	public double getLimit() {
		return sLimitKH;}
	public void setLimit(double limit) {
		sLimitKH = limit;}
	@Parameter(usageName="vehDens", displayName="Vehicles per hour")
	public int getVehRho() {
		return vehRho;}
	public void setVehRho(int vRho) {
		vehRho = vRho;}
	@Parameter(usageName="pedDens", displayName="Peds per hour")
	public int getPedRho() {
		return pedRho;}
	public void setPedRho(int pRho) {
		pedRho = pRho;}
	@Parameter(usageName="percV2X",displayName="Percent Connected")
	public double getPercV2X() {
		return percV2X;}
	public void setPercV2X(double pv2x) {
		percV2X = pv2x;}
	@Parameter(usageName="percAuto",displayName="Percent Autonomous")
	public double getPercAuto() {
		return percAuto;}
	public void setPercAuto(double pAuto) {
		percAuto = pAuto;}
	@Parameter(usageName="percBoth",displayName="Percent Both")
	public double getPercBoth() {
		return percBoth;}
	public void setPercBoth(double pBoth) {
		percBoth = pBoth;}
	/*
	@Parameter(usageName="",displayName="")
	public double get() {
		return ;}
	public void set() {
		 = ;}
	*/
}

//double[] poisStoreV = new double[poisTerms];
//double[] poisStoreP = new double[poisTerms];
//int i;
//for (i=0;i<poisTerms;i++) { 
//	poisStoreV[i] = Math.pow(lambdaCar,i+1)*poisExpV/factorial(i+1);
//	poisStoreP[i] = Math.pow(lambdaPed,i+1)*poisExpP/factorial(i+1);}