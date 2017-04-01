package driving1;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.*;

import repast.simphony.parameter.Parameter;
import repast.simphony.ui.RSApplication;

/*
 * Class to implement the user panel and initialize model variables
 * author: Darryl Michaud
 */

public class UserPanel implements ActionListener{

	// declare model parameters
	static final  double spaceScale	= RoadBuilder.spaceScale;
	static public double tStep		= 0.05;		// duration of one tick in seconds	
	static final  double vBase 		= (spaceScale/tStep)*3600/1000;
						// vBase is the natural speed of the model (one cell per tick), converted to km/hr
	static final  double simLength  = 5*60*60/tStep;	//in ticks (currently 5hrs)
	static public double percV2X	= 0;
	static public double percAuto	= 0;
	static public double percBoth	= 50;
	
	// calculate population range constants
	static public double V2Xlo	= 0;
	static public double V2Xhi	= percV2X/100;
	static public double autLo	= V2Xhi;
	static public double autHi	= autLo + percAuto/100;
	static public double bothLo	= autHi;
	static public double bothHi = bothLo + percBoth/100;
	
	// declare variables in real world units
	static final  double pedVavgKH	= 5;			// km/hr			default:   5 (source Zebala 2012)
	static final  double maxaMS		= 1.4;			// m/s2				default:   1.4
	static final  double minaMS		= 3.5;			// m/s2				default:   3.5
	static final  double emergDecMS	= 4.5;			// m/s2				default:   4.5
	static final  double tGapS		= 1.9;			// s				default:   1.9
	static final  double carLengthM	= 5.28;			// m			source: http://usatoday30.usatoday.com/money/autos/2007-07-15-little-big-cars_N.htm
	static final  double carWidthM	= 1.89;			// m					avg of lg sedan 1990 & 2007
	static final  double jamHeadM	= 2.5;			// m				default:   2.5
	static public double sLimitKH	= 45;			// km/hr			default:  45
	static public int    vehRho		= 400;			// veh/hr each dir 	default: 600
	static public int    pedRho		= 100;			// ppl/hr each dir	default:  60		//should these by total or each (would add factor of two in calc)
	static public double delayTs 	= 0.5;			// seconds			default:   0.5
	static public double confLimS	= 1.7;			// seconds			default:   1.7		//kinda arbitrary
	static public double DmuHatS	= -0.4552452;	// distraction dist scale param (in seconds)
	static public double DsigHat	= 0.6108071;	// distraction dist shape param (already in model units)
	static public double interDlamS	= 0.3524525;	// interdistraction rate (in seconds^-1)
	//TODO: add ped gap coefficients
	
	// convert variables to model units
	static final  double pedVavg	= pedVavgKH/vBase;
	static final  double maxa		= maxaMS*tStep*tStep/spaceScale;
	static final  double mina		= minaMS*tStep*tStep/spaceScale;
	static final  double emergDec	= emergDecMS*tStep*tStep/spaceScale;
	static final  double tGap		= tGapS/tStep;
	static final  double carLength	= carLengthM/spaceScale;
	static final  double carWidth	= carWidthM/spaceScale;
	static final  double jamHead	= jamHeadM/spaceScale;
	static public double sLimit 	= sLimitKH/vBase;
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
//	static public ArrayList<Double> poisStoreV, poisStoreP;
	//TODO: place agents created in later terms of Poisson approximation
	
	static public boolean bothCar	= true;
	static public boolean pedsUp	= true;
	static public boolean pedsDn	= true;
	static public boolean IIDM		= true; //include Improved IDM?
	
	// declare parameters of error-making
	static public boolean estErr 	= true;		// estimation errors
	static final  double  Vs		= 0.1;		// relative standard deviation of headEst from head
	static final  double  errPers	= 20;		// persistence time of estimation errors in seconds
	static final  double  wien1		= Math.exp(-tStep/errPers);		// constants in the calculation
	static final  double  wien2		= Math.sqrt(2*tStep/errPers);	//	of perception errors
	
	// convert initial values to strings for JPanel
	private String sLimits	= String.valueOf(sLimitKH);
	private String vehRhos	= String.valueOf(vehRho);
	private String pedRhos	= String.valueOf(pedRho);
	private String delTs	= String.valueOf(delayTs);
//	private String tSteps	= String.valueOf(tStep);
	private String confTs	= String.valueOf(confLimS);
	private String pV2Xs	= String.valueOf(percV2X);
//	private String pV2Is	= String.valueOf(percV2I);
	private String pAutoS	= String.valueOf(percAuto);
	private String pBothS	= String.valueOf(percBoth);
	
	/*
	 * Builds the GUI user panel
	 */
	public UserPanel() {
		JPanel newPanel = new JPanel();
		
		JCheckBox errOn   = new JCheckBox("Est Errors?",	true);
//		JCheckBox iidmOn  = new JCheckBox("IIDM?",			true);
//		JCheckBox car2way = new JCheckBox("Cars both dir?", true);
//		JCheckBox pedUp   = new JCheckBox("Peds up?",		true);
//		JCheckBox pedDown = new JCheckBox("Peds down?",		true);
		
		JLabel   sLabel = new JLabel("Speed limit (km/hr)");
		JTextField sLim = new JTextField(sLimits, 6);
			sLim.setActionCommand("sLim");
		JLabel   vLabel = new JLabel("Vehicles/hr");
		JTextField vRho = new JTextField(vehRhos, 6);
			vRho.setActionCommand("vRho");
		JLabel   pLabel = new JLabel("Pedestrians/hr");
		JTextField pRho = new JTextField(pedRhos, 6);
			pRho.setActionCommand("pRho");
		JLabel   tLabel = new JLabel("Reaction time (sec)");
		JTextField delT = new JTextField(delTs,   6);
			delT.setActionCommand("delT");
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
		
		errOn.addActionListener(this);
//		iidmOn.addActionListener(this);
//		car2way.addActionListener(this);
//		pedUp.addActionListener(this);
//		pedDown.addActionListener(this);
		sLim.addActionListener(this);
		pRho.addActionListener(this);
		vRho.addActionListener(this);
		delT.addActionListener(this);
		conf.addActionListener(this);
//		tckT.addActionListener(this);
		v2xF.addActionListener(this);
//		v2iF.addActionListener(this);
		autF.addActionListener(this);
		bthF.addActionListener(this);
		
		newPanel.add(errOn);
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
		newPanel.add(tLabel);
		newPanel.add(delT);
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
		RSApplication.getRSApplicationInstance().addCustomUserPanel(newPanel);	
	}

	public void actionPerformed(ActionEvent ae) {
		JComponent which = (JComponent)ae.getSource();
		String type = which.getUIClassID();
		String name = ae.getActionCommand();
		switch (type) {
		case "CheckBoxUI":
			JCheckBox checkSource = (JCheckBox)which;
			switch (name) {
			case "Est Errors?":
				if (checkSource.isSelected()) estErr = true;
				else estErr = false;
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
			case "delT":
				String newTVal = textSource.getText();
				delayTs = Double.parseDouble(newTVal);
				break;
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
				break;
//			case "V2I":
//				String newPv2i = textSource.getText();
//				percV2I = Double.parseDouble(newPv2i);
//				break;
			case "Automated":
				String newPauto = textSource.getText();
				percAuto = Double.parseDouble(newPauto);
				break;
			case "CAV":
				String newPboth = textSource.getText();
				percBoth = Double.parseDouble(newPboth);
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
	@Parameter(usageName="delayT", displayName="Delay time (s)")
	public double get() {
		return delayTs;}
	public void set(double delayTime) {
		delayTs = delayTime;}
}

//double[] poisStoreV = new double[poisTerms];
//double[] poisStoreP = new double[poisTerms];
//int i;
//for (i=0;i<poisTerms;i++) { 
//	poisStoreV[i] = Math.pow(lambdaCar,i+1)*poisExpV/factorial(i+1);
//	poisStoreP[i] = Math.pow(lambdaPed,i+1)*poisExpP/factorial(i+1);}