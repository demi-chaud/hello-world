package driving1;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.*;

import repast.simphony.ui.RSApplication;

/*
 * Class to implement the user panel and initialize model variables
 * author: Darryl Michaud
 */

public class UserPanel implements ActionListener{

	// declare variables in real world units
	static final  double pedVavgKH	= 5;		// km/hr			default:   5
	static final  double maxaMS		= 1.4;		// m/s2				default:   1.4
	static final  double minaMS		= 3.5;		// m/s2				default:   3.5
	static final  double tGapS		= 1.9;		// s				default:   1.9
	static final  double jamHeadM	= 2.5;		// m				default:   2.5
	static public double sLimitKH	= 45;		// km/hr			default:  45
	static public int    vehRho		= 1200;		// veh/hr each dir 	default: 600
	static public int    pedRho		= 500;		// ppl/hr each dir	default:  60		//should these by total or each (would add factor of two in calc)
	static public double delayTs 	= 0;		// seconds			default:   0.5
	
	// declare model parameters
	static final  double spaceScale	= RoadBuilder.spaceScale;
	static public double tStep		= 0.05;		// duration of one tick in seconds	
	static final  double vBase 		= (spaceScale/tStep)*3600/1000;
		// vBase is the natural speed of the model (one unit per tick), converted to km/hr
	
	// convert variables to model units
	static final  double pedVavg	= pedVavgKH/vBase;
	static final  double maxa		= maxaMS*tStep/spaceScale;	// not sure why this doesn't have another
	static final  double mina		= minaMS*tStep/spaceScale;	//	factor of tStep
	static final  double tGap		= tGapS/tStep;
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
//	static public ArrayList<Double> poisStoreV, poisStoreP;
	//TODO: place agents created in later terms of Poisson approximation
	
	static public boolean bothCar	= true;
//	static public boolean bothPed	= false;
	static public boolean pedsUp	= true;
	static public boolean pedsDn	= true;
	
	
	// declare parameters of error-making
	static public boolean estErr 	= true;		// estimation errors
	static final  double  Vs		= 0.1;		// relative standard deviation of headEst from head
	static final  double  errPers	= 20;		// persistence time of estimation errors in seconds
	static final  double  wien1		= Math.exp(-tStep/errPers);		// constants in the calculation
	static final  double  wien2		= Math.sqrt(2*tStep/errPers);	//	of perception errors
	
	// convert initial values to strings for JPanel
	private String sLimits = String.valueOf(sLimitKH);
	private String vehRhos = String.valueOf(vehRho);
	private String pedRhos = String.valueOf(pedRho);
	private String delTs   = String.valueOf(delayTs);
	private String tSteps  = String.valueOf(tStep);
	
	/*
	 * Builds the GUI user panel
	 */
	public UserPanel() {
		JPanel newPanel = new JPanel();
		
		JCheckBox errOn   = new JCheckBox("Estimation Errors?", true);
		JCheckBox car2way = new JCheckBox("Cars both dir?", true);
//		JCheckBox ped2way = new JCheckBox("Peds both dir?", false);
		JCheckBox pedUp   = new JCheckBox("Peds up?",		true);
		JCheckBox pedDown = new JCheckBox("Peds down?",		true);
		
		JLabel   sLabel = new JLabel("Speed limit in km/hr:");
		JTextField sLim = new JTextField(sLimits, 4);
			sLim.setActionCommand("sLim");
		JLabel   vLabel = new JLabel("Vehicles per hour:");
		JTextField vRho = new JTextField(vehRhos, 4);
			vRho.setActionCommand("vRho");
		JLabel   pLabel = new JLabel("Pedestrians per hour:");
		JTextField pRho = new JTextField(pedRhos, 4);
			pRho.setActionCommand("pRho");
		JLabel   tLabel = new JLabel("Reaction time in sec:");
		JTextField delT = new JTextField(delTs,   6);
			delT.setActionCommand("delT");
		JLabel   tckLab = new JLabel("Tick duration in sec:");
		JTextField tckT = new JTextField(tSteps,  5);
			tckT.setActionCommand("tckT");
		
		errOn.addActionListener(this);
		car2way.addActionListener(this);
//		ped2way.addActionListener(this);
		pedUp.addActionListener(this);
		pedDown.addActionListener(this);
		sLim.addActionListener(this);
		pRho.addActionListener(this);
		vRho.addActionListener(this);
		delT.addActionListener(this);
		tckT.addActionListener(this);
		
		newPanel.add(errOn);
		newPanel.add(car2way);
//		newPanel.add(ped2way);
		newPanel.add(pedUp);
		newPanel.add(pedDown);
		newPanel.add(sLabel);
		newPanel.add(sLim);
		newPanel.add(vLabel);
		newPanel.add(vRho);
		newPanel.add(pLabel);
		newPanel.add(pRho);
		newPanel.add(tLabel);
		newPanel.add(delT);
		newPanel.add(tckLab);
		newPanel.add(tckT);

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
			case "Estimation Errors?":
				if (checkSource.isSelected()) estErr = true;
				else estErr = false;
				break;
			case "Cars both dir?":
				if (checkSource.isSelected()) bothCar = true;
				else bothCar = false;
				break;
//			case "Peds both dir?":
//				if (checkSource.isSelected()) bothPed = true;
//				else bothPed = false;
//				break;
			case "Peds up?":
				if (checkSource.isSelected()) pedsUp = true;
				else pedsUp = false;
				break;
			case "Peds down?":
				if (checkSource.isSelected()) pedsDn = true;
				else pedsDn = false;
				break;
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
			case "tckT":
				String newTick = textSource.getText();
				tStep = Double.parseDouble(newTick);
				calcCars();
				calcPeds();
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
}

//double[] poisStoreV = new double[poisTerms];
//double[] poisStoreP = new double[poisTerms];
//int i;
//for (i=0;i<poisTerms;i++) { 
//	poisStoreV[i] = Math.pow(lambdaCar,i+1)*poisExpV/factorial(i+1);
//	poisStoreP[i] = Math.pow(lambdaPed,i+1)*poisExpP/factorial(i+1);}