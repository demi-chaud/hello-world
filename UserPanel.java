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
	static final  double pedVavgKH	= 5;		// km/hr		default:   5
	static final  double maxaMS		= 1.4;		// m/s2			default:   1.4
	static final  double minaMS		= 3.5;		// m/s2			default:   3.5
	static final  double tGapS		= 1.9;		// s			default:   1.9
	static final  double jamHeadM	= 2.5;		// m			default:   2.5
	static public double sLimitKH	= 45;		// km/hr		default:  45
	static public int    vehRho		= 200;		// veh/hr       default: 600
	static public int    pedRho		= 200;		// ppl/hr       default:  60
	static public double delayTs 	= 0.5;		// seconds      default:   0.5
	
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
	//TODO: calculate more terms in Poisson approximation (attempted beginning at bottom)
	static public double Pof1Car	= lambdaCar * poisExpV;		// Poisson probability of 1 arrival in a tick (higher counts are negligible)
	static public double Pof1Ped	= lambdaPed * poisExpP;		//  ditto for peds
	
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
		
		JCheckBox errOn = new JCheckBox("Estimation Errors?", true);
		
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
		sLim.addActionListener(this);
		pRho.addActionListener(this);
		vRho.addActionListener(this);
		delT.addActionListener(this);
		tckT.addActionListener(this);
		
		newPanel.add(errOn);
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
	}
	
	/**
	 * Calculates new values for ped creation
	 */
	public void calcPeds(){
		lambdaPed = pedRho * tStep / 3600;		// ped arrivals per tick
		poisExpP  = Math.exp(-lambdaPed);
		Pof1Ped = lambdaPed * poisExpP;
	}
}


/*int poisTerms = 5;
double poisThisP, poisThisV;
double[] poisStoreV = new double[poisTerms];
double[] poisStoreP = new double[poisTerms];
int i;
for (i=0;i<poisTerms;i++) {  //TODO: 5 is arbitrary here, make more general by setting the size of result as loop stop criterion
									//can switch to vector<> and .add to make it grow
	poisStoreV[i] = Math.pow(lambdaCar,i+1)*poisExpV/factorial(i+1);
	poisStoreP[i] = Math.pow(lambdaPed,i+1)*poisExpP/factorial(i+1);}*/

