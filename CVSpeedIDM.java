import java.awt.EventQueue;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.Timer;

//import org.apache.commons.math3.distribution.*;

public class CVSpeedIDM {
	
	// TODO: add any global variable you need here.	
	// Car following parameters
	double amax = Parameter.amax; //maximum acceleration 0.73
	double ddes = Parameter.ddes; // desired deceleration
	double delta = Parameter.delta; // acceleration exponent
	double minSpacing = Parameter.minSpacing; //minimum spacing
	double s0 = minSpacing; // jam distance;
	double vdes = Parameter.vdes; // desired speed m/s
	double TReact = Parameter.TReact; // driver reaction time
	double vf = Parameter.vf; // speed
	double length = Parameter.length; // car length
	
	double zoneArrive = Parameter.zoneArrive; //  length of approach: from just arriving vehile to stopline
	double gmax = Parameter.gmax; // maximum green duration(s)
	double gmin = Parameter.gmin; // minimum green duration(s)
	double followGap = Parameter.followGap; // m
	double speedVar = Parameter.speedVar;
	
	// intersection configuration parameters
	double zoneLength = Parameter.zoneLength; // communication range
	double loopLocation = Parameter.loopLocation; // loop for actuated control
	boolean visual = true;
	double lenIS = Parameter.lenIS; // length of intersection
	
	// parameter for algorithms
	double [] leadSpeed = new double[4]; // for four approach lanes
	boolean calibrate = Parameter.calibrate;
	int lastPlatoon1 = 0; // default
	int lastPlatoon2 = 0; // default lastPlatoon1 and lastPlatoon2, they are in the same phase but in different lanes: opposite lanes
	int lastApproach = 0; // default
	double lastDepTime1 = 0.0; // default
	double lastDepTime2 = 0.0; // default lastDepTime1 and lastDepTime2, they are in the same phase
	static int signalPlan = 1; // 1 means fixed time; 2 means CV control : 
	double actuatedGap =  5;
	int sigMandatory = 0;
	double pretime=0;
	double preTb=0.0;
	
	ArrayList<Integer> ArrivalList1 =new ArrayList<Integer>();
	ArrayList<Integer> ArrivalList2 =new ArrayList<Integer>();
	ArrayList<Integer> ArrivalList3 =new ArrayList<Integer>();
	ArrayList<Integer> ArrivalList4 =new ArrayList<Integer>();
	ArrayList<Integer> ArrivalList =new ArrayList<Integer>();
	
	ArrayList<Boolean> equipped = new ArrayList<Boolean>();
	ArrayList<Boolean> automated = new ArrayList<Boolean>();
	ArrayList<Boolean> revealed = new ArrayList<Boolean>();
	ArrayList<Double> Arrivals = new ArrayList<Double>();
	ArrayList<signalLight> Signals = new ArrayList<signalLight>();
	private static final Random random = new Random();
	
	signalLight signal = new signalLight(); //  note that if make this be static variable, it will affect the batch of process.
	static ArrayList<Integer> DepartureList = new ArrayList<Integer>(); //called in Lane Class
	ArrayList<Integer> ApproachList = new ArrayList<Integer>();
	
	// parameter for evaluation and storage
	int progid;// index of test
	double pr_auto; // auto percentage in connected vehicles
	double pr_con; // connected vehicles percentage in total vehicles
	
	double vehflow; // 1000.0 veh/h;
	double totalvehFlow; // 1000.0/3600 veh/s;
	double vehflowRatio; //  1.0: ratio of vehflow on phase 1 and phase 2;
	
	double pedflow; // 1000.0 ped/h;
	double totalpedFlow; // 1000.0/3600 ped/s;
	double pedflowRatio; //  1.0: ratio of pedflow on phase 1 and phase 2;;
	
	double triggerLocation; 
	int timecount = 0;
	File arrivalfile, arrivalpedfile, resultfile, resultpedfile, trajfile, syntheticResults, resultsigfile;   
    static FileWriter arrivalwriter;
    static FileWriter arrivalpedwriter;
	static FileWriter resultwriter;
	static FileWriter resultpedwriter;
	static FileWriter resultsigwriter;
	
	static FileWriter realwriter;
	static FileWriter controlwriter;
	static FileWriter trajwriter;
    FileReader arrivalreader, arrivalpedreader, resultreader;	
    java.time.LocalTime ntime1 = java.time.LocalTime.now();
     
    ArrayList<Double> sigSwitchTime = new ArrayList<Double>();
    ArrayList<Double> FDWSwitchTime = new ArrayList<Double>();
    // parameter for simulation
    int c = Parameter.c;	
	int timeStep=Parameter.timeStep;	
	private final Timer timer = new Timer(5, null);
	Lane lane1, lane2, lane3, lane4; 
	roadPanel road; 
	CVSPeedPanel panel;
	
	
	// parameter for pedestrian
	double G_pedmin = Parameter.G_pedmin;
	double Tpc = Parameter.Tpc;
	boolean flagFDWSwitch=false;
	boolean flagFDW=false; // in actuated control
	boolean flagFirst=false; // to store the fisrt time of FDW.
	ArrayList<Double> preTimeFDW = new ArrayList<Double>();
	int inextPhase = 0;
	int pre_pedevent = 0;
	
	ArrayList<Double> WaitListPed1 =new ArrayList<Double>(); //the time when pedestrians arrive and wait.
	ArrayList<Double> WaitListPed2 =new ArrayList<Double>();
	ArrayList<Double> WaitListPed3 =new ArrayList<Double>(); 
	ArrayList<Double> WaitListPed4 =new ArrayList<Double>();
	
	ArrayList<Double> sigstartTimePed1 =new ArrayList<Double>(); // pedestrian green signal start time on approach 1
	ArrayList<Double> sigstartTimePed2 =new ArrayList<Double>();
	ArrayList<Double> sigstartTime =new ArrayList<Double>();
	
	ArrayList<Double> ArrivalTimePed1_1 =new ArrayList<Double>();// pedestrian arrive time on the sidewalk 1 of approach 1.
	ArrayList<Double> ArrivalTimePed1_2 =new ArrayList<Double>();
	ArrayList<Double> ArrivalTimePed2_1 =new ArrayList<Double>();
	ArrayList<Double> ArrivalTimePed2_2 =new ArrayList<Double>();
	ArrayList<Double> ArrivalTimePed3_1 =new ArrayList<Double>();
	ArrayList<Double> ArrivalTimePed3_2 =new ArrayList<Double>();
	ArrayList<Double> ArrivalTimePed4_1 =new ArrayList<Double>();
	ArrayList<Double> ArrivalTimePed4_2 =new ArrayList<Double>();
	
	ArrayList<Double> DepTimePed1_1 =new ArrayList<Double>();// pedestrian departure time on the sidewalk 1 of approach 1.
	ArrayList<Double> DepTimePed1_2 =new ArrayList<Double>();
	ArrayList<Double> DepTimePed2_1 =new ArrayList<Double>();
	ArrayList<Double> DepTimePed2_2 =new ArrayList<Double>();
	ArrayList<Double> DepTimePed3_1 =new ArrayList<Double>();
	ArrayList<Double> DepTimePed3_2 =new ArrayList<Double>();
	ArrayList<Double> DepTimePed4_1 =new ArrayList<Double>();
	ArrayList<Double> DepTimePed4_2 =new ArrayList<Double>();
	
	int count1_1, count1_2, count2_1, count2_2, count3_1, count3_2, count4_1, count4_2; // arrival pedestrians
	int waitN1_1, waitN1_2, waitN2_1, waitN2_2, waitN3_1, waitN3_2, waitN4_1, waitN4_2; // waiting pedestrians
	
	ArrayList<Integer> waitN =new ArrayList<Integer>(); // total waiting pedestrians
	ArrayList<Integer> waitN1 =new ArrayList<Integer>();
	ArrayList<Integer> waitN2 =new ArrayList<Integer>();
	ArrayList<Integer> waitN3 =new ArrayList<Integer>();
	ArrayList<Integer> waitN4 =new ArrayList<Integer>();
	
	boolean arrivalPed1_1 = false; // judge pedestrian arrival or not
	boolean arrivalPed1_2 = false;
	boolean arrivalPed2_1 = false;
	boolean arrivalPed2_2 = false;
	boolean arrivalPed3_1 = false;
	boolean arrivalPed3_2 = false;
	boolean arrivalPed4_1 = false;
	boolean arrivalPed4_2 = false;
	
	// pedestrian delay calculation period
	double ta = 0.0;// start time of triggered event
	double tb = 0.0; // end time of last vehicle departure
	double Tb= 0.0; //extended time of calculation period
	double lastTb = 0.0; // last extended time in previous period
	double tpc = 0 + gmax-Tpc;  //initial value;  start time of pedestrian PC, i.e. flashing time. How to determine pedestrian signal flash time in simulation? Here we use predicted value by trajectory design.
	double tw_last=0.0; // last walking time
	int NS =0; // number of signal switch
	double Wveh =1.0; // weight for vehicle delay
	double Wped	= 0.0;	// weight for pedestrian delay
	double tpc0 = 0.0; //previous FDW starting time.
	// parameter for correlation analysis
	ArrayList<Double> timeList =new ArrayList<Double>(); // list to store time
	ArrayList<Double> taList =new ArrayList<Double>();
	ArrayList<Double> TbList =new ArrayList<Double>();
	ArrayList<Double> prepedDelayList =new ArrayList<Double>(); // list to store predicted pedestrian delay
	ArrayList<Double> prevehDelayList =new ArrayList<Double>();
	ArrayList< ArrayList<Integer>> prevehIDList =new ArrayList<ArrayList<Integer>>();
	
	double flowPed1=0.0; // pedestrian flow rate on approach 1.
	double flowPed2=0.0;
	double flowPed3=0.0;
	double flowPed4=0.0;
	
	int progidPed =0; // id for pedestrian flow scenario
	double lastVehArrival = 0.0;
	
	// parameter for ant colony system algorithm
	int numAnts = 10;// default
	int numIter = 30;// default
	float beta = 2;	
	float alpha = 0.1f;	
	float rho  = 0.1f;
	float q0 = 0.1f;
	
	
	// parameters for performance
	int out_perform=0; //performance analysis: 0 for close; 1 for test; 2 for average delay and stops for full AVs; 3 for penetration rates of CAV_information level; 4 for CAV_ automation level.
	int out_cor = 0; //correlation analysis:  0 for close; 1 for start.
	int out_com = 0;//comparison of computation time between ACS and XX:  0 for close; 1 for start.
	int out_sen = 0; // sensitive analysis of ACS: 0 for close; 1 for e0; 2 for alpha & rho; 3 for beta.
	
	// simulation time comparison
	Duration sim_time; 
	/////////////////////////////////////////////////////
	private final ActionListener listener = new ActionListener() {
        @Override
        public void actionPerformed(ActionEvent e) {   
        	
        	double currenttime = timecount * (double)timeStep/1000.0;
            timecount++;
            if(lane1.carList.size()+lane2.carList.size() + lane3.carList.size()+lane4.carList.size() ==c 
            		&& (lane1.carList.get(lane1.carList.size()-1).location>zoneLength+lenIS) && (lane2.carList.get(lane2.carList.size()-1).location>zoneLength+lenIS)
            		 && (lane3.carList.get(lane3.carList.size()-1).location>zoneLength+lenIS) && (lane4.carList.get(lane4.carList.size()-1).location>zoneLength+lenIS))	
               stop(e);
          
            loadMethod (currenttime);
            road.repaint(); 
        }
	};
	
	void algorithm()
	{
        
		while(true)
		{
			double currenttime = timecount * (double)timeStep /1000.0;        	
			timecount++; 
			
			if( lane1.carList.size()+lane2.carList.size() + lane3.carList.size()+lane4.carList.size()==c
            		&& (lane1.carList.get(lane1.carList.size()-1).location>zoneLength+lenIS) && (lane2.carList.get(lane2.carList.size()-1).location>zoneLength+lenIS)
            		 && (lane3.carList.get(lane3.carList.size()-1).location>zoneLength+lenIS) && (lane4.carList.get(lane4.carList.size()-1).location>zoneLength+lenIS))	//lane1.carList.size()+lane2.carList.size() + lane3.carList.size()+lane4.carList.size()==c
			{
				stop();
				break;
			}
			
			
			loadMethod (currenttime);
			
			if (timecount>100000)//
			{
			break;}
			
		}
	}
	
	void loadMethod (double currenttime)
	{
		// add newly arrived cars
		for(int i =0;i<c;i++)
		{

			if(Arrivals.get(i)<=currenttime && Arrivals.get(i)>currenttime-(double)timeStep /1000.0)
			{
				Car c = new Car(ArrivalList.get(i),vf, -zoneArrive, 5.0, ArrivalList.get(i)/10000, equipped.get(i), automated.get(i),i, Arrivals.get(i));
				switch (ArrivalList.get(i)/10000)
				{
				case 1:
					lane1.addCar(c);
					break;
				case 2:
					lane2.addCar(c);
					break;
				case 3:
					lane3.addCar(c);
					break;
				case 4:
					lane4.addCar(c);
					break;		
				}
			}
		}
		
		switch(signalPlan)
		{
		case 1:
			fixedControl(currenttime);
			break;
		case 2: 
			CVControl(currenttime);  
			break;
		case 3:
			givenControl(currenttime);
			break;
		case 4:
			adaptiveControl(currenttime);
			break;
		case 5:
			//ACSControl(currenttime);
			break;
		case 6:
			//BBControl(currenttime);
			break;

		default:
			fixedControl(currenttime);
			break;
		}

		lane1.carRun(currenttime);
		lane2.carRun(currenttime);
		lane3.carRun(currenttime);
		lane4.carRun(currenttime);
	}
	
	// ordinary constructor:  performance measure 1
	CVSpeedIDM(int id, double pf,  double pr, float wped, double prcon,double prauto,double tf,double dr, boolean vis, int opt, int[] outputs) 
	{
		vehflow = tf*3600.0;
		pedflow = pf*3600.0;
		totalvehFlow = tf;
    	totalpedFlow = pf;
    	vehflowRatio = dr;
    	pedflowRatio = pr;
		
    	flowPed1 = totalpedFlow*pedflowRatio/(1+pedflowRatio);
		flowPed2 = totalpedFlow/(1+pedflowRatio);
    	flowPed3 = totalpedFlow*pedflowRatio/(1+pedflowRatio);
		flowPed4 = totalpedFlow/(1+pedflowRatio);
    	
		calibrate=false;
		timecount = 0;
		visual = vis;
		progid = id;
		progidPed =((Double)(pedflow)).intValue(); 
		
		Wveh = 1-wped;
		Wped = wped;
    	pr_con = prcon;
    	pr_auto = prauto;    	
   	
    	signalPlan = opt;
    	
    	// output modes;
    	out_perform = outputs[0];
    	out_cor = outputs[1];
    	out_com = outputs[2];
    	out_sen = outputs[3];
    	
		lane1 = new Lane(1, signal);
    	lane2 = new Lane(2, signal);
    	lane3 = new Lane(3, signal);
    	lane4 = new Lane(4, signal);
    	
    	triggerLocation = Math.min(-60.0, -(minSpacing+length)/pr_con*2); // set for security distance when need to stop.
    	random.setSeed(id);
    	File arrfile;
    	String dir;
    	String dirVeh;
    	String dirPed;
    	if(!calibrate)    		
    		dir = dirname();    		
    	else
    		dir = "calibrate";
    	
    	dirVeh = dir+"/arrivalVeh"+((Integer)id).toString()+".txt";
    	arrfile = new File(dirVeh);
    	
    	if(arrfile.exists())
    		readDemand(dirVeh);
    	else
    		generateDemand(dirVeh);
    	
    	//
    	dirPed = dir+"/arrivalPed"+((Integer)id).toString()+ ".txt";
    	File arrpedfile;
    	arrpedfile = new File(dirPed);
    	
    	if(arrpedfile.exists())
    		readPedDemand(dirPed);
    	else
    		generatePedDemand(dirPed);
    	
		if(vis)
		{
			timer.addActionListener(listener);
			road = new roadPanel(lane1,lane2,lane3, lane4, signal);
			panel = new CVSPeedPanel(road);
			JFrame f = new JFrame("Revised Algorithm");
	        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	        f.add(panel);
	        f.pack();
	        f.setLocationRelativeTo(null);
	        f.setVisible(true);	        
		}
	}
	
	// comparison and correlation in ACS
	CVSpeedIDM(int id, double pf,  double pr, float wped, double prcon,double prauto,double tf,double dr, boolean vis, int opt,  int para_numAnt, int para_numIter, int[] outputs ) 
	{
		vehflow = tf*3600.0;
		pedflow = pf*3600.0;
		totalvehFlow = tf;
    	totalpedFlow = pf;
    	vehflowRatio = dr;
    	pedflowRatio = pr;
		
    	flowPed1 = totalpedFlow*pedflowRatio/(1+pedflowRatio);
		flowPed2 = totalpedFlow/(1+pedflowRatio);
    	flowPed3 = totalpedFlow*pedflowRatio/(1+pedflowRatio);
		flowPed4 = totalpedFlow/(1+pedflowRatio);
    	
		numAnts=para_numAnt;
		numIter=para_numIter;
		
		// output modes;
    	out_perform = outputs[0];
    	out_cor = outputs[1];
    	out_com = outputs[2];
    	out_sen = outputs[3];
			
		calibrate=false;
		timecount = 0;
		visual = vis;
		progid = id;
		progidPed =((Double)(pedflow)).intValue(); 
		
		Wveh = 1-wped;
		Wped = wped;
    	pr_con = prcon;
    	pr_auto = prauto;    	
   	
    	signalPlan = opt;
    	
    	
		lane1 = new Lane(1, signal);
    	lane2 = new Lane(2, signal);
    	lane3 = new Lane(3, signal);
    	lane4 = new Lane(4, signal);
    	
    	triggerLocation = Math.min(-60.0, -(minSpacing+length)/pr_con*2); // set for security distance when need to stop.
    	random.setSeed(id);
    	File arrfile;
    	String dir;
    	String dirVeh;
    	String dirPed;
    	if(!calibrate)    		
    		dir = dirname();    		
    	else
    		dir = "calibrate";
    	
    	dirVeh = dir+"/arrivalVeh"+((Integer)id).toString()+".txt";
    	arrfile = new File(dirVeh);
    	
    	if(arrfile.exists())
    		readDemand(dirVeh);
    	else
    		generateDemand(dirVeh);
    	
    	//
    	dirPed = dir+"/arrivalPed"+((Integer)id).toString()+ ".txt";
    	File arrpedfile;
    	arrpedfile = new File(dirPed);
    	
    	if(arrpedfile.exists())
    		readPedDemand(dirPed);
    	else
    		generatePedDemand(dirPed);
    	
		if(vis)
		{
			timer.addActionListener(listener);
			road = new roadPanel(lane1,lane2,lane3, lane4, signal);
			panel = new CVSPeedPanel(road);
			JFrame f = new JFrame("Revised Algorithm");
	        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	        f.add(panel);
	        f.pack();
	        f.setLocationRelativeTo(null);
	        f.setVisible(true);	        
		}
		//start();
	}
	
	
	// sensitive analysis
	CVSpeedIDM(int id, double pf,  double pr, float wped, double prcon,double prauto,double tf,double dr, boolean vis, int opt, float para_q0, int para_numAnt, int para_numIter, float alp, float rh, float be, int[] outputs) 
	{
		vehflow = tf*3600.0;
		pedflow = pf*3600.0;
		totalvehFlow = tf;
    	totalpedFlow = pf;
    	vehflowRatio = dr;
    	pedflowRatio = pr;
		
    	flowPed1 = totalpedFlow*pedflowRatio/(1+pedflowRatio);
		flowPed2 = totalpedFlow/(1+pedflowRatio);
    	flowPed3 = totalpedFlow*pedflowRatio/(1+pedflowRatio);
		flowPed4 = totalpedFlow/(1+pedflowRatio);
    	
		q0 = para_q0;
		numAnts=para_numAnt;
		numIter=para_numIter;
		
		alpha=alp;
		rho=rh;
		beta=be;

		calibrate=false;
		timecount = 0;
		visual = vis;
		progid = id;
		progidPed =((Double)(pedflow)).intValue(); 
		
		Wveh = 1-wped;
		Wped = wped;
    	pr_con = prcon;
    	pr_auto = prauto;    	
   	
    	signalPlan = opt;
	
		// output modes;
    	out_perform = outputs[0];
    	out_cor = outputs[1];
    	out_com = outputs[2];
    	out_sen = outputs[3];
    	
		lane1 = new Lane(1, signal);
    	lane2 = new Lane(2, signal);
    	lane3 = new Lane(3, signal);
    	lane4 = new Lane(4, signal);
    	
    	triggerLocation = Math.min(-60.0, -(minSpacing+length)/pr_con*2); // set for security distance when need to stop.
    	random.setSeed(id);
    	File arrfile;
    	String dir;
    	String dirVeh;
    	String dirPed;
    	if(!calibrate)    		
    		dir = dirname();    		
    	else
    		dir = "calibrate";
    	
    	dirVeh = dir+"/arrivalVeh"+((Integer)id).toString()+".txt";
    	arrfile = new File(dirVeh);
    	
    	if(arrfile.exists())
    		readDemand(dirVeh);
    	else
    		generateDemand(dirVeh);
    	
    	//
    	dirPed = dir+"/arrivalPed"+((Integer)id).toString()+ ".txt";
    	File arrpedfile;
    	arrpedfile = new File(dirPed);
    	
    	if(arrpedfile.exists())
    		readPedDemand(dirPed);
    	else
    		generatePedDemand(dirPed);
    	
		if(vis)
		{
			timer.addActionListener(listener);
			road = new roadPanel(lane1,lane2,lane3, lane4, signal);
			panel = new CVSPeedPanel(road);
			JFrame f = new JFrame("Revised Algorithm");
	        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	        f.add(panel);
	        f.pack();
	        f.setLocationRelativeTo(null);
	        f.setVisible(true);	        
		}
	}
	 
	public void start() {
		if(visual)
			timer.start();
		else
			algorithm();

	}
	private void stop(ActionEvent e)
	{
		((Timer)e.getSource()).stop();
		try{
			resultwriter.write("\r\n");
			resultwriter.flush();
			resultwriter.close();

			if(calibrate)
			{
				trajwriter.write("\r\n");
				trajwriter.flush();
				trajwriter.close();
			}
			//controlwriter.close();
			//realwriter.close();
			java.time.LocalTime ntime2 = java.time.LocalTime.now();
			java.time.Duration duration = java.time.Duration.between(ntime1, ntime2);
			System.out.println(duration.toMillis());
			sim_time = duration;
			performanceindex();
		}
		catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} 
	}	 
	private void stop()
	{

		try{
			resultwriter.write("\r\n");
			resultwriter.flush();
			resultwriter.close();


		
			if(calibrate)
			{
				trajwriter.write("\r\n");
				trajwriter.flush();
				trajwriter.close();
			}

			//controlwriter.close();
			//realwriter.close();
			java.time.LocalTime ntime2 = java.time.LocalTime.now();
			java.time.Duration duration = java.time.Duration.between(ntime1, ntime2);
			System.out.println(duration.toMillis());
			sim_time = duration;
			performanceindex();
		}
		catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} 
	}

	
	/**
	 * @param dirname
	 * @param id
	 */
	public void readDemand(String dirnameVeh) //read demand files
	{
		
		arrivalfile = new File(dirnameVeh);
		String dirname = dirname();
		new File(dirname).mkdirs();
		
		try {			
			// read files
			arrivalreader = new FileReader(arrivalfile);
			BufferedReader br = new BufferedReader(arrivalreader);
			String line = null;
			int count = 0;
			while ((line = br.readLine()) != null && !line.isEmpty() && count<c) {
				String[] str = line.split(",");				
				ArrivalList.add(Integer.parseInt(str[0])) ;				
				Arrivals.add(Double.parseDouble(str[1]));
				
				switch (ArrivalList.get(count)/10000)
				{
					case 1:
						ArrivalList1.add(count);
						break;
					case 2:
						ArrivalList2.add(count);
						break;
					case 3:
						ArrivalList3.add(count);
						break;
					case 4:
						ArrivalList4.add(count);
						break;

				}
		
				equipped.add(ArrivalList.get(count)>10000 && ArrivalList.get(count)<16000 || ArrivalList.get(count)>20000 && ArrivalList.get(count)<26000 
						|| ArrivalList.get(count)>30000 && ArrivalList.get(count)<36000 || ArrivalList.get(count)>40000 && ArrivalList.get(count)<46000
						);	
				
				automated.add(ArrivalList.get(count)>10000 && ArrivalList.get(count)<13000 || ArrivalList.get(count)>20000 && ArrivalList.get(count)<23000
						|| ArrivalList.get(count)>30000 && ArrivalList.get(count)<33000 || ArrivalList.get(count)>40000 && ArrivalList.get(count)<43000
						);	
				revealed.add(false);
				count ++;
			 }
			c = Arrivals.size();
			br.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
		//time limit for pedestrian generation
		lastVehArrival = Arrivals.get(c-1);	
		
		
		///////////////////////////////////////////// create files to save vehicle results/////////////////////

		String  filepath;		
		filepath=filePath(dirname, 1);
		resultfile = new File(filepath);		
		if(!resultfile.exists()){
			try {
				resultfile.createNewFile();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		try {			
			resultwriter = new FileWriter(resultfile, false);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 

	}
	public void readPedDemand(String dirnamePed) {
        ////////////////////////////////////////
		//read pedestrian files
		arrivalpedfile = new File(dirnamePed);
		String dirname = dirname();
		new File(dirname).mkdirs();
		try {
			arrivalpedreader = new FileReader(arrivalpedfile);		
			BufferedReader br = new BufferedReader(arrivalpedreader);
			String line = null;
			while ((line = br.readLine()) != null && !line.isEmpty()) {
				String[] str = line.split(",");				
				ArrivalTimePed1_1.add(Double.parseDouble(str[0]));			
				ArrivalTimePed1_2.add(Double.parseDouble(str[1]));
				ArrivalTimePed2_1.add(Double.parseDouble(str[2]));			
				ArrivalTimePed2_2.add(Double.parseDouble(str[3]));
				ArrivalTimePed3_1.add(Double.parseDouble(str[4]));			
				ArrivalTimePed3_2.add(Double.parseDouble(str[5]));
				ArrivalTimePed4_1.add(Double.parseDouble(str[6]));			
				ArrivalTimePed4_2.add(Double.parseDouble(str[7]));
				DepTimePed1_1.add(Double.parseDouble(str[0]));			
				DepTimePed1_2.add(Double.parseDouble(str[1]));
				DepTimePed2_1.add(Double.parseDouble(str[2]));			
				DepTimePed2_2.add(Double.parseDouble(str[3]));
				DepTimePed3_1.add(Double.parseDouble(str[4]));			
				DepTimePed3_2.add(Double.parseDouble(str[5]));
				DepTimePed4_1.add(Double.parseDouble(str[6]));			
				DepTimePed4_2.add(Double.parseDouble(str[7]));
			 }
    	br.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
		
           ///////////////////////////////////////////// create files to save pedestrian results/////////////////////
		String  filepath;		
		filepath=filePath(dirname,2);
		resultpedfile = new File(filepath);		
		if(!resultpedfile.exists()){
			try {
				resultpedfile.createNewFile();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		try {			
			resultpedwriter = new FileWriter(resultpedfile, false);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 

	}
	
	public void generateDemand(String dirnameVeh){ //generate demand files
    	
		// generate arrival times
		
		double flow1 = totalvehFlow*vehflowRatio/(1+vehflowRatio), flow2= totalvehFlow/(1+vehflowRatio);
		double flow3 = totalvehFlow*vehflowRatio/(1+vehflowRatio), flow4= totalvehFlow/(1+vehflowRatio);

		
		double time1 = 0.0, time2 = 0.0, time3 =0.0, time4 = 0.0;
		int num1 = 10000, num2 = 20000, num3=30000, num4=40000;
		
		double [] arrival1 = new double[c];
		double [] arrival2 = new double[c];
		double [] arrival3 = new double[c];
		double [] arrival4 = new double[c];

		
		// Generate vehicle arrival times
		for(int i =0;i<c;i++)
		{
			double temptime1 = - Math.log(1.0-random.nextDouble())/flow1+ 0.5;// headway
			time1 += temptime1;
			arrival1[i] = time1;
			double temptime2 = - Math.log(1.0-random.nextDouble())/flow2+ 0.5;
			time2 += temptime2;
			arrival2[i] = time2;
			
			double temptime3 = - Math.log(1.0-random.nextDouble())/flow3+ 0.5;// headway
			time3 += temptime3;
			arrival3[i] = time3;
			double temptime4 = - Math.log(1.0-random.nextDouble())/flow4+ 0.5;
			time4 += temptime4;
			arrival4[i] = time4;

		}
		
		// Choose cars from both direction to form one (total c vehicles at intersection)
		int i1=0, i2=0, i3=0, i4=0;
		for(int i=0;i<c;i++)
		{
			double [] array = {arrival1[i1], arrival2[i2], arrival3[i3], arrival4[i4]};
			double minArray = array[0];
			int index = 0;
			for (int j =0; j<array.length; j++)
			{
				if(minArray>array[j])
				{
					minArray = array[j];
					index = j;
				}
			}
			switch (index)
			{
				case 0:
					num1 += 1;
					Arrivals.add(arrival1[i1]);//arrival time
					ArrivalList.add(num1);// arrival cars
					ArrivalList1.add(i);
					i1 += 1;
					break;
				case 1:
					num2 += 1;
					Arrivals.add(arrival2[i2]);//arrival time
					ArrivalList.add(num2);// arrival cars
					ArrivalList2.add(i);
					i2 += 1;
					break;
				case 2:
					num3 += 1;
					Arrivals.add(arrival3[i3]);//arrival time
					ArrivalList.add(num3);// arrival cars
					ArrivalList3.add(i);
					i3 += 1;
					break;
				case 3:
					num4 += 1;
					Arrivals.add(arrival4[i4]);//arrival time
					ArrivalList.add(num4);// arrival cars
					ArrivalList4.add(i);
					i4 += 1;
					break;
				
			}
		}			
		
		for(int i =0;i<c;i++)
		{
			revealed.add(false);
			equipped.add(false);
			automated.add(false);
		}
		

		int nc1 = (int)Math.ceil(pr_con*(double)i1); // number of equipped vehicles in approach 1;
		int nc2 = (int)Math.ceil(pr_con*(double)i2);
		int nc3 = (int)Math.ceil(pr_con*(double)i3);
		int nc4 = (int)Math.ceil(pr_con*(double)i4);

		
		if(nc1>=1)
		{
			ArrayList<Integer> cmb1 = rndcmb(i1-1,nc1-1);
			cmb1.add(i1-1);
			while(true)  //default condition is 8 (??200*0.2*pr_con=0.2) when c is 200; byin problem for pr_con=0.2 when c is 500,  here change to 20 (??500*0.2*pr_con=0.2) when c is 500;
			{
				int ii1 = 0;
				if(cmb1.get(0)<8)	//8 means every 8 vehicles should have one connected vehicle. 
					for(ii1=0;ii1<cmb1.size()-1;ii1++)
					{
						if(-cmb1.get(ii1)+cmb1.get(ii1+1)>8)
							break;
					}
				if(ii1<cmb1.size()-1)
				{
					cmb1 = rndcmb(i1-1,nc1-1);
					cmb1.add(i1-1);
				}
				else
				{
					break;
				}
				
			}
			for(int i:cmb1)
				equipped.set(ArrivalList1.get(i),true);
			int na1 = (int)Math.ceil(pr_auto*(double)nc1);
			if(na1>=1)
			{
				ArrayList<Integer> acmb1 = rndcmb(nc1,na1);
				for(int i:acmb1)
					automated.set(ArrivalList1.get(cmb1.get(i)),true);
			}			
		}
		
		if(nc2>=1)
		{
			ArrayList<Integer> cmb2 = rndcmb(i2-1,nc2-1);
			cmb2.add(i2-1);
			while(true)  //byin problem for pr_con=0.2;
			{
				int ii2 = 0;
				if(cmb2.get(0)<8)
					for(ii2=0;ii2<cmb2.size()-1;ii2++)
					{
						if(-cmb2.get(ii2)+cmb2.get(ii2+1)>8)
							break;
					}
				if(ii2<cmb2.size()-1)
				{
					cmb2 = rndcmb(i2-1,nc2-1);
					cmb2.add(i2-1);
				}
				else
				{
					break;
				}
				
			}
			for(int i:cmb2)
				equipped.set(ArrivalList2.get(i),true);
			int na2 = (int)Math.ceil(pr_auto*(double)nc2);
			if(na2>=1)
			{
				ArrayList<Integer> acmb2 = rndcmb(nc2,na2);
				for(int i:acmb2)
					automated.set(ArrivalList2.get(cmb2.get(i)),true);
			}			
		}
		
		if(nc3>=1)
		{
			ArrayList<Integer> cmb3 = rndcmb(i3-1,nc3-1);
			cmb3.add(i3-1);
			while(true)  //default condition is 8 (??200*0.2*pr_con0.2) when c is 200; byin problem for pr_con=0.2 when c is 500,  here change to 20 (??500*0.2*pr_con0.2) when c is 500;
			{
				int ii3 = 0;
				if(cmb3.get(0)<8)	//8 means every 8 vehicles should have one connected vehicle. 
					for(ii3=0;ii3<cmb3.size()-1;ii3++)
					{
						if(-cmb3.get(ii3)+cmb3.get(ii3+1)>8)
							break;
					}
				if(ii3<cmb3.size()-1)
				{
					cmb3 = rndcmb(i3-1,nc3-1);
					cmb3.add(i3-1);
				}
				else
				{
					break;
				}
				
			}
			for(int i:cmb3)
				equipped.set(ArrivalList3.get(i),true);
			int na3 = (int)Math.ceil(pr_auto*(double)nc3);
			if(na3>=1)
			{
				ArrayList<Integer> acmb3 = rndcmb(nc3,na3);
				for(int i:acmb3)
					automated.set(ArrivalList3.get(cmb3.get(i)),true);
			}			
		}
		
		if(nc4>=1)
		{
			ArrayList<Integer> cmb4 = rndcmb(i4-1,nc4-1);
			cmb4.add(i4-1);
			while(true)  //default condition is 8 (??200*0.2*pr_con=0.2) when c is 200; byin problem for pr_con=0.2 when c is 500,  here change to 20 (??500*0.2*pr_con=0.2) when c is 500;
			{
				int ii4 = 0;
				if(cmb4.get(0)<8)	//8 means every 8 vehicles should have one connected vehicle. 
					for(ii4=0;ii4<cmb4.size()-1;ii4++)
					{
						if(-cmb4.get(ii4)+cmb4.get(ii4+1)>8)
							break;
					}
				if(ii4<cmb4.size()-1)
				{
					cmb4 = rndcmb(i4-1,nc4-1);
					cmb4.add(i4-1);
				}
				else
				{
					break;
				}
				
			}
			for(int i:cmb4)
				equipped.set(ArrivalList4.get(i),true);
			int na4 = (int)Math.ceil(pr_auto*(double)nc4);
			if(na4>=1)
			{
				ArrayList<Integer> acmb4 = rndcmb(nc4,na4);
				for(int i:acmb4)
					automated.set(ArrivalList4.get(cmb4.get(i)),true);
			}			
		}
		
		
		
		for(int i=0;i<c;i++)
		{
			if(!equipped.get(i))
				ArrivalList.set(i, ArrivalList.get(i)+6000); // id code from X6000~X6999 for conventional vehicles
			else if(!automated.get(i))
				ArrivalList.set(i, ArrivalList.get(i)+3000); // id code from X3000~X5999 for equipped but no automated vehicles
		}		
		
		// time limit for pedestrian generation
		lastVehArrival = Arrivals.get(c-1); 
		////////////////////////////////////////////////// create files to save arriving vehicles and vehicle results////////////////////////////////////////////////////  	
    	
    	arrivalfile = new File(dirnameVeh);
    	String dirname = dirname();
    	new File(dirname).mkdirs();
    	if(!arrivalfile.exists()){
			try {
				arrivalfile.createNewFile();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		String  filepath;	
		filepath=filePath(dirname,1);
		resultfile = new File(filepath);		
		if(!resultfile.exists()){
			try {
				resultfile.createNewFile();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		

		
		try {
			arrivalwriter = new FileWriter(arrivalfile, false);			
			resultwriter = new FileWriter(resultfile, false);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
		
		
		for(int i=0;i<c;i++)
		{
			try{
				arrivalwriter.write(((Integer)ArrivalList.get(i)).toString());
				arrivalwriter.write(",");
				arrivalwriter.write(((Double)Arrivals.get(i)).toString());
				arrivalwriter.write("\r\n");
			}
			catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} 
		}
		try{
			arrivalwriter.write("\r\n");
			arrivalwriter.flush();
			arrivalwriter.close();
		}
		catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}    
		
    }	
	
	public void generatePedDemand(String dirnamePed) {
        
		
		// Generate pedestrian arrival times
		double arrivetime1_1 = 0.0;
		double arrivetime1_2 = 0.0;
		double arrivetime2_1 = 0.0;
		double arrivetime2_2 = 0.0;
		
		double arrivetime3_1 = 0.0;
		double arrivetime3_2 = 0.0;
		double arrivetime4_1 = 0.0;
		double arrivetime4_2 = 0.0;
		
		double temptime1_1;
		double temptime1_2;
		double temptime2_1;
		double temptime2_2;
		double temptime3_1;
		double temptime3_2;
		double temptime4_1;
		double temptime4_2;
		
		int mode=1;// mode=1: poisson distribution;  mode=0; uniform distribution
		while (arrivetime1_1<lastVehArrival){
			
			if (mode==1)
			    temptime1_1 = - Math.log(1.0-random.nextDouble())/flowPed1;	// this is poisson distribution
			else
			    temptime1_1 = random.nextDouble()+(1/flowPed1-1); //this is uniform distribution
			arrivetime1_1 += temptime1_1;
			ArrivalTimePed1_1.add(arrivetime1_1);
			DepTimePed1_1.add(arrivetime1_1);
		}
	
		while (arrivetime1_2<lastVehArrival) {
			
			if (mode==1)
			   temptime1_2 = - Math.log(1.0-random.nextDouble())/flowPed1;
			else
			   temptime1_2 = random.nextDouble()+(1/flowPed1-1); 
			arrivetime1_2 += temptime1_2;
			ArrivalTimePed1_2.add(arrivetime1_2);
			DepTimePed1_2.add(arrivetime1_2);
		}
		while (arrivetime2_1<lastVehArrival) {
			
			if (mode==1)
			   temptime2_1 = - Math.log(1.0-random.nextDouble())/flowPed2;
			else
			   temptime2_1 = random.nextDouble()+(1/flowPed2-1); 
			arrivetime2_1 += temptime2_1;
			ArrivalTimePed2_1.add(arrivetime2_1);
			DepTimePed2_1.add(arrivetime2_1);
		}
		while (arrivetime2_2<lastVehArrival) {
			
			if (mode==1)
			    temptime2_2 = - Math.log(1.0-random.nextDouble())/flowPed2;
			else
				temptime2_2 = random.nextDouble()+(1/flowPed2-1); 
			arrivetime2_2 += temptime2_2;
			ArrivalTimePed2_2.add(arrivetime2_2);
			DepTimePed2_2.add(arrivetime2_2);
		}
		
		while (arrivetime3_1<lastVehArrival){
			
			if (mode==1)
			   temptime3_1 = - Math.log(1.0-random.nextDouble())/flowPed3;	
			else
			   temptime3_1 = random.nextDouble()+(1/flowPed3-1); 
			arrivetime3_1 += temptime3_1;
			ArrivalTimePed3_1.add(arrivetime3_1);
			DepTimePed3_1.add(arrivetime3_1);
		}
	
		while (arrivetime3_2<lastVehArrival) {
			
			if (mode==1)
              temptime3_2 = - Math.log(1.0-random.nextDouble())/flowPed3;
			else
			  temptime3_2 = random.nextDouble()+(1/flowPed3-1); 
			arrivetime3_2 += temptime3_2;
			ArrivalTimePed3_2.add(arrivetime3_2);
			DepTimePed3_2.add(arrivetime3_2);
		}
		while (arrivetime4_1<lastVehArrival) {
			
			if (mode==1)
			   temptime4_1 = - Math.log(1.0-random.nextDouble())/flowPed4;
			else
			   temptime4_1 = random.nextDouble()+(1/flowPed4-1); 
			arrivetime4_1 += temptime4_1;
			ArrivalTimePed4_1.add(arrivetime4_1);
			DepTimePed4_1.add(arrivetime4_1);
		}
		while (arrivetime4_2<lastVehArrival) {
			
			if(mode==1)
				temptime4_2 = - Math.log(1.0-random.nextDouble())/flowPed4;
			else
				temptime4_2 = random.nextDouble()+(1/flowPed4-1); 
			arrivetime4_2 += temptime4_2;
			ArrivalTimePed4_2.add(arrivetime4_2);
			DepTimePed4_2.add(arrivetime4_2);
		}
		
		int maxsize = Math.max(ArrivalTimePed1_1.size(), Math.max(ArrivalTimePed1_2.size(), Math.max(ArrivalTimePed2_1.size(), Math.max(ArrivalTimePed2_2.size(), Math.max(ArrivalTimePed3_1.size(),Math.max(ArrivalTimePed3_2.size(), Math.max(ArrivalTimePed4_1.size(),ArrivalTimePed4_2.size())))))));
		for (int i=ArrivalTimePed1_1.size(); i<maxsize; i++)
		{
			ArrivalTimePed1_1.add(-1.0);
			DepTimePed1_1.add(-1.0);
		}
		for (int i=ArrivalTimePed1_2.size(); i<maxsize; i++)
		{
			ArrivalTimePed1_2.add(-1.0);
			DepTimePed1_2.add(-1.0);
		}
		for (int i=ArrivalTimePed2_1.size(); i<maxsize; i++)
		{
			ArrivalTimePed2_1.add(-1.0);
			DepTimePed2_1.add(-1.0);
		}
		for (int i=ArrivalTimePed2_2.size(); i<maxsize; i++)
		{
			ArrivalTimePed2_2.add(-1.0);
			DepTimePed2_2.add(-1.0);
		}
		for (int i=ArrivalTimePed3_1.size(); i<maxsize; i++)
		{
			ArrivalTimePed3_1.add(-1.0);
			DepTimePed3_1.add(-1.0);
		}
		for (int i=ArrivalTimePed3_2.size(); i<maxsize; i++)
		{
			ArrivalTimePed3_2.add(-1.0);
			DepTimePed3_2.add(-1.0);
		}
		for (int i=ArrivalTimePed4_1.size(); i<maxsize; i++)
		{
			ArrivalTimePed4_1.add(-1.0);
			DepTimePed4_1.add(-1.0);
		}
		for (int i=ArrivalTimePed4_2.size(); i<maxsize; i++)
		{
			ArrivalTimePed4_2.add(-1.0);
			DepTimePed4_2.add(-1.0);
		}
		//////////////////////////////////////////////////////// create files to save arriving pedestrians and pedestrian results/////////////////////////////////////////////////////	 
    	arrivalpedfile = new File(dirnamePed);
    	String dirname = dirname();
    	new File(dirname).mkdirs();
    	if(!arrivalpedfile.exists()){
			try {
				arrivalpedfile.createNewFile();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
    	
    	try {			
			arrivalpedwriter = new FileWriter(arrivalpedfile, false);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
    	///////////////////
		int count=0;
		while(count<maxsize)
		{
			try{

				arrivalpedwriter.write(((Double)ArrivalTimePed1_1.get(count)).toString());

				arrivalpedwriter.write(",");

				arrivalpedwriter.write(((Double)ArrivalTimePed1_2.get(count)).toString());

				arrivalpedwriter.write(",");

				arrivalpedwriter.write(((Double)ArrivalTimePed2_1.get(count)).toString());

				arrivalpedwriter.write(",");

				arrivalpedwriter.write(((Double)ArrivalTimePed2_2.get(count)).toString());
				arrivalpedwriter.write(",");
				
				arrivalpedwriter.write(((Double)ArrivalTimePed3_1.get(count)).toString());

				arrivalpedwriter.write(",");

				arrivalpedwriter.write(((Double)ArrivalTimePed3_2.get(count)).toString());

				arrivalpedwriter.write(",");
				
				arrivalpedwriter.write(((Double)ArrivalTimePed4_1.get(count)).toString());

				arrivalpedwriter.write(",");

				arrivalpedwriter.write(((Double)ArrivalTimePed4_2.get(count)).toString());
				arrivalpedwriter.write("\r\n");
				count++;
			}
			catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} 
		}
		try{
			arrivalpedwriter.write("\r\n");
			arrivalpedwriter.flush();
			arrivalpedwriter.close();
		}
		catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} 
		
		// create file to save pedestrian results
		String  filepath;	
		filepath=filePath(dirname,2);
		resultpedfile = new File(filepath);		
		if(!resultpedfile.exists()){
			try {
				resultpedfile.createNewFile();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		try {					
			resultpedwriter = new FileWriter(resultpedfile, false);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
		
	}
	
	public void performanceindex ()// put in stop(e);
	{
		String dirname, filepath;
		dirname = dirname();		
		filepath=filePath(dirname,1);
		resultfile = new File(filepath);
				
		// observed vehicle delay, stops and pedestrian delay in simulation.
		try {
			//Actual vehicle delay and stops
			resultreader = new FileReader(resultfile);
			BufferedReader br = new BufferedReader(resultreader);
			String line = null;
			int count = 0;
			double veh_delay=0.0;
			double veh_stops=0.0;
			while ((line = br.readLine()) != null && !line.isEmpty() && count<c) {
				String[] str = line.split(",");	
				veh_delay=veh_delay+(Double.parseDouble(str[2])-Double.parseDouble(str[1]));
				veh_stops=veh_stops+(Double.parseDouble(str[3]));
				count++;
			}
			br.close();
			
			//Result1: pedestrians Actual pedestrian delay
			double ped_delay1_1=0, ped_delay1_2=0, ped_delay2_1=0, ped_delay2_2=0, ped_delay3_1=0, ped_delay3_2=0, ped_delay4_1=0, ped_delay4_2=0;
			double ped_delay = 0;
	        int[] arrsize= {ArrivalTimePed1_1.size(),ArrivalTimePed1_2.size(),ArrivalTimePed2_1.size(),ArrivalTimePed2_2.size(),ArrivalTimePed3_1.size(),ArrivalTimePed3_2.size(),ArrivalTimePed4_1.size(),ArrivalTimePed4_2.size()};
	        int maxsize = Arrays.stream(arrsize).max().getAsInt();
	        
			for (int i=0; i< maxsize ; i++) {
				if (ArrivalTimePed1_1.get(i)!=-1.0)
					ped_delay1_1= DepTimePed1_1.get(i)-ArrivalTimePed1_1.get(i);
				else
					ped_delay1_1=0;
				if (ArrivalTimePed1_2.get(i)!=-1.0)
					ped_delay1_2= DepTimePed1_2.get(i)-ArrivalTimePed1_2.get(i);
				else
					ped_delay1_2=0;
				if (ArrivalTimePed2_1.get(i)!=-1.0)
					ped_delay2_1= DepTimePed2_1.get(i)-ArrivalTimePed2_1.get(i);
				else
					ped_delay2_1=0;
				if (ArrivalTimePed2_2.get(i)!=-1.0)
					ped_delay2_2= DepTimePed2_2.get(i)-ArrivalTimePed2_2.get(i);
				else
					ped_delay2_2=0;
				if (ArrivalTimePed3_1.get(i)!=-1.0)
					ped_delay3_1= DepTimePed3_1.get(i)-ArrivalTimePed3_1.get(i);
				else
					ped_delay3_1=0;
				if (ArrivalTimePed3_2.get(i)!=-1.0)
					ped_delay3_2= DepTimePed3_2.get(i)-ArrivalTimePed3_2.get(i);
				else
					ped_delay3_2=0;
				if (ArrivalTimePed4_1.get(i)!=-1.0)
					ped_delay4_1= DepTimePed4_1.get(i)-ArrivalTimePed4_1.get(i);
				else
					ped_delay4_1=0;
				if (ArrivalTimePed4_2.get(i)!=-1.0)
					ped_delay4_2= DepTimePed4_2.get(i)-ArrivalTimePed4_2.get(i);
				else
					ped_delay4_2=0;
				
				ped_delay += ped_delay1_1+ped_delay1_2+ped_delay2_1+ped_delay2_2+ped_delay3_1+ped_delay3_2+ped_delay4_1+ped_delay4_2;	
				// special case setting
				if (count2_1==0 && count2_2 ==0)
					ped_delay += ped_delay1_1+ped_delay1_2; // case in 2nd approach is 0;
				if (count1_1==0 && count1_2 ==0)
					ped_delay += ped_delay2_1+ped_delay2_2; // case in 1st approach is 0;
				
				String str_deptime = (((Double)DepTimePed1_1.get(i)).toString()+ "," +  ((Double)DepTimePed1_2.get(i)).toString()+ ","+ ((Double)DepTimePed2_1.get(i)).toString()+","+((Double)DepTimePed2_2.get(i)).toString() + "," + 
				((Double)DepTimePed3_1.get(i)).toString()+ ","+ ((Double)DepTimePed3_2.get(i)).toString()+ "," + ((Double)DepTimePed4_1.get(i)).toString()+ "," + ((Double)DepTimePed4_2.get(i)).toString());
				String str_delay = ((Double)ped_delay1_1).toString()+","+ ((Double)ped_delay1_2).toString()+ ","+ ((Double)ped_delay2_1).toString()+ ","+((Double)ped_delay2_2).toString()+","+ 
				((Double)ped_delay3_1).toString()+","+ ((Double)ped_delay3_2).toString()+","+ ((Double)ped_delay4_1).toString()+","+ ((Double)ped_delay4_2).toString();
				
				
				try {
					resultpedwriter.write(str_deptime + "," + str_delay  +"\r\n");
	
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			try{
				resultpedwriter.flush();
				resultpedwriter.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			// Results2: signal information
			 ///////////////////////////////////////////// create files to save sig results/////////////////////
			new File(dirname + "sigPlan").mkdirs();
			String sigfilepath = filePath(dirname + "sigPlan", 1);
			resultsigfile = new File(sigfilepath);		
			if(!resultsigfile.exists()){
				try {
					resultsigfile.createNewFile();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			try {			
				resultsigwriter = new FileWriter(resultsigfile, false);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} 
			for (int i=0; i<Math.min(sigSwitchTime.size(), FDWSwitchTime.size()); i++) {
				String str_sigswitchtime = ((Double)sigSwitchTime.get(i)).toString()+","+ ((Double)FDWSwitchTime.get(i)).toString();
				try {
					resultsigwriter.write(str_sigswitchtime  +"\r\n");	
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			try{
				resultsigwriter.flush();
				resultsigwriter.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}	
			
			
			///////////////
			// performance index 1: average values
			 double aveveh_delay = veh_delay /c;
			 double aveveh_stop =  veh_stops /c;
			 int arrn=count1_1 + count1_2 + count2_1 + count2_2 + count3_1 + count3_2 + count4_1 + count4_2;
			 double aveped_delay = ped_delay/(arrn);
			//  
			// or 
			 int waitn=0;
			 for (int i=0; i<waitN.size(); i++)
				 waitn+=waitN.get(i);
			 double aveped_delay1 = ped_delay/waitn;
			 
			// performance index 2: average person values	
			double aveper_delay = (veh_delay*1.2 + ped_delay)/(1.2*c + arrn); 
			//or
			double aveper_delay1 = (veh_delay*1.2 + ped_delay)/(1.2*c + waitn);
			
			//Save final results to file 	
			String output; 
			//output= Double.toString(aveveh_delay) + "," + Double.toString(aveveh_stop) + "," + Double.toString(aveped_delay) + "," + Double.toString(aveper_delay) + "," + Double.toString(aveped_delay1) + "," + Double.toString(aveper_delay1)+ "," + Double.toString(sim_time.toMillis()); // for pedestrian control
			output= Double.toString(aveveh_delay) + "," + Double.toString(aveveh_stop) + "," + Double.toString(aveped_delay) + "," + Double.toString(aveper_delay) + "," + Double.toString(aveped_delay1) + "," + Double.toString(aveper_delay1)+ "," + Double.toString(waitn)+ "," + Double.toString(arrn);
			// only vehicles
			//output= Double.toString(aveveh_delay) + "," + Double.toString(aveveh_stop);
			DecimalFormat df2=new DecimalFormat("0.00");
			System.out.println(output);

			try {	
				//1)performance
				if (out_perform!=0) {
					if (out_perform==1) { // test
						String pathname =  "syn_result/test/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue() +"/signalPlan_" + ((Integer)signalPlan).toString()+".txt";								
						syntheticResults = new File(pathname);
						new File("syn_result/test/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue() + "/").mkdirs();
					}

					// normal performance
					if (out_perform==2) {
						String pathname =  "syn_result/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue() +"/signalPlan_" + ((Integer)signalPlan).toString()+".txt";								
						syntheticResults = new File(pathname);
						new File("syn_result/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue() + "/").mkdirs();
					}

					// CAV performance
					if (out_perform==3) { // information level
						String pathname =  "syn_result/info/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue() +"/signalPlan_" + ((Integer)signalPlan).toString()+".txt";								
						syntheticResults = new File(pathname);
						new File("syn_result/info/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue() + "/").mkdirs();
					}

					if (out_perform==4) { // automation level
						String pathname =  "syn_result/auto/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue() +"/signalPlan_" + ((Integer)signalPlan).toString()+".txt";								
						syntheticResults = new File(pathname);
						new File("syn_result/auto/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue() + "/").mkdirs();
					}

					if(!syntheticResults.exists()){
						try {
							syntheticResults.createNewFile();
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
					controlwriter = new FileWriter(syntheticResults, true); // true means save old results; false means no save.			
					controlwriter.write(dirname + "Wped=" +df2.format(Wped) + "/");				
				}
				//2)correlation analysis
				if (out_cor==1)			
					/////////////////////////// results for algorithm analysis: relationship between predicted delay and actual delay//////////////////////////////         
					// save predicted delay per departure sequence to file			
					{
						String path_predictedDelay =  "Analyse/correlation/predictedDelay" + progid + "w" +  df2.format(Wped)+ ".txt";
						File predictedDelay = new File(path_predictedDelay);
						new File("Analyse/correlation/").mkdirs();
						if(!predictedDelay.exists()){
							try {
								predictedDelay.createNewFile();
							} catch (IOException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}
						FileWriter analysiswriter = new FileWriter(predictedDelay, false); 
						for (int i=0; i<taList.size(); i++) 
						{
							String idList=null;
							if (!prevehIDList.get(i).isEmpty())
								idList = ((Integer)prevehIDList.get(i).get(0)).toString();
							for (int j=1; j<prevehIDList.get(i).size(); j++)
								idList = idList + "," +  ((Integer)prevehIDList.get(i).get(j)).toString();

							String output2 = ((Double)taList.get(i)).toString()+ "," + ((Double)TbList.get(i)).toString()+ "," + ((Double)prevehDelayList.get(i)).toString() +"," + ((Double)prepedDelayList.get(i)).toString()+  "," + ((Double)timeList.get(i)).toString() + ","  + idList;
							try {
								analysiswriter.write(output2);
								//controlwriter.write(Double.toString(sim_time));
								analysiswriter.write("\r\n");
							}
							catch (IOException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}  
						}
						try {
							analysiswriter.flush();
							analysiswriter.close();
						}
						catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} 	
					} 
				
				//3)computation time analysis
				if (out_com==1) {
					if (signalPlan==4 || signalPlan==5) {
						String pathname = "Analyse/timecompare/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue()+ "/signalPlan_" + ((Integer)signalPlan).toString()+".txt";
						syntheticResults = new File(pathname);
						new File("Analyse/timecompare/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue() + "/").mkdirs();
						if(!syntheticResults.exists()){
							try {
								syntheticResults.createNewFile();
							} catch (IOException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}			    	
						controlwriter = new FileWriter(syntheticResults, true); 
						controlwriter.write("Wped=" +df2.format(Wped) + "/" + "ant=" + Integer.toString(numAnts)+ "/" + "iter=" + Integer.toString(numIter)+ "/");
					}
				}
				//4) ACS parameter sensitivity 
				if (signalPlan==5 && out_sen!=0) {
					if (out_sen==1) {
						String pathname = "Analyse/sensitivity/e0/" + Integer.toString(progid)+ "/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue()+ "_A_" + Integer.toString(numAnts)+ "_I_" + Integer.toString(numIter)+ ".txt"; // for q0
					    syntheticResults = new File(pathname);
					    new File("Analyse/sensitivity/e0/" + Integer.toString(progid)+ "/").mkdirs();
					}
					
					if (out_sen==2) {
						String pathname = "Analyse/sensitivity/alpha&rho/" + Integer.toString(progid)+ "/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue()+ "alpha" + df2.format(alpha) + "_rho" + df2.format(rho)+ ".txt"; // for alpha and rho
						syntheticResults = new File(pathname);
						new File("Analyse/sensitivity/alpha&rho/" + Integer.toString(progid)+ "/").mkdirs();
					}
					
					if (out_sen==3) {
						String pathname = "Analyse/sensitivity/beta/" + Integer.toString(progid)+ "/vehflow=" + ((Double)(vehflow)).intValue() + "_pedflow=" + ((Double)(pedflow)).intValue()+ "alpha" + df2.format(alpha) + "_rho" + df2.format(rho)+ "_beta" + df2.format(beta) + ".txt"; // for beta
						syntheticResults = new File(pathname);
						new File("Analyse/sensitivity/beta/" + Integer.toString(progid)+ "/").mkdirs();
					}
	
					if(!syntheticResults.exists()){
						try {
							syntheticResults.createNewFile();
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
					controlwriter = new FileWriter(syntheticResults, true); 
					controlwriter.write("q0=" +df2.format(q0) + "/" + "ant=" + Integer.toString(numAnts)+ "/" + "iter=" + Integer.toString(numIter)+ "/");
				}
			    
			    if (out_cor==0) {// save results except 2) correlation analysis, which saves inside.
			    	controlwriter.write(",");
			    	controlwriter.write(output);
			    	controlwriter.write("\r\n");
			    	controlwriter.flush();
			    	controlwriter.close();
			    }
							
			}
			catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}  	
            
		
	
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
			
	}
	
	String filePath(String dirname, int type)
	{
		String filepath;
		if (type==1) // for vehicle
		{
			switch(signalPlan)
			{
			case 1: 
				filepath = dirname+"/fixedResultVeh"+((Integer)progid).toString()+".txt";
				break;
			case 2: 
				filepath = dirname+"/resultVeh"+((Integer)progid).toString()+".txt";
				break;
			case 3: 
				filepath = dirname+"/caliResultVeh"+((Integer)progid).toString()+".txt";
				break;
			case 4: 
				filepath = dirname+"/adaptiveResultVeh"+((Integer)progid).toString()+".txt";
				break;
			case 5:
				filepath = dirname+"/ACSControlVeh"+((Integer)progid).toString()+".txt";
				break;
			case 6:
				filepath = dirname+"/BBControlVeh"+((Integer)progid).toString()+".txt";
				break;	
				
			default:
				filepath = dirname+"/fixedResultVeh"+((Integer)progid).toString()+".txt";
				break;
			}
			return
					filepath;
		}
		else // for pedestrian
		{
			//DecimalFormat df1=new DecimalFormat("0.0"); 
			DecimalFormat df2=new DecimalFormat("0.00"); 
			String str = "w" + df2.format(Wped);
			switch(signalPlan)
			{
			case 1: 
				filepath = dirname+"/fixedResultPed"+((Integer)progid).toString()+str+".txt";
				break;
			case 2: 
				filepath = dirname+"/resultPed"+((Integer)progid).toString()+str+".txt";
				break;
			case 3: 
				filepath = dirname+"/caliResultPed"+((Integer)progid).toString()+str+".txt";
				break;
			case 4: 
				filepath = dirname+"/adaptiveResultPed"+((Integer)progid).toString()+str+".txt";
				break;
			case 5:
				filepath = dirname+"/ACSControlPed"+((Integer)progid).toString()+str+".txt";
				break;
			case 6:
				filepath = dirname+"/BBControlPed"+((Integer)progid).toString()+str+".txt";
				break;	
				
			default:
				filepath = dirname+"/fixedResultPed"+((Integer)progid).toString()+str+".txt";
				break;
			}
			return
					filepath;
		}

	}

	public int nchoosek(int n, int k) // Cn,k
	{
		// the number of possible combinations (choose k elements from n items)
		if(k==0)
			return 1;
		else if(k==n)
			return 1;
		else if(k==1)
			return n;
		else
			return nchoosek(n-1,k)+nchoosek(n-1,k-1);
	} 
	public ArrayList<ArrayList<Integer>> cmbbnk(int n, int k)
	{
		// output all the possible combinations (choose k elements from n items)// with order
		if(k>n || k == 0)
			return new ArrayList<ArrayList<Integer>>();

		if(n==1 && k == 1)
		{
			ArrayList<ArrayList<Integer>> aa = new ArrayList<ArrayList<Integer>>();
			ArrayList<Integer> bb = new ArrayList<Integer>();
			bb.add(1);
			aa.add(bb);
			return aa;
		}

		ArrayList<ArrayList<Integer>> answer = cmbbnk(n-1,k);
		ArrayList<ArrayList<Integer>> answer1 = cmbbnk(n-1,k-1);
		if(answer1.isEmpty())
		{
			ArrayList<Integer> c = new ArrayList<Integer>();
			c.add(n);
			answer.add(c);
		}
		for(ArrayList<Integer> c: answer1)
		{
			c.add(n);
			answer.add(c);
		}

		return answer;

	}    
	public ArrayList<Integer> rndcmb(int n, int k) 
	{
		// choose k values in 0~n, and ascend them.
		int [] a = new int[n];
		ArrayList<Integer> cmb = new ArrayList<Integer>();
		for(int i = 0;i<n;i++)
		{
			a[i] = i;
		}

		for (int i = 0; i < n; i++) {
			int r = (int) (random.nextInt(i+1));     // int between 0 and i
			int swap = a[r];
			a[r] = a[i];
			a[i] = swap;
		}

		for(int i =0;i<Math.min(n, k);i++)
			cmb.add(a[i]);
		Collections.sort(cmb);
		return cmb;
	}
	
	boolean CVTrigger(double time)// when to trigger the control
	{
		boolean vehevent = false;
		boolean pedevent = false;
		for(int i=1;i<=4;i++)// 4 lanes
		{
			Lane lane = null;
			if (i==1)
				lane=lane1;
			if (i==2)
				lane=lane2;
			if (i==3)
				lane=lane3;
			if (i==4)
				lane=lane4;

			
			for(int j = lane.carList.size()-1;j>=0;j--)
			{
				Car car = lane.carList.get(j);
				if(car.location>=-zoneLength && car.prevLocation<-zoneLength && car.equipped)
					vehevent= true; //1. as long as one new equipped car comes
				else if(car.location>=-zoneLength && car.location<=0)
				{
					if(car.equipped && car.speed < 1e-2 && car.prevSpeed>=1e-2 )
						vehevent= true; //2. equipped car on lane stops.
				}		
			}
		}
			
		pedevent = pedestrianArrival (time);
			
		
		if (vehevent==true || pedevent==true)
			return true;
		else
			return false;
	}
	
	
	public void executionSignal (double time)// current step
	{
	
		Lane l = null;
		// considering cars already pass the intersection
		for(int i = 0;i<DepartureList.size();i++)
		{
			if (ApproachList.get(i)==1)
				l=lane1;
			if (ApproachList.get(i)==2)
				l=lane2;
			if (ApproachList.get(i)==3)
				l=lane3;
			if (ApproachList.get(i)==4)
				l=lane4;
			
			if(l.findCar(DepartureList.get(i)).location>-1e-3)
				l.assignSpeed(DepartureList.get(i),-1.0);
		}
		// If the first car in the departure list passes the intersection, take it out.
		while(!DepartureList.isEmpty())
		{
			if (ApproachList.get(0)==1)
				l=lane1;
			if (ApproachList.get(0)==2)
				l=lane2;
			if (ApproachList.get(0)==3)
				l=lane3;
			if (ApproachList.get(0)==4)
				l=lane4;

			
			if(l.findCar(DepartureList.get(0))!=null && l.findCar(DepartureList.get(0)).location > 0)// if the car has passed the intersection,
			{				 			 

				    
				if (lastApproach==1 || lastApproach==3)
				{
					if (ApproachList.get(0)==1)
						lastPlatoon1=lastPlatoon1+1;// total number of vehicles has already passed.
				    if (ApproachList.get(0)==3)
				    	lastPlatoon2=lastPlatoon2+1;
				    if (ApproachList.get(0)==2)
				    {
				    	lastPlatoon1=1;
				        lastPlatoon2=0;
				    }
				    if (ApproachList.get(0)==4)
				    {
				    	lastPlatoon1=0;
				        lastPlatoon2=1;
				    }
				 
				}
				if (lastApproach==2 || lastApproach==4)
				{
					if (ApproachList.get(0)==2)
						lastPlatoon1=lastPlatoon1+1;
				    if (ApproachList.get(0)==4)
				    	lastPlatoon2=lastPlatoon2+1;
				    if (ApproachList.get(0)==1)
				    {
				    	lastPlatoon1=1;
				        lastPlatoon2=0;
				    }
				    if (ApproachList.get(0)==3) 
				    {
				    	lastPlatoon1=0;
				        lastPlatoon2=1;
				    }

				}	

				lastApproach = ApproachList.get(0);
				leadSpeed[ApproachList.get(0)-1] = l.findCar(DepartureList.get(0)).speed;
				
				if (ApproachList.get(0)==1 || ApproachList.get(0)==2)
					lastDepTime1 = time;// when vehicle entering zone?
				else
					lastDepTime2 = time;
				
				
				///////////////
				if(signal.phase!=2-ApproachList.get(0)%2)
					preTimeFDW.remove(0);			
				////////////////
				
				DepartureList.remove(0);
				ApproachList.remove(0);
				
				
				
				//System.out.println(((Double)time).toString());
				//System.out.println(DepartureList.toString());
			}
			else
				break;
		}
		///////////////////////////////////////////////////////////////////////////////////
		
		// signal change in execution from decision of vehicle departure sequence
        
		if(lane1.isISEmpty() && lane2.isISEmpty() && lane3.isISEmpty() && lane4.isISEmpty() && (time - signal.startTime > gmin))// when to switch light (pedestrian FDW)
		{				
			if(time-signal.startTime >= gmax) {
				
				signal.switchSignal(time);
				// pedestrians				
				pedestrianDischarge (time);		
				
				// store
				sigSwitchTime.add(time);

			}
				
			else if(!DepartureList.isEmpty())
			{
										
				if (ApproachList.get(0)==1)
					l=lane1;
				if (ApproachList.get(0)==2)
					l=lane2;
				if (ApproachList.get(0)==3)
					l=lane3;
				if (ApproachList.get(0)==4)
					l=lane4;

				if(l.findCar(DepartureList.get(0)).location> triggerLocation  && signal.phase != l.phase)
					// If the I/S is empty and the first car in the departure list is near the I/S, 
					// and if the signal is not for the first car, the signal should be switched.
					// the first car in the departure list is within a certain distance to the intersection, switch the signal if needed
				{					
										
					signal.switchSignal(time);
					// pedestrians				
					pedestrianDischarge (time);					
					// store
					sigSwitchTime.add(time);
					
				}
			}	
			
			else if(DepartureList.isEmpty())// add this for pedestrian signal
			{
				if(signal.phase != sigMandatory)
				{
					signal.switchSignal(time);
					// pedestrians				
					pedestrianDischarge (time);
					
					// store
					sigSwitchTime.add(time);					
					sigMandatory = signal.phase;
				}	
			}
			
		}
		
		//////////////consider the flashing don't walk time stamp//////////////
		if (preTimeFDW.size()!=0) {		
			tpc=preTimeFDW.get(0);	
		}					
		if (time>=tpc && time < tpc + Tpc ) {		
			flagFDWSwitch=true;			
			if (tpc!=tpc0 && tpc!=0) {
				FDWSwitchTime.add(tpc);
				tpc0=tpc;
			}			
		}
		else {
			flagFDWSwitch=false;
		}

	}
	
	
	void pedestrianDischarge (double time) {
		// for real pedestrian delay
		//if (signal.phase==2) // 
		//{			
			if (waitN1_1!=0)
				for (int i=0; i<waitN1_1; i++) 
					DepTimePed1_1.set(count1_1-i-1, time);
			if (waitN1_2!=0)
				for (int i=0; i<waitN1_2; i++) 
					DepTimePed1_2.set(count1_2-i-1, time);
			if (waitN3_1!=0)
				for (int i=0; i<waitN3_1; i++) 
					DepTimePed3_1.set(count3_1-i-1, time);
			if (waitN3_2!=0)
				for (int i=0; i<waitN3_2; i++) 
					DepTimePed3_2.set(count3_2-i-1, time);
			
			//
			waitN.add(waitN1_1+waitN1_2+waitN3_1+waitN3_2);
			waitN1.add(waitN1_1+waitN1_2);
			waitN1_1 = 0;
			waitN1_2 = 0; 
			waitN3_1 = 0;
			waitN3_2 = 0;
			WaitListPed1.clear();
			WaitListPed3.clear();
		//}
		//if (signal.phase==1)
		//{
			if (waitN2_1!=0)
				for (int i=0; i<waitN2_1; i++) 
					DepTimePed2_1.set(count2_1-i-1, time);
			if (waitN2_2!=0)
				for (int i=0; i<waitN2_2; i++) 
					DepTimePed2_2.set(count2_2-i-1, time);
			if (waitN4_1!=0)
				for (int i=0; i<waitN4_1; i++) 
					DepTimePed4_1.set(count4_1-i-1, time);
			if (waitN4_2!=0)
				for (int i=0; i<waitN4_2; i++) 
					DepTimePed4_2.set(count4_2-i-1, time);
			
			//
			waitN.add(waitN2_1+waitN2_2+waitN4_1+waitN4_2);
			waitN2.add(waitN2_1+waitN2_2);
			waitN2_1 = 0;
			waitN2_2 = 0;
			waitN4_1 = 0;
			waitN4_2 = 0;
			WaitListPed2.clear();
			WaitListPed4.clear();
		//}	
	}
	
	boolean pedestrianArrival (double time) {   
		boolean pedevent = false;
		double timeint= timeStep/1000.0;
		// arrival pedestrians
				if (Wped>=0) {
					// pedestrian arrivals			
					if ((Math.abs((int)(ArrivalTimePed1_1.get(count1_1)*1000)/1000.0 - time)) < timeint && count1_1<ArrivalTimePed1_1.size()-1) {	// keep time as 0.000 format	
						arrivalPed1_1 = true;
						count1_1++;		
						//System.out.println((int)(ArrivalTimePed1_1.get(count1_1)*1000)/1000.0);
					}
					else
						arrivalPed1_1 = false;
				
					if ((Math.abs((int)(ArrivalTimePed1_2.get(count1_2)*1000)/1000.0 - time)) < timeint && count1_2<ArrivalTimePed1_2.size()-1) {				
						arrivalPed1_2 = true;
						count1_2++;
					}
					else
						arrivalPed1_2 = false;
						
						
					if ((Math.abs((int)(ArrivalTimePed2_1.get(count2_1)*1000)/1000.0 - time)) < timeint && count2_1<ArrivalTimePed2_1.size()-1) {				
						arrivalPed2_1 = true;
						count2_1++;
					}
					else
						arrivalPed2_1 = false;
					
					if ((Math.abs((int)(ArrivalTimePed2_2.get(count2_2)*1000)/1000.0 - time)) < timeint && count2_2<ArrivalTimePed2_2.size()-1) {				
						arrivalPed2_2 = true;
						count2_2++;
					}
					else
						arrivalPed2_2 = false;	
						
					
					if ((Math.abs((int)(ArrivalTimePed3_1.get(count3_1)*1000)/1000.0 - time)) < timeint && count3_1<ArrivalTimePed3_1.size()-1) {				
						arrivalPed3_1 = true;
						count3_1++;
					}
					else
						arrivalPed3_1 = false;
					
					if ((Math.abs((int)(ArrivalTimePed3_2.get(count3_2)*1000)/1000.0 - time)) < timeint && count3_2<ArrivalTimePed3_2.size()-1) {				
						arrivalPed3_2 = true;
						count3_2++;
					}
					else
						arrivalPed3_2 = false;	
					
					if ((Math.abs((int)(ArrivalTimePed4_1.get(count4_1)*1000)/1000.0 - time)) < timeint && count4_1<ArrivalTimePed4_1.size()-1) {				
						arrivalPed4_1 = true;
						count4_1++;
					}
					else
						arrivalPed4_1 = false;
					
					if ((Math.abs((int)(ArrivalTimePed4_2.get(count4_2)*1000)/1000.0 - time)) < timeint && count4_2<ArrivalTimePed4_2.size()-1) {				
						arrivalPed4_2 = true;
						count4_2++;
					}
					else
						arrivalPed4_2 = false;	
					
					// pedestrian waiting and triggering event :  
					double tol= 0.5;
					signalLight sig = signal.copy();	
					if (sig.phase==2) 
					{ 
						if(arrivalPed2_1==true && (ArrivalTimePed2_1.get(count2_1-1)-sig.startTime>tol) || arrivalPed2_2==true && (ArrivalTimePed2_2.get(count2_2-1)-sig.startTime>tol)) // case of pedestrian arrival at red interval
						{
							if (arrivalPed2_1==true )
								waitN2_1++;
							if (arrivalPed2_2==true )
								waitN2_2++;
							
							WaitListPed2.add(time);
							pedevent = true;
						}
						if ((arrivalPed1_1==true || arrivalPed1_2==true) && (time>=tpc && time<tpc+Tpc)) // case of pedestrian arrival at PC interval, 
						{
							if (arrivalPed1_1==true)
								waitN1_1++;
							if (arrivalPed1_2==true )
								waitN1_2++;
							
							WaitListPed1.add(time);
							pedevent = true;
						}
						
						//
						if(arrivalPed4_1==true && (ArrivalTimePed4_1.get(count4_1-1)-sig.startTime>tol)|| arrivalPed4_2==true && (ArrivalTimePed4_2.get(count4_2-1)-sig.startTime>tol)) 
						{
							if (arrivalPed4_1==true)
								waitN4_1++;
							if (arrivalPed4_2==true)
								waitN4_2++;
							
							WaitListPed4.add(time);
							pedevent = true;
						}
						if ((arrivalPed3_1==true || arrivalPed3_2==true)&& (time>=tpc && time<tpc+Tpc)) 
						{
							if (arrivalPed3_1==true)
								waitN3_1++;
							if (arrivalPed3_2==true)
								waitN3_2++;
							
							WaitListPed3.add(time);
							pedevent = true;
						}
						
					}
					if (sig.phase==1) 
					{ 
						if(arrivalPed1_1==true && (ArrivalTimePed1_1.get(count1_1-1)-sig.startTime>tol) || arrivalPed1_2==true && (ArrivalTimePed1_2.get(count1_2-1)-sig.startTime>tol)) 
						{
							if (arrivalPed1_1==true)
								waitN1_1++;
							if (arrivalPed1_2==true)
								waitN1_2++;
							
							WaitListPed1.add(time);
							pedevent = true;
						}
						if ((arrivalPed2_1==true || arrivalPed2_2==true) && (time>=tpc && time<tpc+Tpc))	
						{
							if (arrivalPed2_1==true)
								waitN2_1++;
							if (arrivalPed2_2==true)
								waitN2_2++;
							
							WaitListPed2.add(time);
							pedevent = true;
						}
						//
						if(arrivalPed3_1==true && (ArrivalTimePed3_1.get(count3_1-1)-sig.startTime>tol) || arrivalPed3_2==true && (ArrivalTimePed3_2.get(count3_2-1)-sig.startTime>tol)) 
						{
							if (arrivalPed3_1==true)
								waitN3_1++;
							if (arrivalPed3_2==true)
								waitN3_2++;
							
							WaitListPed3.add(time);
							pedevent = true;
						}
						if ((arrivalPed4_1==true || arrivalPed4_2==true) && (time>=tpc && time<tpc+Tpc))	
						{
							if (arrivalPed4_1==true)
								waitN4_1++;
							if (arrivalPed4_2==true)
								waitN4_2++;
							
							WaitListPed4.add(time);
							pedevent = true;
						}
					}
					
				}
		return pedevent;
	}
	
	
    void fixedControl(double time) // problem
	{
		if(time-signal.startTime>=gmax )
			signal.switchSignal(time);
	}	 
	void givenControl(double time) // problem
	{
		if(!Signals.isEmpty()&&time>=Signals.get(0).startTime && signal.phase != Signals.get(0).phase)
		{
			signal.switchSignal(time);
			Signals.remove(0);
		}
		else if(!Signals.isEmpty()&&time>=Signals.get(0).startTime)
			Signals.remove(0);
	}	 
	void adaptiveControl(double time) // compared one in the paper: actuated algorithm
	{
		
		if(time > signal.startTime + gmax - Tpc) {
			{
				if (flagFDW==false)
				{
				   tpc=time; //time of FDW
				   flagFDW=true;
				}
				if (time > signal.startTime + gmax) {
					signal.switchSignal(time);
					pedestrianDischarge (time);
					flagFDW=false;
				}
			}		
		}  		
		else if((time - Math.max(Lane.lastPassingTime[signal.phase - 1], signal.startTime) > actuatedGap) && (time-signal.startTime>gmin -Tpc))//Biao: initial actuatedGap=5  is the vehicles gap equal to 65/vmax. it is not gmin.
		{
			if (flagFDW==false) {
				tpc=time; //time of FDW
				actuatedGap=0;
				flagFDW=true;
			}
			
			if(time-signal.startTime>gmin) {
				signal.switchSignal(time); 
				pedestrianDischarge (time);
				actuatedGap=5;
				flagFDW=false;
			}
		}
		
		pedestrianArrival (time);

	}	 
	
	ArrayList<ArrayList<Car>> carsBylane ()
	{
		ArrayList<ArrayList<Car>> allcars = new ArrayList<ArrayList<Car>>();
		ArrayList<Car> currentCars = new ArrayList<Car>();
		ArrayList<Car> carsLane1 = new ArrayList<Car>();
		ArrayList<Car> carsLane2 = new ArrayList<Car>();
		ArrayList<Car> carsLane3 = new ArrayList<Car>();
		ArrayList<Car> carsLane4 = new ArrayList<Car>();
		
		ArrayList<Car> carsCom1 = new ArrayList<Car>();
		ArrayList<Car> carsCom2 = new ArrayList<Car>();
		
		for(int i=4;i>=1;i--)
		{
			Lane lane = null;
			if (i==1)
				lane=lane1;
			if (i==2)
				lane=lane2;
			if (i==3)
				lane=lane3;
			if (i==4)
				lane=lane4;

			
			for(int j = lane.carList.size()-1;j>=0;j--)
			{
				Car car = lane.carList.get(j);
				if(car.location>=-zoneLength && car.location<=0)
				{
					if(car.equipped)
					{
						// if car is equipped, then add car to the list
						revealed.set(car.idxList, true);
						if(car.speed<1)
						{
							// if car stopped, add all the cars ahead of this car in the zone to the list
							for(int k = j-1;k>=0;k--)
							{
								if(lane.carList.get(k).location<=0)
									revealed.set(lane.carList.get(k).idxList,true);
							}
						}
					}

					if(revealed.get(car.idxList))// cars can be known
					{
						
						if (i==1)
							carsLane1.add(0,car);
						if (i==2)
							carsLane2.add(0,car);
						if (i==3)
							carsLane3.add(0,car);
						if (i==4)
							carsLane4.add(0,car);
						//currentCars.add(0,car);
					}
				}
			}
		}
        // make lanes combination to be a phase
		for (int com=1; com<=2; com++)
		{
			if (com==1)
			{
				carsCom1.addAll(carsLane1);
				carsCom1.addAll(carsLane3);
				Collections.sort(carsCom1, new ArrivalComparator());		
			}
			
			if (com==2)
			{
				carsCom2.addAll(carsLane2);
				carsCom2.addAll(carsLane4);
				Collections.sort(carsCom2, new ArrivalComparator());				
			}
		}
		currentCars.addAll(carsCom1);
		currentCars.addAll(carsCom2);
		
		ArrayList<Car> currentCars_sort = new ArrayList<Car>();
		int vehlimit = 18;// 20 for time compare	
		// way1: this is better than original version if we consider limited vehicles.
		if(currentCars.size()>vehlimit)
		{
			currentCars_sort.addAll(currentCars);		
			Collections.sort(currentCars_sort, new ArrivalComparator());					
			double lastEntertime = currentCars_sort.get(vehlimit-1).enterTime;

			while(currentCars_sort.get(currentCars_sort.size()-1).enterTime>lastEntertime) {
				currentCars_sort.remove(vehlimit);
			}  
			while(true && !carsCom1.isEmpty())
			{
				Car e = carsCom1.get(carsCom1.size()-1);
				if(currentCars_sort.indexOf(e)<0)
					carsCom1.remove(e);
				else
					break;
			}

			while(true && !carsCom2.isEmpty())
			{
				Car e = carsCom2.get(carsCom2.size()-1);
				if(currentCars_sort.indexOf(e)<0)
					carsCom2.remove(e);
				else
					break;
			}
		    currentCars.clear();
		    currentCars.addAll(carsCom1);
		    currentCars.addAll(carsCom2);
		}
	
		// way2: original way. 
//		if(currentCars.size()>vehlimit)  // set a limit of cars for calculation
//		{
//			while(currentCars.size()>vehlimit) {
//				currentCars.remove(vehlimit);
//			}
//			//System.out.println("truncated");
//			while(true && !carsCom1.isEmpty())
//			{
//				Car e = carsCom1.get(carsCom1.size()-1);
//				if(currentCars.indexOf(e)<0)
//					carsCom1.remove(e);
//				else
//					break;
//			}
//
//			while(true && !carsCom2.isEmpty())
//			{
//				Car e = carsCom2.get(carsCom2.size()-1);
//				if(currentCars.indexOf(e)<0)
//					carsCom2.remove(e);
//				else
//					break;
//			}
//			
//		}
		
		allcars.add(currentCars);
		allcars.add(carsCom1);
		allcars.add(carsCom2);
	  

		
		return
			allcars;
	}
	
	
	void CVControl(double time)  // Enumeration method 
	{
		executionSignal (time);// execution process
		
		int lowerInt=0;
        if (vehflow >= 1500)
        	lowerInt=2;  // avoid too many frequencies of event triggering in high demand.
        
		if(CVTrigger(time) && time-pretime>lowerInt)// decision process  
		{
			pretime=time;
//			System.out.println(time);
//			System.out.println(tpc);
//			System.out.println(signal.startTime);
					
			ArrayList<Car> currentCars = new ArrayList<Car>();
			ArrayList<Car> carsCom1 = new ArrayList<Car>();
			ArrayList<Car> carsCom2 = new ArrayList<Car>();

			ArrayList<ArrayList<Car>> allcars=carsBylane();
			currentCars.addAll(allcars.get(0));
			carsCom1.addAll(allcars.get(1));
			carsCom2.addAll(allcars.get(2));
				
			int numcars = carsCom1.size()+carsCom2.size();
			int numcmbs = nchoosek(carsCom1.size()+carsCom2.size(),carsCom1.size());
			ArrayList<ArrayList<Integer>> combinations = cmbbnk(carsCom1.size()+carsCom2.size(),carsCom1.size());
			
			ArrayList<Double> designedSpeed = new ArrayList<Double>(); 
			double delay = 1000000000.0;
			double ta_opt =0.0;
			double Tb_opt= 0.0;
			double pedDelay_opt =0.0;
			double vehDelay_opt = 0.0;
				
			ArrayList<Double> deptime_opt = new ArrayList<Double>(); 
			ArrayList<Integer> platoon_opt = new ArrayList<Integer>();
			ArrayList<Double> signalStartTime_opt = new ArrayList<Double>();
			boolean improve=false;
			
			if (currentCars.isEmpty()) {
				if (signal.phase==1) 
					sigMandatory=2; // next phase is mandatorily started.
				else
					sigMandatory=1;
			}
			
			
			double maxv=0;
			double maxp=0;        		  			
			for(int cmbcount = 0; cmbcount < numcmbs; cmbcount ++)
			{ 

				ArrayList<Integer> depapp = new ArrayList<Integer>();
				ArrayList<Integer> depseq = new ArrayList<Integer>();
				ArrayList<Integer> platoon = new ArrayList<Integer>();
				ArrayList<Integer> platoon1 = new ArrayList<Integer>();
				ArrayList<Integer> platoon2 = new ArrayList<Integer>();
				ArrayList<Integer> platoon3 = new ArrayList<Integer>();
				ArrayList<Integer> platoon4 = new ArrayList<Integer>();

				ArrayList<Double> deptime = new ArrayList<Double>();
				ArrayList<Double> speed = new ArrayList<Double>();
				signalLight sig = signal.copy();



				double[] leadspeed = new double[4];
				leadspeed = leadSpeed.clone();

				for(int i=0;i<numcars;i++)
					depapp.add(2);
				if(!combinations.isEmpty())
					for(int idx:combinations.get(cmbcount))
						depapp.set(idx-1,1);  

				double lastdeptime =0.0;
				double lastdeptime1 = 0.0;
				double lastdeptime2 =0.0;

				if (lastApproach!=0)
				{
					if (sig.phase != (2-lastApproach%2)) {
						lastdeptime1 = Math.max(lastDepTime1, lastDepTime2);
						lastdeptime2 = Math.max(lastDepTime1, lastDepTime2);
					}
					else
					{
						lastdeptime1 = lastDepTime1;
						lastdeptime2 = lastDepTime2;
					}
				}
				//////////////
				int numC1=0, numC2=0;
				for(int i=0;i<numcars;i++) {

					if (depapp.get(i)==1)
						numC1++;  //record number of cars in Com1
					else
						numC2++; //record number of cars in Com2

					if(i==0) {
						if(depapp.get(i) == (2-lastApproach%2)) // phase 1 (app 1,3) or 2 (app 2,4)				
						{ 									
							if (depapp.get(i) ==1) // lane 1 or 3
							{
								if (carsCom1.get(0).approach==1) {									
									platoon1.add(lastPlatoon1+1);
									platoon3.add(lastPlatoon2);
									platoon.add(lastPlatoon1+1);	
								}								    
								else {	
									platoon1.add(lastPlatoon1);
									platoon3.add(lastPlatoon2+1);
									platoon.add(lastPlatoon2+1);																
								}																	

							}
							if (depapp.get(i) ==2)
							{
								if (carsCom2.get(0).approach==2) {									
									platoon2.add(lastPlatoon1+1);
									platoon4.add(lastPlatoon2);
									platoon.add(lastPlatoon1+1);				
								}								    
								else {			
									platoon2.add(lastPlatoon1);
									platoon4.add(lastPlatoon2+1);
									platoon.add(lastPlatoon2+1);										
								}	
							}		
						}					
						else { 
							if (depapp.get(i)==1)
								if (carsCom1.get(0).approach==1)
									platoon1.add(1);
								else
									platoon3.add(1);

							if (depapp.get(i)==2)
								if (carsCom2.get(0).approach==2)
									platoon2.add(1);
								else
									platoon4.add(1);
							platoon.add(1); }
					}

					// i!=0
					else {
						if(depapp.get(i) == depapp.get(i-1)) {
							int numplatoon;
							if (depapp.get(i)==1)
								if (carsCom1.get(numC1-1).approach==1) {
									if (platoon1.isEmpty())
										numplatoon=1;
									else
										numplatoon = platoon1.get(platoon1.size()-1)+1;
									platoon.add(numplatoon);
									platoon1.add(numplatoon);
								}
								else {
									if (platoon3.isEmpty())
										numplatoon=1;
									else
										numplatoon = platoon3.get(platoon3.size()-1)+1;
									platoon.add(numplatoon);
									platoon3.add(numplatoon);
								}
							else
								if (carsCom2.get(numC2-1).approach==2) {
									if (platoon2.isEmpty())
										numplatoon=1;
									else
										numplatoon = platoon2.get(platoon2.size()-1)+1;									 
									platoon.add(numplatoon);
									platoon2.add(numplatoon);
								}	
								else {
									if (platoon4.isEmpty())
										numplatoon=1;
									else
										numplatoon = platoon4.get(platoon4.size()-1)+1;
									platoon.add(numplatoon);
									platoon4.add(numplatoon);
								}	
						}
						else { 
							platoon1.clear();
							platoon2.clear();
							platoon3.clear();
							platoon4.clear();
							if (depapp.get(i)==1)
								if (carsCom1.get(numC1-1).approach==1)
									platoon1.add(1);
								else
									platoon3.add(1);

							if (depapp.get(i)==2)
								if (carsCom2.get(numC2-1).approach==2)
									platoon2.add(1);
								else
									platoon4.add(1);
							platoon.add(1); }
					}
				}

				int pos1 =0, pos2 =0; 
				for(int i = 0; i<numcars; i++)
				{
					if(depapp.get(i)==1)
					{
						depseq.add(pos1);
						pos1 ++;
					}
					else
					{
						depseq.add(carsCom1.size()+pos2);
						pos2 ++;
					}
				}
				double Delay = 0.0;
				double vehDelay = 0.0; 
				double pedDelay = 0.0;
				////////////////
				NS = 0; 
				sigstartTime.clear();
				sigstartTimePed1.clear();
				sigstartTimePed2.clear();		
				tw_last= signal.startTime; // for NS=0

				for(int i=0;i<numcars;i++)
				{	
					// assume signal change for delay estimate
					if (currentCars.get(depseq.get(i)).approach==1 || currentCars.get(depseq.get(i)).approach==2)
						lastdeptime=lastdeptime1;
					else
						lastdeptime=lastdeptime2;

					if ((i==0 && depapp.get(i) != (2-lastApproach%2)) || (i>0 && depapp.get(i-1)!=depapp.get(i))) {
						lastdeptime=Math.max(lastdeptime1, lastdeptime2);
					}

					//					
					Trajectory TrajectoryCV = new Trajectory(time,  lastdeptime,  sig,  currentCars.get(depseq.get(i)),  depapp.get(i),   platoon.get(i),  leadspeed[currentCars.get(depseq.get(i)).approach-1]); // "sig" is changed very time here.
					deptime.add(TrajectoryCV.deptime);
					speed.add(TrajectoryCV.speed);
					leadspeed[currentCars.get(depseq.get(i)).approach-1]=TrajectoryCV.leadspeed;			

					///////////////////// for pedestrian delay calculation /////////////
					if (TrajectoryCV.NS!=0)
					{
						NS += TrajectoryCV.NS;
						tw_last = TrajectoryCV.tw_last;
						if (TrajectoryCV.NS==2)
						{
							sigstartTime.add(tw_last-(G_pedmin+Tpc));
							sigstartTime.add(tw_last);
						}
						else
							sigstartTime.add(tw_last);
					}
					///////////////////////	/////////////////////////////////////////////

					vehDelay += (deptime.get(i)-currentCars.get(depseq.get(i)).virtualArrivalTime);// for pedestrian control model

					//System.out.println(vehDelay);
					//subdelay += deptime.get(i);// original CV
					//lastdeptime = deptime.get(i);// 2-way

					// 4-way
					if (i>0 && depapp.get(i-1)!=depapp.get(i))// initialization
					{
						lastdeptime1=lastdeptime;
						lastdeptime2=lastdeptime;
					}
					if (currentCars.get(depseq.get(i)).approach==1 || currentCars.get(depseq.get(i)).approach==2)//set
						lastdeptime1=deptime.get(i);
					else
						lastdeptime2=deptime.get(i);	
					/////////////////	
				}
				//if(subdelay<delay+1e-3) // original CV

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//// pedestrian delay calculation
				double[] tax = new double[4];		
				double minta= (double)Integer.MAX_VALUE;
				

				if (Wped>=0) {
					ta = time; // time of triggered event
					// need to discuss and to confirm					
					if (!WaitListPed1.isEmpty())
						tax[0]=WaitListPed1.get(0);
					else
						tax[0]=ta;
					if (!WaitListPed2.isEmpty())
						tax[1]=WaitListPed2.get(0);
					else
						tax[1]=ta;
					if (!WaitListPed3.isEmpty())
						tax[2]=WaitListPed3.get(0);
					else
						tax[2]=ta;
					if (!WaitListPed4.isEmpty())
						tax[3]=WaitListPed4.get(0);
					else
						tax[3]=ta;


					for (int num = 0; num < tax.length; num++) {
						if(minta>tax[num])
							minta=tax[num];
					}	

					/*if (time==25.9)
					  System.out.println(1);*/

					if (numcars==0)
						tb = ta;
					else
						tb = Math.max(lastdeptime1, lastdeptime2);



					signalLight sig0 = signal.copy();
					if (!sigstartTime.isEmpty())
					{
						if (sig0.phase==2)
						{
							for (int i=0; i < sigstartTime.size(); i++)
							{
								if (i%2==0)
									sigstartTimePed2.add(sigstartTime.get(i));
								else
									sigstartTimePed1.add(sigstartTime.get(i));
							}						
						}
						else
						{
							for (int i=0; i < sigstartTime.size(); i++)
							{
								if (i%2==0)
									sigstartTimePed1.add(sigstartTime.get(i));
								else
									sigstartTimePed2.add(sigstartTime.get(i));
							}	
						}
					}
					pedestrianDelay pD = new pedestrianDelay(sig0, flowPed1, flowPed2, flowPed3, flowPed4,  tax, ta,  tb, lastTb, tpc,  tw_last,  NS,  WaitListPed1,  WaitListPed2, WaitListPed3,  WaitListPed4,  sigstartTimePed1,  sigstartTimePed2);
					pedDelay = pD.delay;
					Tb = pD.Tb;
					
					//option 1: objective function is total delay
//					Delay = Wveh * vehDelay + Wped * pedDelay;	
					
					//option 2: objective function is average delay	
					double numpeds= (tb-ta)*(flowPed1+flowPed2+ flowPed3+ flowPed4)*2;
					if (numcars!=0 && numpeds!=0)		
					     Delay = Wveh * vehDelay/numcars + Wped * pedDelay/numpeds;
					else if (numcars!=0 && numpeds==0)
						 Delay = vehDelay/numcars;
					else if (numcars==0 && numpeds!=0)
						 Delay = pedDelay/numpeds;
				}
				else
					Delay = vehDelay;

				 
				if(Delay < delay )// 
				{
//					System.out.println(time);
//					System.out.println(Delay);
//				    System.out.println(vehDelay);
//					System.out.println(pedDelay);
					//platoon_opt=(ArrayList<Integer>) platoon.clone();

					// update the sort :  better way to save time?
					double min, value; 
					int idx, idxvalue;
					int[] index = new int[deptime.size()];
					for (int i=0; i< deptime.size(); i++)
						index[i]=i;

					for(int i = 0; i < deptime.size(); i++){
						min  = deptime.get(i);
						idx=i;

						for(int j = i + 1; j < deptime.size(); j++){
							if(min > deptime.get(j)){
								min = deptime.get(j);
								idx= j;
							}
						}
						value = deptime.get(i); 
						deptime.set(i, min); 
						deptime.set(idx, value); 
						idxvalue = index[i]; 
						index[i] = index[idx];
						index[idx] = idxvalue;
					}
					
					int nextPhase = 0;
					if (numcars!=0) {
						if(currentCars.get(depseq.get(index[0])).approach==1 || currentCars.get(depseq.get(index[0])).approach==3)
							nextPhase = 1;
						else
							nextPhase = 2;
					}
                    
					//if (flagFDWSwitch==false || (flagFDWSwitch==true && (nextPhase==inextPhase) && numcars!=0)) {	//option1: no decision planning during FDW  when event maybe happens, but the trajectories of no first vehicles can be improved (leading to less delay).
					if (flagFDWSwitch==false) { // option2: no decision planning during FDW when event maybe happens
						improve=true;
						
						DepartureList.clear();
						ApproachList.clear();
						designedSpeed.clear();
						
					
						for(int i = 0; i < numcars; i++)
						{
							DepartureList.add(currentCars.get(depseq.get(index[i])).id);
							ApproachList.add(currentCars.get(depseq.get(index[i])).approach); 
							designedSpeed.add(speed.get(index[i]));
						}
					}
				

					delay = Delay;
					lastTb = Tb;

					// save for analysis
					ta_opt = minta;
					Tb_opt = Tb;
					vehDelay_opt = vehDelay;
					pedDelay_opt = pedDelay;

					deptime_opt.clear();
					deptime_opt=(ArrayList<Double>) deptime.clone();
					signalStartTime_opt.clear();
					signalStartTime_opt=(ArrayList<Double>) sigstartTime.clone();	

				}
			}
		//}
			
		/*	if (time>305)
				System.out.println(platoon_opt);*/
			//System.out.println(delay);
			//System.out.println(deptime_opt);
			
		if (improve==true) {
			
			// save for analysis
			if (Wped>=0) {
				timeList.add(time);
				taList.add(ta_opt);
				TbList.add(Tb_opt);
				prevehDelayList.add(vehDelay_opt);
				prepedDelayList.add(pedDelay_opt);		
			}
			
			// predictions of FDW time stamp from optimal departure sequence.
			preTimeFDW.clear();		
			
//			if (time>266)
//			{
//				System.out.println(time);
//				System.out.println(signalStartTime_opt);
//			}
				
			
			if (!signalStartTime_opt.isEmpty() ) {
				for (int i=0; i<signalStartTime_opt.size(); i++)
				{
					preTimeFDW.add(signalStartTime_opt.get(i)-Tpc);
				}
			}
			else
				preTimeFDW.add(0.0);
			   // System.out.println(preTimeFDW);			
             ///////////////////////////////
			    
			    
			ArrayList <Integer> vehIDList = new ArrayList <Integer>();
			for(int i=0; i<DepartureList.size();i++)				
			{
				// assign designed speeds
				vehIDList.add(DepartureList.get(i));
				switch (ApproachList.get(i))
				{
				case 1:
					lane1.assignSpeed(DepartureList.get(i), designedSpeed.get(i));
					break;
				case 2:
					lane2.assignSpeed(DepartureList.get(i), designedSpeed.get(i));
					break;
				case 3:
					lane3.assignSpeed(DepartureList.get(i), designedSpeed.get(i));
					break;
				case 4:
					lane4.assignSpeed(DepartureList.get(i), designedSpeed.get(i));
					break;
				}
				
				//////
				if(ApproachList.get(0)==1 || ApproachList.get(0)==3)
					inextPhase = 1;
				else
					inextPhase = 2;
			}
			
			prevehIDList.add(vehIDList);		
		}
			
			
		}// trigger
	
	}	  
	
	
    
	private String dirname()
	{
		DecimalFormat df1=new DecimalFormat("0.0");		
		
			return "result/vehflow=" + df1.format(vehflow) + "_pedflow=" + df1.format(pedflow) + "/vehRatio="+df1.format(vehflowRatio) + "/pedRatio="+df1.format(pedflowRatio)+
				"/pr_con="+df1.format(pr_con)+"/pr_auto="+ df1.format(pr_auto)+"/";

	}	
	
    public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			@Override
			public void run() {
					
				int out_perform=1; //performance analysis: 0 for close; 1 for test; 2 for average delay and stops for full AVs; 3 for penetration rates of CAV_information level; 4 for CAV_ automation level.
				int out_cor = 0; //correlation analysis:  0 for close; 1 for start.
				int out_com = 0;//comparison of computation time between ACS and XX:  0 for close; 1 for start.
				int out_sen = 0; // sensitive analysis of ACS: 0 for close; 1 for e0; 2 for alpha & rho; 3 for beta.
				int[] outputs = {out_perform,out_cor,out_com,out_sen};
				
				float [] wped = { 0.0f, 0.05f, 0.1f, 0.15f, 0.2f,0.25f, 0.3f, 0.35f, 0.40f, 0.45f, 0.5f}; //
	
				
				//one example test
				//for (int i3=0; i3< 10; i3++) {
					//CVSpeedIDM idm = new CVSpeedIDM(0, 800/3600.0, 1, 0.0f, 1, 1, 800/3600.0, 1, true, 4, outputs) ;    // options, 1(FC),2(CAVs enumeration method),3(given control),4(actuated control),5(ACS control)    
					CVSpeedIDM idm = new CVSpeedIDM(0, 800/3600.0, 1, 0.0f, 1, 1, 800/3600.0, 1, false, 4, outputs) ; 
				    idm.start(); 
				//}
				System.out.println("Simulation Done!");
				
						
			}

		});
	}
	 
}
