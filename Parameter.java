

public class Parameter {
	// Car following parameters
	static double amax = 1.8; //maximum acceleration 0.73
	static double ddes = 3; // desired deceleration
	static double delta = 4; // acceleration exponent
	static double minSpacing = 2.0; //minimum spacing
	static double s0 = minSpacing; // jam distance;
	static double vdes = 60/3.6; // desired speed m/s
	static double TReact = 1.4; // driver reaction time
	static double a = amax; // acceleration rate for crossing the intersection
	static double vf = 15; // free speed
	static double length = 5.0; // car length
	static double signalStart = 0.0;
	static double cap = 1/2.3; // saturation flow veh/s
	static double zoneArrive = 200; //  length of approach: from just arriving vehile to stopline
	static double gmax = 45; // maximum green duration(s)
	//static double gmin = 5.0; // minimum green duration(s)
	static double followGap = 15; // m
	static double speedVar = 0.0;
	static double vmin = 8.0/3.6;	// minimum of cruise vehicle speed
	// car in panel;
	static int h = 3; // length of car in y coordinate (pixel);
	static int w = 5; // length of car in x coordinate (pixel);
	
	// intersection configuration parameters
	static double zoneLength = 100; // communication range
	static double loopLocation = -65.0; // loop for actuated control
	static boolean visual = true;
	static double wl = 20; // default value if vehicle length
	static double lenIS = 2*5;  // length of intersection
	static double ignoreProbability = 0.1;	
	static double zone = -100; // starting location of the zone of interest;
	static double start = -400; // where the car is initialized;
	static double signalLocation = 0.0; // where the signal light is;
	
	// parameter for algorithms
	static double [] leadSpeed = new double[2]; 
	static int distribution = -1; // 1 means Gaussian distribution 
	static double shape = 2; // shape parameter used for Gaussian distribution, it is the standard deviation (m/s)
	static int lastPlatoon = 1;
	static int lastApproach = 0;
	
    // parameter for simulation
	static int c = 400;	// Nb of vehicles generated at intersection
	static int timeStep=25;	// simulation 0.025s per step default 25; if make it bigger (100), then faster yet little difference appears
	static boolean calibrate = false;
	
	
	//parameter for pedestrains
	static double Tar = 0.01; //no all-red signal in original version? we try to use to be 2s
	static double G_pedmin = 5.0;// minimum pedestrian crossing time = FDW interval
	static double g_pedmin = G_pedmin - 0.0; // minimum pedestrian signal green time
	static double Tpc = G_pedmin + Tar; // time pedestrian clearance
	static double gmin = g_pedmin + Tpc; // minimum vehicle signal green time time
	
}
