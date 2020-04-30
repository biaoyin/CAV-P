import java.util.ArrayList;


///////////////////////Branch and Bound////////////////////////////	
	public class Node{
		
		// Car following parameters
		double amax = Parameter.amax; //maximum acceleration 0.73
		double ddes = Parameter.ddes; // desired deceleration
		double delta = Parameter.delta; // acceleration exponent
		double minSpacing = Parameter.minSpacing; //minimum spacing
		double s0 = minSpacing; // jam distance;
		double vdes = Parameter.vdes; // desired speed m/s
		double TReact = Parameter.TReact; // driver reaction time
		static double a = Parameter.a; // acceleration rate for crossing the intersection
		static double vf = Parameter.vf; // speed
		double length = Parameter.length; // car length
		double signalStart = Parameter.signalStart;
		static double cap = Parameter.cap; // saturation flow veh/s
		double zoneArrive = Parameter.zoneArrive; //  length of approach: from just arriving vehile to stopline
		static double gmax = Parameter.gmax; // maximum green duration(s)
		static double gmin = Parameter.gmin; // minimum green duration(s)
		double followGap = Parameter.followGap; // m
		double speedVar = Parameter.speedVar;
		static double vmin = Parameter.vmin;	// minimum of cruise vehicle speed
		static double lenIS = Parameter.lenIS;  // length of intersection
		
		ArrayList<Car> Cars1 = new ArrayList<Car>();
		ArrayList<Car> Cars2 = new ArrayList<Car>();
		
		ArrayList<Integer> currentCarList = new ArrayList<Integer>();
		ArrayList<Integer> currentOptList = new ArrayList<Integer>();

		double subdelay = 0.0;
		double [] leadSpeed = {vf,vf};
		int lastApp;
		int lastPlatoon;
		double time;
		double lastDepTime;
		signalLight signal;
		double LB;
		double UB;

		public Node(double sd, int plt, int app, double tm, double [] ls, double lastdeptime, double lb, double ub, signalLight ss,
				ArrayList<Car> cars1, ArrayList<Car> cars2, ArrayList<Integer> ccl, ArrayList<Integer> col)
		{
			time = tm;
			leadSpeed[0] = ls[0];
			leadSpeed[1] = ls[1];
			subdelay = sd;
			lastPlatoon = plt;
			lastApp = app;
			lastDepTime = lastdeptime;
			for(Car i:cars1)	
				Cars1.add(i);
			for(Car i:cars2)	
				Cars2.add(i);
			for(Integer i:ccl)
				currentCarList.add(i);
			for(Integer i:col)
				currentOptList.add(i);
			
			signal = ss.copy();
			LB = lb;
			UB = ub;
			double delay1 = subdelay, delay2 = subdelay;
			double Delay1 = subdelay, Delay2 = subdelay;
		
			ArrayList<Car> carlist1 = new ArrayList<Car>();
			ArrayList<Car> carlist2 = new ArrayList<Car>();
			for (Car i:Cars1)
				carlist1.add(i);
			for (Car i:Cars2)
				carlist1.add(i);
			for (Car i:Cars2)
				carlist2.add(i);
			for (Car i:Cars1)
				carlist2.add(i);
				
           
			delay1 = calDelay(lastPlatoon, lastApp, lastDepTime, time, leadSpeed, carlist1, signal, true);// for LB calculation
			Delay1 = calDelay(lastPlatoon, lastApp, lastDepTime, time, leadSpeed, carlist1, signal, false);// for UB calculation
			delay2 = calDelay(lastPlatoon, lastApp, lastDepTime, time, leadSpeed, carlist2, signal, true);// for LB calculation
			Delay2 = calDelay(lastPlatoon, lastApp, lastDepTime, time, leadSpeed, carlist2, signal, false);// for UB calculation
			
			//System.out.println(((Double)delay1).toString());

			if(delay1<delay2)
			{
				LB = delay1+subdelay;
			}
			else 
				LB = delay2 + subdelay;
			
			if(Delay1<Delay2 && Delay1 + subdelay <UB )
			{
				UB = subdelay+Delay1;
				currentOptList.clear();
				currentOptList.addAll(ccl);
				for (int i=0; i<carlist1.size();i++)
					currentOptList.add(carlist1.get(i).id);

				
			}
			else if(Delay1>=Delay2 && Delay2 + subdelay <UB)
			{
				UB = subdelay +Delay2;
				currentOptList.clear();
				currentOptList.addAll(ccl);
				for (int i=0; i<carlist2.size();i++)
					currentOptList.add(carlist2.get(i).id);


			}

		}
		
		
		public ArrayList<Node> generateChildren(double delay)
		{
			ArrayList<Node> children = new ArrayList<Node>();
			int platoon;
			double depTime = 0.0;

			signalLight sig = signal.copy();
			
			// for the first lane
			if(!Cars1.isEmpty())
			{
				int depapp=1;
				if (lastApp == depapp)
					platoon = lastPlatoon + 1;
				else
					platoon = 1;
				Trajectory TrajectoryBB = new Trajectory(time,  lastDepTime,  sig,  Cars1.get(0),  depapp,  platoon,  leadSpeed[0]);
				depTime = TrajectoryBB.deptime;
				leadSpeed[0] = TrajectoryBB.leadspeed;
				
				
				ArrayList<Integer> ccl = new ArrayList<Integer>();
				for(int i = 0;i<currentCarList.size();i++)
				{
					ccl.add(currentCarList.get(i));
				}
				ccl.add(Cars1.get(0).id);
				
				Node n1 = new Node(subdelay+depTime, platoon, 1, time, leadSpeed, depTime, LB, Math.min(UB, delay), sig, new ArrayList<Car> (Cars1.subList(1, Cars1.size())), Cars2, ccl, currentOptList) ;
				children.add(n1);
			}
			
			// for the second lane
			sig = signal.copy();
			if(!Cars2.isEmpty())
			{
				int depapp=2;
				if (lastApp == depapp)
					platoon = lastPlatoon + 1;
				else
					platoon = 1;
				
				Trajectory TrajectoryBB = new Trajectory(time,  lastDepTime,  sig,  Cars2.get(0),  depapp,  platoon,  leadSpeed[1]);
				depTime = TrajectoryBB.deptime;
				leadSpeed[1] = TrajectoryBB.leadspeed;
				
				
				
				ArrayList<Integer> ccl = new ArrayList<Integer>();
				for(int i =0;i<currentCarList.size();i++)
				{
					ccl.add(currentCarList.get(i));
				}
				ccl.add(Cars2.get(0).id);
				Node n2 = new Node(subdelay+depTime, platoon, 2, time, leadSpeed,depTime, LB, Math.min(UB, delay),sig, Cars1, new ArrayList<Car> (Cars2.subList(1, Cars2.size())), ccl, currentOptList) ;
				children.add(n2);
			}
			return children;
		}
		
		double calDelay(int lastplatoon, int lastApp, double lastDepTime, double time, double[] leadSpeed, ArrayList<Car> carlist, signalLight signal, boolean fl)
		{
			//important:  must create  new variables, otherwise, the results leadspeed, lastapp will change the direct variables.
			int platoon = lastplatoon;
			double lastdeptime = lastDepTime;
			int lastapp = lastApp;
			double [] leadspeed = {leadSpeed[0], leadSpeed[1]};		
			signalLight sig = signal.copy();
					
			boolean flag  = fl;	
			double delay = 0.0;
			for(int i = 0; i<carlist.size();i++)
			{
				int depapp = carlist.get(i).approach;
		    	if (lastapp==depapp)
		    		platoon = platoon + 1;
		    	else
		    		platoon = 1;
		    	
				Trajectory TrajectoryBB = new Trajectory(time,  lastdeptime,  sig,  carlist.get(i),  depapp,  platoon,  leadspeed[depapp-1],  flag);
				delay += TrajectoryBB.deptime;
				leadspeed[depapp-1]=TrajectoryBB.leadspeed;
				lastapp = carlist.get(i).approach;	
				lastdeptime = TrajectoryBB.deptime;				
			}
			return delay;
		}
		
	}
	////////////////////////////////////////////////////