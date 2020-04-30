
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

public class Lane{
		/**
		 * if run testJunction.java, use testJunction to replace CVSpeedIDM.java 
		 */
	    double minSpacing = Parameter.minSpacing; //minimum spacing
	    double length = Parameter.length;
	    double loopLocation = Parameter.loopLocation; // loop for actuated control
	    double lenIS = Parameter.lenIS; // size of the intersection
	    boolean calibrate = Parameter.calibrate;
	    double vf = Parameter.vf;
	    int distribution = Parameter.distribution;
	    double followGap = Parameter.followGap; // m
	    double shape = Parameter.shape; // shape parameter used for Gaussian distribution, it is the standard deviation (m/s)
		// class of each lane
	    int approach;
		int phase; // signal phase
		signalLight signal; // the signal for this lane
		double zone = Parameter.zone; // starting location of the zone of interest;
		double start = Parameter.start; // where the car is initialized;
		double signalLocation = Parameter.signalLocation; // where the signal light is;
		
		int signalPlan = CVSpeedIDM.signalPlan; // 1 means fixed time; 2 means CV contro
		//int signalPlan = testJunction.signalPlan;
    	static double [] lastPassingTime = new double[2]; // for apporach 1 and 2
		
		ArrayList<Car> carList = new ArrayList<Car>(); // current car list on this lane
		ArrayList<Double> assignedSpeed = new ArrayList<Double>(); // assigned speed of each car, same length as carList
		
		Lane(int lane, signalLight s)
		{
			approach= lane;
			if (lane==1 || lane==3)
				phase = 1;
			if (lane==2 || lane==4)
				phase = 2;
			signal = s;
		}
		
		boolean assignSpeed(int vid, double v) // assign speed to vehicle with id vid: first looking for vid, and then assign speed.
		{
			int idx=-1;
			for(int i = 0;i<carList.size(); i++)
			{
				if(carList.get(i).id==vid)
				{
					idx = i;
					break;
				}
			}
			
			if(idx<0)
				return false;
			else
			{
				assignedSpeed.set(idx,v);
				return true;
			}
		}
		
		void addCar(Car c) // add car to the list
		{	
			if(carList.size()>0 && carList.get(carList.size()-1).id == c.id)
				return;
			if(carList.size() > 0 &&carList.get(carList.size()-1).location-c.location-carList.get(carList.size()-1).length < minSpacing)
			{
				c.location = carList.get(carList.size()-1).location-carList.get(carList.size()-1).length-minSpacing;
				c.prevLocation = c.location;
				c.speed = carList.get(carList.size()-1).speed;
			}
			carList.add(c);
			assignedSpeed.add(-1.0);
		}
		
		Car findCar(int cid) //find car with id cid and return Car 
		{
			int idx = -1;
			for(int i = 0;i<carList.size();i++)
			{
				if(carList.get(i).id==cid)
					idx = i;
			}
			
			if(idx >=0)
				return carList.get(idx);
			else
				return null;			
		}
		
		boolean isISEmpty() //check if the intersection is empty or not
		{
			boolean ISEmpty = true;
			for(int i =0;i<carList.size();i++)
			{
				if(carList.get(i).location<lenIS && carList.get(i).location>1e-3) {
					ISEmpty = false;
					break;// add by yin
				}
			
			}
			return ISEmpty;
		}
		void carRun(double time) // at each time step, let the vehicles on the lane run
		{
			boolean firstCarInZone;
			for(int i = 0; i < carList.size(); i ++)
			{				
				if((signalPlan == 4) && carList.get(i).location>loopLocation-3 && carList.get(i).location-length<=loopLocation)
				{
					lastPassingTime[phase-1] = time;
					//System.out.println(time);
				}

				if(carList.get(i).location>lenIS && carList.get(i).prevLocation<=lenIS)// just leave the intersection (far side)
				{
						String str = ((Integer)carList.get(i).id).toString()+","+
								((Double)carList.get(i).virtualArrivalTime).toString()+","+
								((Double)time).toString()+","+
								((Integer)carList.get(i).numstop).toString() + "\r\n";
						try {
							CVSpeedIDM.resultwriter.write(str);
							//testJunction.resultwriter.write(str);					
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}						
						
				}
				if(  carList.get(i).location <= 0 && (i== 0 || carList.get(i-1).location>0 ))		
				    firstCarInZone = true;// in the communication zone
				else 
					firstCarInZone = false;
				
				if(!carList.get(i).automated && !carList.get(i).equipped) // conventional vehicle
				{
					if(carList.get(i).location > 0 && i ==0) 
						// the first car that already passed the stopline, drive as the first car
						carList.get(i).followFirst();
					else if(firstCarInZone && signal.phase != phase )
						// the first car in zone with red signal, follow red light
						carList.get(i).followRedLight();
					else if(i==0)
						carList.get(i).followFirst();
					else
						// passed the intersection but not the first car, or in the zone but not the first in zone, or first in zone but with green signal, normal follow
						carList.get(i).followNormal(carList.get(i-1));

				}
				else  
				{
					double v=-1.0;
					if(carList.get(i).location>zone && (signalPlan == 2 || signalPlan == 5 || signalPlan == 6))//related cv control methods
						if(!carList.get(i).automated)//equipped
							v = humanDriver(assignedSpeed.get(i), carList.get(i).speed); //byin  default is design speed, or use Gaussian to adjust
						else // automated
							v = assignedSpeed.get(i);
					
					if(v<0)
					{
						if(carList.get(i).location > 0 && i ==0) 
							// the first car that already passed the stopline, drive as the first car
							carList.get(i).followFirst();
						else if(firstCarInZone && signal.phase != phase )
							// the first car in zone with red signal, follow red light
							carList.get(i).followRedLight();
						else if(i==0)
							carList.get(i).followFirst();
						else
							// passed the intersection but not the first car, or in the zone but not the first in zone, or first in zone but with green signal, normal follow
							carList.get(i).followNormal(carList.get(i-1));
					}
					else
					{
						if(!CVSpeedIDM.DepartureList.isEmpty())
						{
							//if(firstCarInZone && (carList.get(i).id!= CVSpeedIDM.DepartureList.get(0) || signal.phase != phase && carList.get(i).location> -followGap))
							if(firstCarInZone &&  (carList.get(i).approach %2 != (CVSpeedIDM.DepartureList.get(0)/10000) %2
									|| signal.phase != phase && carList.get(i).location> -followGap)) // modified by byin
							    carList.get(i).followInstructionRedLight(v);
							else if(i==0)
								carList.get(i).followInstruction(v, null);
							else							
								carList.get(i).followInstruction(v, carList.get(i-1));
						}
					}						
				}			
			}
		}
		
		
		double humanDriver(double vd, double v0)
		{
			/* The parameters are
			 * vd - the designed speed for this car;
			 * v0 - the original speed for this car;
			 */
			
			double v=0.0;
			
			Random random = new Random();
			//option = random.nextInt(10);
			//double x;
			//x = random.nextDouble();
			switch(distribution) {
			case 0:	v = v0;																// ignores instruction	
					// mode = false;
					break; 		
			case 1:	// NormalDistribution gauss = new NormalDistribution();				
					// v = vd + shape *gauss.inverseCumulativeProbability(x);			//	Gaussian distribution
					v = vd + shape *random.nextGaussian() ;				
					if (v > vf) v = vf;
					if (v < 0) v = 0.0;
					// mode = true;	
					break; 
			case 2:	int kappa = 7; double lambda = 0.5;									//	Erlang distribution 
					v= ErlangDistribution (kappa,lambda,vd);	
					if (v > vf) v = vf;
					if (v < 0) v = 0.0;
					// mode = true;
					break; 
			/*case 3: double nu = 5;														//	Student t-distribution
					TDistribution Students = new TDistribution(nu);
					v = vd + shape *Students.inverseCumulativeProbability(x);
					if (v > vf) v = vf;
					if (v < 0) v = 0.0; 
					// mode = true;
					break;*/
					
			default : v = vd;
					break;
			}
			return v;
		}
		
		
		double ErlangDistribution ( double kappa, double lambda,double vd) {	
			double A = 1;
			double f;
			Random y = new Random();	
			for (int i = 1; i < kappa + 1; i++)	{				
				f = y.nextDouble();									//  Erlang-distributed random numbers (wikipedia)
				A = A*f;
			}
			double ErlangRandom =-1/lambda *Math.log(A);
			return vd + ErlangRandom-(kappa-1)/lambda;
		}
	}