import java.util.ArrayList;
import java.util.Random;
import java.util.Vector;

public class Ant implements Cloneable {

		private Vector<Integer> tabu; //
		private Vector<Integer> allowedCars; 
		
		private float[][] distance; 
		//private float[][] distance_cost;

		private float beta;
		private float q0;

		private float tourLength; 
		private int carNum; 

		private int firstCar; 
		private int currentCar; 
		private int virtualCar;

		private Random rand = new Random();
		
		////////////
		
		private double [] leadspeed = new double[4];
		private double [] leadspeed_bak = new double[4];
		
		private double [] lastdeptime= new double[2];
		private double [] lastDeptime= new double[2];
		
		private int lastapproach;
		private int [] lastplatoon= new int[2];
		//private int lastPlatoon;
		
		ArrayList<Double> deptime = new ArrayList<Double>();
		ArrayList<Double> speed = new ArrayList<Double>();
		ArrayList<Integer> platoon = new ArrayList<Integer>();
		ArrayList<signalLight> sig = new ArrayList<signalLight>();
		
		private Vector<Double> tabuSpeed;
		private Vector<Double> depTime;
		private int numcars1=0;
		private int numcars2=0;
		
		ArrayList<Car> curCars = new ArrayList<Car>();
		private signalLight signal;
		
		
		private int NS; 
		private double tw_last;
		private int[] T_NS= new int[2];// store two possibilities of two conflicting approaches
		private double[] T_tw_last = new double[2];
		private   ArrayList<Double> sigstartTime =new ArrayList<Double>();
		double G_pedmin = Parameter.G_pedmin;
		double Tpc = Parameter.Tpc;
		////////////

		public Ant(){
			carNum = 30;
			tourLength = 0;  
		}
		/**
		 * Constructor of Ant
		 * @param num ant
		 */
		public Ant(int num){
			carNum = num;
			tourLength = 0;

		}
		/**
		 * initialize ant deposit ant position at virtual car, namely the last departure car
		 * @param distance 
		 * @param b beta
		 * @param q exploitation ratio
		 */
		public void init(float[][] distance,  float b, float q, int nc1, int nc2, ArrayList<Car> clist, signalLight sig, double[] ls, double[] ldt, int lda, int[] ldp ){
			////
			
			leadspeed=ls.clone();
			leadspeed_bak=leadspeed.clone();
			lastdeptime=ldt.clone();
			lastplatoon=ldp.clone();
			
			lastapproach = lda;					
			numcars1 = nc1;
			numcars2 = nc2;
			curCars = clist; 
			signal = sig;
			
			beta = b;
			q0 = q;
			allowedCars = new Vector<Integer>();
			tabu = new Vector<Integer>();
			this.distance = distance;
			
	        virtualCar = 0;
	        firstCar = virtualCar;

	        //initialize allowedCars
	        if (numcars1!=0 && numcars2!=0) {
	        	allowedCars.add(1);	
	        	allowedCars.add(numcars1+1);  // put first cars of lane 1 and lane 2 into allowedCars (1,2,...); this vector will be changed iteratively.
	        }
	        if (numcars1==0 || numcars2==0)
	        	allowedCars.add(1);

			tabu.add(Integer.valueOf(firstCar)); // store departure sequence
			currentCar = firstCar;
			/////
			tabuSpeed = new Vector<Double>();
			tabuSpeed.add(-1.0);
			
			depTime = new Vector<Double>();
			depTime.add(-1.0);
			
			NS=0;
			tw_last= sig.startTime;
			
			T_NS[0]=0;
			T_NS[1]=T_NS[0];
			T_tw_last[0]= sig.startTime;
			T_tw_last[1]=T_tw_last[0];
		}

		/**
		 * @param pheromone matrix
		 * @param time stamp
		 */
		public void selectNextCar(float[][] pheromone, double time){
			deptime.clear();
			speed.clear();
			platoon.clear();
		    sig.clear();
			/////////////////////////////calculate cost-to-go by departure time	/////////////////////////
			leadspeed_bak=leadspeed.clone();// backup leadspeed
			// platoon setting
		    for (int j=0; j<allowedCars.size(); j++)// this is a trail test other than a series.  
		    {
		    	int i = allowedCars.get(j);
		    	int depapp = curCars.get(i-1).approach;
		    	int depphase = 2-depapp%2;
		    	sig.add(signal.copy());
		    	
		    	//
		    	if (depapp==1 || depapp==2)
					lastDeptime[j]=lastdeptime[0];
				else
					lastDeptime[j]=lastdeptime[1];				
				if (depphase != (2-lastapproach%2)) // if change approach
					lastDeptime[j]=Math.max(lastdeptime[0], lastdeptime[1]);
				
				// platoon setting						
					if(depphase == (2-lastapproach%2)) // phase 1 (app 1,3) or 2 (app 2,4)				
					{ 									
						if (depphase ==1) // lane 1 or 3
						{
							if (depapp==1) {									
								platoon.add(lastplatoon[0]+1);	
							}								    
							else {	
								platoon.add(lastplatoon[1]+1);																
							}																								
						}
						if (depphase ==2)
						{
							if (depapp==2) {									
								platoon.add(lastplatoon[0]+1);				
							}								    
							else {			
								platoon.add(lastplatoon[1]+1);										
							}	
						}		
					}					
					else { 
						platoon.add(1); 
						}

		    	
				Trajectory TrajectoryAnt = new Trajectory(time,  lastDeptime[j],  sig.get(j),  curCars.get(i-1),  depphase,   platoon.get(j),  leadspeed[depapp-1]);
				deptime.add(TrajectoryAnt.deptime);
				speed.add(TrajectoryAnt.speed);
				leadspeed[depapp-1]=TrajectoryAnt.leadspeed;
				T_NS[j]=TrajectoryAnt.NS;
				// for pedestrian delay calculation
				if (T_NS[j]!=0)
					T_tw_last[j] = TrajectoryAnt.tw_last;
	
		    }


			//////////////////////////////Basic framework of TSP///////////////////
			float[] p = new float[carNum];
			float sum = 0.0f;
			double lastDeptime_global= 0.0;           			
			
			lastDeptime_global=Math.max(lastdeptime[0], lastdeptime[1]);
			for (int j=0; j<allowedCars.size(); j++) {
				int i = allowedCars.get(j);
				//note: if this change, need to change delay0
				// definition 1: discharge time
				distance[currentCar][i] = (float)(Math.max(0.0001, deptime.get(j)-lastDeptime_global));// defined by deptime - use local lastdeptime = total discharge time for all cars			
				// definition 2: delay -->  bad results
				//distance[currentCar][i] = (float)(Math.max(0.0001, deptime.get(j)-curCars.get(i-1).virtualArrivalTime));
								
				sum += Math.pow(pheromone[currentCar][i], 1)*Math.pow(1.0f/distance[currentCar][i], beta);
			}
			// probability 			
			for (int i = 0; i < carNum; i++)
			{
				boolean flag = false;
				for (Integer j:allowedCars) {
					if (i == j.intValue()) {
						p[i] = (float) (Math.pow(pheromone[currentCar][i], 1)*Math.pow(1.0f/distance[currentCar][i], beta))/sum;
						flag = true;
						break;
					}
				}
				if (flag == false) {
					p[i] = 0.f;
				}
			}

			//randomly choose (exploration) next Car
			float selectP1 = rand.nextFloat();		
			int selectCar = 0;
			float sum1 = 0.f;

			for (int i = 0; i < carNum; i++)
			{
				sum1 += p[i];
				if (sum1 > selectP1) {
					selectCar = i;
					if (selectCar==0) {
						selectP1 = rand.nextFloat();
						continue;
					}
					break;
				}
			}			
			// exploitation for next car
			float selectP2 = rand.nextFloat();			
			while ((sum1<=selectP1) && (selectP2>q0))
			{
				selectP2 = q0-0.01f;
				System.out.println(selectP2);
			}
			
			if (selectP2 <= q0)
			{
				int bestCar;
				float maxValue=-10000.f;
		        float cal=0.f;
				for (Integer i:allowedCars) {
						cal = (float) (Math.pow(pheromone[currentCar][i.intValue()], 1)*Math.pow(1.0f/distance[currentCar][i.intValue()], beta));
						if (cal> maxValue) {
							bestCar=i.intValue();
							selectCar=bestCar;
							maxValue=cal;
						}
				}
			}
			
			
            // store results for next visited car to use
			for (int idx = 0; idx<allowedCars.size(); idx++) {
				if (allowedCars.get(idx)==selectCar) {
					int depapp = curCars.get(selectCar-1).approach;
								
					if (lastapproach!=0  && 2-depapp%2 != (2-lastapproach%2))
					{
						lastdeptime[0] = Math.max(lastdeptime[0], lastdeptime[1]);
						lastdeptime[1] = Math.max(lastdeptime[0], lastdeptime[1]);
						lastplatoon[0] = 0;
						lastplatoon[1] = 0;
						
					}
					if (depapp==1 || depapp==2)//set
					{
						lastdeptime[0] = deptime.get(idx);
						lastplatoon[0] = platoon.get(idx);
					}						
					else
					{
						lastdeptime[1] = deptime.get(idx);
						lastplatoon[1] = platoon.get(idx);
					}
					
					
					signal = sig.get(idx);
					tabuSpeed.add(speed.get(idx));
					depTime.add(deptime.get(idx));
					
					/////for pedestrian delay calculation
					if (T_NS[idx]!=0)
					{
						NS += T_NS[idx];
						tw_last=T_tw_last[idx];
						if (T_NS[idx]==2)
						{							
							sigstartTime.add(tw_last-(G_pedmin+Tpc));
							sigstartTime.add(tw_last);
						}
						else
							sigstartTime.add(tw_last);
					}
					/////
					
					break;
				}
			}
			
			if(selectCar==0)
				System.out.println(sum);
				
			lastapproach = curCars.get(selectCar-1).approach;// sometimes sum is indefinitely thus make selectCar=0, and make it wrong
			double ls=leadspeed[lastapproach-1];
			leadspeed=leadspeed_bak.clone();// we need to keep other speeds except the selected speed.
			leadspeed[lastapproach-1]=ls;
						
			// dynamically change allowedCars	        
			for (Integer i:allowedCars) {
				if (i.intValue() == selectCar) {
					allowedCars.remove(i);
					break;
				}
			}				
			
			if (selectCar < numcars1)// lane 1+3: selectCar is not the last car on lane 1 or 3
			{
				allowedCars.add(selectCar+1);
			}

			if (selectCar>=(numcars1+1) && selectCar < (numcars1+numcars2)) { // lane 2+4: selectCar is not the last car on lane 2 or 4
				allowedCars.add(selectCar+1);
			}
				
			//
			tabu.add(Integer.valueOf(selectCar));			
			currentCar = selectCar;			
			//////////////////////////////////////////////Basic framework////////////////////////////////////////////////
		}

		/**
		 * cost function
		 * @return 
		 */
		private float calculateTourLength(){
			float len = 0.f;
			for (int i = 0; i < carNum-1; i++) {				
				len += distance[this.tabu.get(i).intValue()][this.tabu.get(i+1).intValue()];  // total discharge time or travel time? need to choose which is better, here is the first
			   
			}
			return len;
		}

		public Vector<Integer> getAllowedCars() {
			return allowedCars;
		}

		public void setAllowedCars(Vector<Integer> allowedCars) {
			this.allowedCars = allowedCars;
		}

		public float getTourLength() {
			tourLength = calculateTourLength();
			return tourLength;
		}
		public void setTourLength(float tourLength) {
			this.tourLength = tourLength;
		}
		
		
		public int getCarNum() {
			return carNum;
		}
		public void setCarNum(int carNum) {
			this.carNum = carNum;
		}
		public Vector<Integer> getTabu() {
			return tabu;
		}
		public void setTabu(Vector<Integer> tabu) {
			this.tabu = tabu;
		}
		public Vector<Double> getTabuSpeed() {
			return tabuSpeed;
		}
		public void setTabuSpeed(Vector<Double> tabuSpeed) {
			this.tabuSpeed = tabuSpeed;
		}		
		public Vector<Double> getDepTime() {
			return depTime;
		}
		public void setDepTime(Vector<Double> depTime) {
			this.depTime = depTime;
		}		
		public int getFirstCar() {
			return firstCar;
		}
		public void setFirstCar(int firstCar) {
			this.firstCar = firstCar;
		}
		//////////////////
		public int getNS() {
			return NS;
		}		
		public double getTw_last() {
			return tw_last;
		}		
		public ArrayList<Double> getSigstartTime() {
			return sigstartTime;
		}
	}