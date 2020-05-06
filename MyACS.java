import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class MyACS {  
	
		private Ant[] ants; //
		private int antNum; //
		private int carNum; //carNum includes the virtual car: last departure car
		private int MAX_GEN; //
		private float[][] pheromone; //
		private float[][] distance; //
		private float bestLength; //
		
		int[] bestTour; //
		int[] bestTour_comp;
		double[] bestSpeed;
		double[] bestDepTime;
		int[] bestTour_joint;  
		double[] bestSpeed_joint;
		double[] bestDepTime_joint;
		double preTb;
		
		private float delta0;
	
		//
		private float alpha; 
		private float beta;
		private float rho;
		private float q0;
		//
		private double curtime;
		private int numCars1;
		private int numCars2;
	
		ArrayList<Car> Cars = new ArrayList<Car>();
		
		signalLight sig; 
		private double[] Leadspeed;
		private double[] Lastdeptime;
		private int Lastapproach;
		private int[] Lastplatoon;
		
		public MyACS(){

		}		
		/** constructor of ACS
		 * @param n Car number
		 * @param m ants number
		 * @param g iteration
		 * @param a alpha
		 * @param b beta
		 * @param r rho
		 * @param q q0 the probability for exploitation
		 **/
		public MyACS(signalLight sig0, int n,  int m, int g, float a, float b, float r, float q, double time, int nc1, int nc2, ArrayList<Car> curCars, double[] ls, double[] ldt, int lda, int[] ldp) {
			sig = sig0;
			carNum = n;
			antNum = m;
			ants = new Ant[antNum];
			MAX_GEN = g;
			alpha = a;
			beta = b;
			rho = r;
			q0 = q;
			curtime = time;
			numCars1 = nc1;
			numCars2 = nc2;
			Cars = curCars;
			Leadspeed = ls.clone();
			Lastdeptime = ldt.clone();
			Lastapproach = lda;
			Lastplatoon = ldp.clone();
			
		}

		//@SuppressWarnings("resource")
		/**
		 * 
		 * @param filename 
		 * @throws IOException
		 */
		void init(float pre_delay) {   // for each new case, do this initialization.
			// Initialize cost-to-go value
			distance = new float[carNum][carNum]; //  save the cost-to-go value, defined by (current car departure time - last car departure time)  

			// Initialize pheromone
			pheromone=new float[carNum][carNum];  
			delta0 = 1.0f/((carNum)*pre_delay); // carNum-1:  car numbers at current case, one means last departure car				
			for(int i=0;i<carNum;i++)  
			{  
				for(int j=0;j<carNum;j++){  
					pheromone[i][j]=delta0;  //initial pheromone	
				}  
			}  
						
			
			bestLength=(float)Integer.MAX_VALUE;  
			bestTour=new int[carNum]; 
			bestTour_comp=new int[carNum];  
			bestSpeed=new double[carNum];
			bestDepTime=new double[carNum];
			bestTour_joint=new int[carNum];  
			bestSpeed_joint=new double[carNum];						
			bestDepTime_joint=new double[carNum];
			for(int i=0;i<antNum;i++){  
				ants[i]=new Ant(carNum);  
				ants[i].init(distance, beta, q0, numCars1, numCars2, Cars, sig, Leadspeed, Lastdeptime, Lastapproach, Lastplatoon);  
			}  
		}
						
	   
		public void solve(signalLight sig1, double Wped, double time, ArrayList<Double> WaitListPed1, ArrayList<Double> WaitListPed2,ArrayList<Double> WaitListPed3, ArrayList<Double> WaitListPed4, double flowPed1, double flowPed2, double flowPed3, double flowPed4, double LastTb, double tpc){
			
			double delay=(double)Integer.MAX_VALUE;
			for (int g = 0; g < MAX_GEN; g++) {  // each iteration finds a vehicle order																			
				
				for (int i = 0; i < antNum; i++) {			
					//////// joint optimization: for each ant, there is a generated ds J.
					int NS = 0; // number of Walk signal switches on all (two confliciting approaches) 
					double tw_last = 0.0; // for NS=0; the previous Walk switch time before tb.
					ArrayList<Double> sigstartTime =new ArrayList<Double>();
					double lastTb = LastTb;
					
					for (int j = 1; j < carNum; j++) {
						ants[i].selectNextCar(pheromone, curtime); //curtime is the simulation time
					}
					ants[i].getTabu().add(ants[i].getFirstCar());
					
					//if (ants[i].getTourLength() < bestLength) //this is only for single vehicle optimization, we don't use this in joint optimization	
					//{
						//bestLength = ants[i].getTourLength();		
						for (int k = 0; k < carNum; k++) {
							bestTour[k] = ants[i].getTabu().get(k); 
                            bestSpeed[k] = ants[i].getTabuSpeed().get(k);
                            bestDepTime[k] = ants[i].getDepTime().get(k);
						}									
						
						NS=ants[i].getNS();
						tw_last=ants[i].getTw_last();
						sigstartTime= ants[i].getSigstartTime();// pedestrian signal start time for each phase (two conflicting approaches)												
					//}
						
					///local pheromone update
					for (int j = 0; j < carNum-1; j++)
						pheromone[ants[i].getTabu().get(j)][ants[i].getTabu().get(j+1)] = (1-rho)*pheromone[ants[i].getTabu().get(j)][ants[i].getTabu().get(j+1)] + rho*delta0; // the last car is the virtualcar
								
					////////////////////////////Joint optimization for each ant//////////////////////////
					//when adding pedestrian model: make the decision based on total weighted vehicle delay and pedestrian delay	
					if (Arrays.toString(bestTour).compareTo(Arrays.toString(bestTour_comp))!=0)
						{
						// vehicle delay calculation
						bestTour_comp = bestTour.clone();
						double Delay = 0.0;
						double vehDelay = 0.0;
						
						//vehicle delay calculation
						for(int ii = 0; ii < carNum-1; ii ++)
						   vehDelay += (bestDepTime[ii+1]-Cars.get(bestTour[ii+1]-1).virtualArrivalTime);
						
						
						//pedestrian delay calculation			
						double pedDelay = 0.0;
						double ta=0.0; // time of current event
						double tb=0.0; // time of last departure vehicle
						double Tb=0.0; // time extension of the calculation horizon
						
						double Wveh=1-Wped;
	
						ArrayList<Double> sigstartTimePed1 =new ArrayList<Double>(); // traffic signal
						ArrayList<Double> sigstartTimePed2 =new ArrayList<Double>();
										
						double[] tax = new double[4];
						double minta= (double)Integer.MAX_VALUE;
					    // pedestrian delay calculation
						if (Wped>=0) {
							ta = time;
							// this is related to the already waiting pedestrians (affect calculating delay).					
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
							////////////		                
							double maxIndex = bestDepTime[0];
							for (int num = 0; num < bestDepTime.length; num++) {
								 if(maxIndex<bestDepTime[num])
									 maxIndex=bestDepTime[num];
							}	
							if (carNum==0)
			                	tb = ta;
			                else
			                	tb = maxIndex;
							
			                	
							if (!sigstartTime.isEmpty())
							{
								if (sig1.phase==2)
								{
									for (int ii=0; ii < sigstartTime.size(); ii++)
									{
										if (ii%2==0)
											sigstartTimePed2.add(sigstartTime.get(ii));
										else
											sigstartTimePed1.add(sigstartTime.get(ii));
									}						
								}
								else
								{
									for (int ii=0; ii < sigstartTime.size(); ii++)
									{
										if (ii%2==0)
											sigstartTimePed1.add(sigstartTime.get(ii));
										else
											sigstartTimePed2.add(sigstartTime.get(ii));
									}	
								}
							}
							
							pedestrianDelay pD = new pedestrianDelay(sig1, flowPed1, flowPed2, flowPed3, flowPed4, tax, ta, tb, lastTb, tpc, tw_last,  NS,  WaitListPed1,  WaitListPed2, WaitListPed3,  WaitListPed4, sigstartTimePed1,  sigstartTimePed2);
							pedDelay = pD.delay;					
							Tb = pD.Tb;						
							//if (pedDelay<vehDelay)// way?
							   Delay = Wveh * vehDelay + Wped * pedDelay;
							//else
							   //Delay = vehDelay;	
						}
						else
							Delay = vehDelay;
						
						//Solved for case that have same values by different sequences, choose sequence beginning (or occur earlier) at lane 1 in priority.
						if (Delay == delay && Arrays.toString(bestTour).compareTo(Arrays.toString(bestTour_joint))<0)// compare with ascii
						{				
							bestLength = ants[i].getTourLength();
							
							bestTour_joint=bestTour.clone();
							bestSpeed_joint=bestSpeed.clone();
							bestDepTime_joint=bestDepTime.clone();
							preTb=Tb;
						}
										
						if(Delay < delay)// use weighted delay function to evaluate solution; differently use distance function to make decision (to select which is the next car visited)
						{
							bestLength = ants[i].getTourLength();// this is the opted length in this evaluation, other than bestlength of shortest distance.
							
							bestTour_joint=bestTour.clone();
							bestSpeed_joint=bestSpeed.clone();
							bestDepTime_joint=bestDepTime.clone();						
							delay = Delay;	
							preTb=Tb;
							//System.out.println(vehDelay);
							//System.out.println(pedDelay);
						}
						
					}
				///end of delay calculation
					

				}// end of this iteration
				
				////global pheromone update
				for(int j=0;j<carNum-1;j++){  	
					//pheromone[bestTour[j]][bestTour[j+1]] = (1-alpha)*pheromone[bestTour[j]][bestTour[j+1]] + alpha*(1.f/bestLength);
					pheromone[bestTour_joint[j]][bestTour_joint[j+1]] = (1-alpha)*pheromone[bestTour_joint[j]][bestTour_joint[j+1]] + alpha*(1.f/bestLength);// joint optimization
				} 
												
	            ////////////////////////////initialization for next iteration
				for(int i=0;i<antNum;i++){  
					ants[i].init(distance, beta, q0, numCars1, numCars2, Cars, sig, Leadspeed, Lastdeptime, Lastapproach, Lastplatoon);  
				}  
			}// end of all iteration
			
		}
		
		public Ant[] getAnts() {
			return ants;
		}
		public void setAnts(Ant[] ants) {
			this.ants = ants;
		}
	}