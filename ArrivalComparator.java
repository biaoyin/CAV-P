import java.util.Comparator;

public class ArrivalComparator implements Comparator<Object> {

	public int compare(Object object1, Object object2) {  
	        Car c1 = (Car) object1;   
	        Car c2 = (Car) object2;  
	        Double a1 = c1.enterTime;
	        Double a2 = c2.enterTime;
	        
	        return a1.compareTo(a2); 
	    }  
}
