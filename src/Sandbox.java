import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.math.*;


public class Sandbox {
	KalmanFilter kf;
	GittinsIndexGenerator gittins_index_generator;
	public Sandbox(){
		Out out=new Out("estimates.txt");
		Double refMean=0.05;
		Double refStdDev=0.1;
		Double Tc=0.7;
		KalmanFilter testpkt_filter=new KalmanFilter(1.0);
		Integer pktlen=100;
		Double k;
		Integer n=pktlen;
		gittins_index_generator=new GittinsIndexGenerator();
		int measureLength=1000;
		Random rand=new Random();
		List<Double> measurement_store = new ArrayList<Double>();
		for (int i = 0; i < measureLength; i++) {
			Double _ber=refStdDev*rand.nextGaussian()+refMean;
			if(_ber<=0.0){
				_ber=0.001;
			}else if(_ber>=1.0){
				_ber=0.999;
			}
			measurement_store.add(_ber);
		}
		
		for(Double ber: measurement_store){
			k=pktlen*ber;
			testpkt_filter.performTimeUpdate();
			testpkt_filter.setR(getTestPktVariance(k, n));
			testpkt_filter.measurementUpdate(f((double) (k/n)));
			System.out.println(ber+" "+k+" "+n+" "+fPrime((double) (k/n))+" "+getTestPktVariance(k, n)+" "+testpkt_filter.getEstimate());
		}
		
	}
	
	private Double getDataPktVariance(Boolean result, Double Tc){
		if(result){
			return ((Tc*Tc)/12.0)*fPrime((Tc/2.0));
		}else{
			return (Math.pow(((1.0/2.0)-Tc),2.0) / 12.0) +fPrime((Tc/2.0)+(1.0/4.0));
		}
	}
	
	private Double getTestPktVariance(double k, double n){
		return (k/(n*n))*(1-(k/n))*fPrime((double) (k/n));
	}
	
	private Double f(Double _theta){
		Double theta=_theta;

		if(theta<=0.5){
			return Math.log(2*theta);
		}else{
			return -1.0*Math.log(2*(1-theta));
		}
	}
	
	private Double fPrime(Double _theta){
		Double theta=_theta;

		if(theta<=0.5){
			return 1.0/(theta);
		}else{
			return 1.0/(1-theta);
		}
	}
	
}
