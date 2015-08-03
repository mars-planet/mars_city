
import org.oewf.aouda.telemetry.*;

public class dataStorageDaemon {
  public static void main(String[] args) {
    MarvinDataProvider mdp=null;
    mdp = new MarvinDataProvider("192.168.1.200", 6668);
    MarvinListener mListener = new MarvinListenerImplementation();

    System.out.println("Storing Data");
    mdp.addListener(mListener); 
  }
}

