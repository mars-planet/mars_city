import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.io.IOException;
import py4j.GatewayServer;

import psengine.PSEngine;
import psengine.PSUtil;
import psengine.util.LibraryLoader;
import java.util.Arrays;

public class pyEUROPA {

    // Default server port
    private static int PORT = 25333;
    protected static PSEngine psEngine_;

    // Main thread of py4j
    public static void main(String[] args) throws IOException{

        // Kill existing bound ports
        Process p = Runtime.getRuntime().exec("fuser -k "+PORT+"/tcp");

        GatewayServer gatewayServer = new GatewayServer(new pyEUROPA(), PORT);
        gatewayServer.start();
    }

    public static PSEngine makePSEngine(String debug){
        PSEngine psEngine;
        LibraryLoader.loadLibrary("System_"+debug);
        psEngine = PSEngine.makeInstance();
        return psEngine;
    } 

}
