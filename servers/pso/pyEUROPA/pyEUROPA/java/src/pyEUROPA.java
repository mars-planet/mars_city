import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import py4j.GatewayServer;

import psengine.PSEngine;
import psengine.util.LibraryLoader;

public class pyEUROPA {

    // Default server port
    private static int PORT = 25333;
    protected static PSEngine psEngine_;

    // Main thread of py4j
    public static void main(String[] args){
        GatewayServer gatewayServer = new GatewayServer(new pyEUROPA(), PORT);
        gatewayServer.start();
    }

    public static PSEngine makePSEngine(String debug)
    {
        PSEngine psEngine;
        LibraryLoader.loadLibrary("System_"+debug);
        psEngine = PSEngine.makeInstance();

        return psEngine;
    } 

}
