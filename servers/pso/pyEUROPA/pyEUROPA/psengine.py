__author__ = "Mathew Kallada"

import os
import subprocess
import glob
import time
import traceback
import signal
from py4j.java_gateway import JavaGateway

PID = None
GATEWAY = None
ps_engine_instances = []

def makePSEngine():
    global PID, GATEWAY, ps_engine_instances

    # Check location of EUROPA
    if os.environ.has_key("EUROPA_HOME"):
        europa_dir = os.environ["EUROPA_HOME"]
    elif os.path.isdir(os.path.expanduser("~/europa/")):
        europa_dir = os.path.expanduser("~/europa/")
    else:
        raise ValueError("Unable to find EUROPA directory! "+\
            "Please either put EUROPA in ~/europa/ or set the "+\
            "EUROPA_HOME enviroment variable.")

    java_folder = os.path.join(os.path.dirname(__file__), 'java')
    java_lib = os.path.join(java_folder, 'lib')
    java_src = os.path.join(java_folder, 'src')
    
    if os.name == 'win32':
        java_bin = "C:/pyEUROPA/bin"
    else:
        java_bin = '/tmp/pyEUROPA/bin'
    
    if not os.path.exists(java_bin):
        os.makedirs(java_bin)
    
    # Set CLASSPATH for java
    classpath = os.environ["CLASSPATH"] if os.environ.has_key("CLASSPATH") else ''
    classpath += (".:" + java_bin + ':')
    for lib_jar in glob.glob(os.path.join(java_lib, '*.jar')):
        classpath = ''.join([os.path.join(java_lib, lib_jar), ':', classpath])

    classpath = ''.join([europa_dir+"/lib/PSEngine.jar", ':', classpath])

    # JAVA_HOME
    javahome = os.environ.get("JAVA_HOME")
    
    # Compile java module
    compile_cmd = os.path.join(javahome, 'bin/javac') if javahome else 'javac'
    compile_cmd = ' '.join([compile_cmd, '-d', java_bin, '-cp', 
        classpath, os.path.join(java_src, 'pyEUROPA.java')])
    pc = subprocess.Popen(compile_cmd, shell = True); pc.wait()
    
    # Run py4j gateway
    run_cmd = os.path.join(javahome, 'bin/java') if javahome else 'java'
    run_cmd = ' '.join([run_cmd, "-D"+europa_dir+"/lib/", '-cp',
     classpath, 'pyEUROPA'])
    pr = subprocess.Popen(run_cmd, shell = True, preexec_fn=os.setsid)
    PID = pr.pid

    # Wait for py4j gateway to start
    time.sleep(1)

    try:
        GATEWAY = JavaGateway()
        psengine = GATEWAY.entry_point.makePSEngine("o")
        ps_engine_instances.append(psengine)
    except:
        stopPSEngine()

        print traceback.format_exc()
        return

    return psengine

def stopPSEngine():

    # Shutdown all PSEngine instances
    for psengine in ps_engine_instances:
        psengine.shutdown()

    # Send the signal to all the process groups
    if PID:
        os.killpg(PID, signal.SIGTERM)
    else:
        raise ValueError("You must first create a gateway.")
