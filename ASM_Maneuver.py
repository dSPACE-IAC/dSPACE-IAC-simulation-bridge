"""
Copyright 2024, dSPACE GmbH. All rights reserved.
"""


import sys, os
import time
import xml.etree.ElementTree as ET
import socket

# Import XIL API .NET classes from the .NET assemblies
from ASAM.XIL.Implementation.TestbenchFactory.Testbench import TestbenchFactory
from ASAM.XIL.Interfaces.Testbench.Common.Error import TestbenchPortException


# The following lines must be adapted to the dSPACE platform used
#------------------------------------------------------------------------------------------------
# Use an MAPort configuration file that is suitable for your platform and simulation application
# See the folder Common\PortConfigurations for some predefined configuration files
MAPortConfigFile = r"MAPortConfig.xml"

MAPortConfigFile = os.path.join(os.path.dirname(__file__), MAPortConfigFile)

def format_list(items):
    if len(items) > 1:
        return ', '.join(items[:-1]) + ' and ' + items[-1] + '.'
    elif items:
        return items[0]
    else:
        return ''
    
def parameterizeOrchestrator(maPort, number_of_clients):

    paramTimeoutClient_signal_path = 'Orchestrator/Model Root/Preprocessor/Subsystem/S-Function/P2'
    maPort.Write(paramTimeoutClient_signal_path, MyValueFactory.CreateFloatValue(6000000))

    paramRequireClients_signal_path = 'Orchestrator/Model Root/Preprocessor/Subsystem/S-Function/P3'
    maPort.Write(paramRequireClients_signal_path, MyValueFactory.CreateFloatValue(number_of_clients))

    paramNewSubscriberPeriodMS_signal_path = 'Orchestrator/Model Root/Preprocessor/Subsystem/S-Function/P4'
    maPort.Write(paramNewSubscriberPeriodMS_signal_path, MyValueFactory.CreateFloatValue(0))

    paramStaticCosim_signal_path = 'Orchestrator/Model Root/Preprocessor/Subsystem/S-Function/P5'
    maPort.Write(paramStaticCosim_signal_path, MyValueFactory.CreateFloatValue(1.0))

    paramBlockingSimulation_signal_path = 'Orchestrator/Model Root/Preprocessor/Subsystem/S-Function/P6'
    maPort.Write(paramBlockingSimulation_signal_path, MyValueFactory.CreateFloatValue(1.0))

    paramForceUnsubscribe_signal_path = 'Orchestrator/Model Root/Preprocessor/Subsystem/S-Function/P7'
    maPort.Write(paramForceUnsubscribe_signal_path, MyValueFactory.CreateFloatValue(0))

    paramAdtfBufferSizePub_signal_path = 'Orchestrator/Model Root/data_message_receive/ADTF_Message_Decode/adtf_msg_decode/P7'
    maPort.Write(paramAdtfBufferSizePub_signal_path, MyValueFactory.CreateFloatValue(5))

    paramAdtfBufferSizeSub_signal_path = 'Orchestrator/Model Root/subscribe_message_receive/ADTF_Message_Decode1/adtf_msg_decode/P7'
    maPort.Write(paramAdtfBufferSizeSub_signal_path, MyValueFactory.CreateFloatValue(5))

    
if __name__ == "__main__":

    MAPort = None
    valid_commands = ["start", "stop", "reset", "status", "healthcheck", "noclif", "raceparam", "loadonly", "manualflag", "trackflag", "vehicleflag", "deactivatepush2pass", "synchrace", "orchparam"]

    env_command = os.environ.get("DS_START_COMMAND", None)
    if len(sys.argv) < 2:
        if env_command:
            command_list = [command.lower() for command in env_command.split('-')]
        else:          
          print("Using default command to start the ASM maneuver directly.\n")
          command_list = ['start']
    else:
        command_list = [command.lower() for command in sys.argv[1::]]    
    

    for command in command_list:
      if command.split('_')[0] not in valid_commands:
          print("Invalid command. Use {0}".format(format_list(valid_commands)))
          sys.exit(1)


    is_orchestrator = os.environ.get("DS_IS_ORCHESTRATOR", None)

    # Pare MAPort config and add VEOS application from veosloadosa script
    # os.environ["DS_VEOS_LOAD_OSA"] ="/home/dspace/VEOS_LNX/ASM_Traffic.osa"
    osa_path = os.environ.get("DS_VEOS_LOAD_OSA", None)
    if osa_path:
        tree = ET.parse(MAPortConfigFile) 
        root = tree.getroot()
        sdf_node = root.find(".//SystemDescriptionFile")
        new_sdf = osa_path.replace('.osa', '.sdf')
        if not sdf_node.text == new_sdf:
            sdf_node.text = new_sdf
            MAPortConfigFile = os.path.join(os.path.dirname(__file__), "MAPortConfig_new.xml")
            tree.write(MAPortConfigFile)

    try:
        #--------------------------------------------------------------------------
        # Create a TestbenchFactory object; the TestbenchFactory is needed to
        # create the vendor-specific Testbench
        #--------------------------------------------------------------------------
        MyTestbenchFactory = TestbenchFactory()

        #--------------------------------------------------------------------------
        # Create a dSPACE Testbench object; the Testbench object is the central object to access
        # factory objects for the creation of all kinds of Testbench-specific objects
        #--------------------------------------------------------------------------
        MyTestbench = MyTestbenchFactory.CreateVendorSpecificTestbench("dSPACE GmbH", "XIL API", "2023-A")

        #--------------------------------------------------------------------------
        # We need an MAPortFactory to create an MAPort and also a ValueFactory to create ValueContainer
        # objects
        #--------------------------------------------------------------------------
        MyMAPortFactory = MyTestbench.MAPortFactory
        MyValueFactory = MyTestbench.ValueFactory
        MyVariableRefFactory = MyTestbench.VariableRefFactory

        #--------------------------------------------------------------------------
        # Create and configure an MAPort object and start the simulation
        #--------------------------------------------------------------------------
        print("Creating MAPort instance...")
        # Create an MAPort object using the MAPortFactory
        MAPort = MyMAPortFactory.CreateMAPort("MAPort")
        print("...done.\n")
        # Load the MAPort configuration
        print("Configuring MAPort...")
        MAPortConfig = MAPort.LoadConfiguration(MAPortConfigFile)
        # Apply the MAPort configuration
        MAPort.Configure(MAPortConfig, False)
        print("...done.\n")

        #----------------------------------------------------------------------
        # Select scenario
        #----------------------------------------------------------------------
        #  Path to model variables
        if not is_orchestrator:
            roadSwitch = "ASM_Traffic/Model Root/Environment/Road/UserInterface/PAR_Plant/ROAD_SELECT_SWITCH/SW_Road_Select"
            scenarioSwitch = "ASM_Traffic/Model Root/Environment/Maneuver/UserInterface/PAR_Plant/MANEUVER_SELECT_SWITCH/Sw_Maneuver_Select"
                
            # Get scenario ID
            scenario_id = os.environ.get("DS_ASM_SCENARIO_ID", None)
            if scenario_id:
                print("Selecting scenario ID ", scenario_id)
                scenario_id_int = int(scenario_id)
                if scenario_id_int in range(1, 7):
                        
                    MAPort.Write(roadSwitch, MyValueFactory.CreateFloatValue(scenario_id_int))
                    MAPort.Write(scenarioSwitch, MyValueFactory.CreateFloatValue(scenario_id_int))
                    print("Scenario set ")
                else:
                    print("Scenario not in range 1 to 6")
            else:
                print("No scenario ID provided via envrionment variable DS_ASM_SCENARIO_ID. Using default scenario.")

            #----------------------------------------------------------------------
            # Set custom data timing
            #----------------------------------------------------------------------
            #  Path to model variables
            numSamplesPath = "ASM_Traffic/Tunable Parameters/IO.send_period_samples.v"

            # Get num samples
            num_samples = os.environ.get("DS_CUSTOM_DATA_NUM_SAMPLES", None)
            if num_samples:
                num_samples_int = int(num_samples)
                if num_samples_int > 0:
                    print("Setting custom data send period to sample period: ", num_samples)
                    MAPort.Write(numSamplesPath, MyValueFactory.CreateFloatValue(num_samples_int))
            else:
                print("No number of samples provided via envrionment variable DS_CUSTOM_DATA_NUM_SAMPLES. Using default number of sample for custom data interface.")   



            # V-ESI IP settings in case of local race
            vesi_ip_path = 'ASM_Traffic/Model Root/VesiInterface/Send_CustomData/dsa_VEOS_eth_tcp_client_BL1/dsa_veos_eth_tcp_client_BL/SParameter1'
            vesi_ip = os.environ.get("DS_VESI_IP", None)
            if vesi_ip:
                vesi_ip_int = [int(element) for element in vesi_ip.split('.')]
                MAPort.Write(vesi_ip_path, MyValueFactory.CreateUintVectorValue(vesi_ip_int))
                print("V-ESI IP set to:", vesi_ip)    


            # Set scenrio parameters
            s_init_value = os.environ.get("DS_INIT_S", None)
            s_init_overwrite_path = "ASM_Traffic/Model Root/Environment/Road/PlantModel/set_s_init/Overwrite/Value"
            s_init_path = "ASM_Traffic/Model Root/Environment/Road/PlantModel/set_s_init/s_init_const/Value"
            if s_init_value:
                s_init_int = int(s_init_value)
                MAPort.Write(s_init_overwrite_path, MyValueFactory.CreateFloatValue(1.0))
                MAPort.Write(s_init_path, MyValueFactory.CreateFloatValue(s_init_int))

            d_init_value = os.environ.get("DS_INIT_D", None)
            d_init_overwrite_path = "ASM_Traffic/Model Root/Environment/Road/PlantModel/set_d_init/Overwrite/Value"
            d_init_path = "ASM_Traffic/Model Root/Environment/Road/PlantModel/set_d_init/d_init_const/Value"
            if d_init_value:
                d_init_int = int(d_init_value)
                MAPort.Write(d_init_overwrite_path, MyValueFactory.CreateFloatValue(1.0))
                MAPort.Write(d_init_path, MyValueFactory.CreateFloatValue(d_init_int))

            route_id_value = os.environ.get("DS_ROUTE_ID", None)
            route_id_overwrite_path = "ASM_Traffic/Model Root/Environment/Road/PlantModel/set_route_init/Overwrite/Value"
            route_id_path = "ASM_Traffic/Model Root/Environment/Road/PlantModel/set_route_init/route_init_const/Value"
            if route_id_value:
                route_id_int = int(route_id_value)
                MAPort.Write(route_id_overwrite_path, MyValueFactory.CreateFloatValue(1.0))
                MAPort.Write(route_id_path, MyValueFactory.CreateFloatValue(route_id_int))

            #----------------------------------------------------------------------
            # Start, stop and reset ASM maneuver
            #----------------------------------------------------------------------
            #  Path to model variables
            manevuerStart = "ASM_Traffic/Model Root/Environment/Maneuver/UserInterface/PAR_Plant/ManeuverControl/MANEUVER_START/MDLDCtrl_ManeuverStart"
            manevuerStop = "ASM_Traffic/Model Root/Environment/Maneuver/UserInterface/PAR_Plant/ManeuverControl/MANEUVER_STOP/MDLDCtrl_ManeuverStop"
            manevuerReset = "ASM_Traffic/Model Root/Environment/Maneuver/UserInterface/PAR_Plant/ManeuverControl/RESET/MDLDCtrl_Reset"
            maneuverStateDir = "ASM_Traffic/Model Root/Environment/Maneuver/UserInterface/DISP_Plant/ManeuverState[]/Out1"
            maneuverStateLastValue = -1

        for command in command_list:
            if command == "start":
                MAPort.Write(manevuerStart, MyValueFactory.CreateFloatValue(1.0))
                time.sleep(0.5)
                MAPort.Write(manevuerStart, MyValueFactory.CreateFloatValue(0.0))
                print("Start maneuver set. \n")

            elif command == "stop":
                MAPort.Write(manevuerStop, MyValueFactory.CreateFloatValue(1.0))
                time.sleep(0.5)
                MAPort.Write(manevuerStop, MyValueFactory.CreateFloatValue(0.0))
                print("Stop maneuver set.\n")
                
            elif command == "reset":
                MAPort.Write(manevuerReset, MyValueFactory.CreateFloatValue(1.0))
                time.sleep(0.5)
                MAPort.Write(manevuerReset, MyValueFactory.CreateFloatValue(0.0))
                print("Stop maneuver set.\n")
            
            elif command == "status":
                maneuverState = MAPort.Read(maneuverStateDir)
                print("Maneuver state:", maneuverState.Value)
                
            elif command == "healthcheck" and not is_orchestrator:
                print("Starting healthcheck.\n")
                while True:
                    maneuverState = MAPort.Read(maneuverStateDir)
                    if maneuverState.Value != maneuverStateLastValue:
                        with open("/home/dspace/scripts/maneuverstatus.sh", "w") as fileStatus:
                            if maneuverState.Value == 3:
                                print("State changed to healthy. Maneuver State:",  maneuverState.Value)
                                fileStatus.write("exit 0")
                            else:
                                print("State changed to unhealthy.\n")
                                fileStatus.write("exit 1")
                        maneuverStateLastValue = maneuverState.Value
                    time.sleep(0.5)
                print("Healthcheck exited unexpectedly.\n")
                with open("/home/dspace/scripts/maneuverstatus.sh", "w") as fileStatus:
                    fileStatus.write("exit 1")

            elif command == "healthcheck" and is_orchestrator:
                print("Starting healthcheck.\n")
                while True:
                    with open("/home/dspace/scripts/maneuverstatus.sh", "w") as fileStatus:
                        #print("State changed to healthy.\n")
                        fileStatus.write("exit 0")
                    time.sleep(0.5)
                print("Healthcheck exited unexpectedly.\n")
                with open("/home/dspace/scripts/maneuverstatus.sh", "w") as fileStatus:
                    fileStatus.write("exit 1")

            elif command == "orchparam" and is_orchestrator:
                number_of_clients = os.environ.get("DS_NUMBER_OF_CLIENTS", None)
                print("Parametrize Orchestrator. Number of Clients:", number_of_clients)
                parameterizeOrchestrator(MAPort, float(number_of_clients))
                print("Orchestrator parametrization done.")

            elif command == "raceparam":
                orchestrator_ip = os.environ.get("DS_ORCHESTRATOR_IP", None)
                if orchestrator_ip:
                    orchestrator_ip_int = [int(element) for element in orchestrator_ip.split('.')]
                    asm_ip = [int(element) for element in socket.gethostbyname(socket.gethostname()).split('.')]

                    MAPort.Write('ASM_Traffic/Model Root/RaceInterface/Triggered_ADTF_Send/If Action\nSubsystem/If Action\nSubsystem1/Param_Dst_IP_Address/Value', MyValueFactory.CreateUintVectorValue(asm_ip))

                    # Orchestrator IP
                    MAPort.Write('ASM_Traffic/Model Root/RaceInterface/Triggered_ADTF_Send/If Action\nSubsystem/If Action\nSubsystem1/ADTF_Send/For Iterator\nSubsystem/VEOS_UDP_TX_Block/dsa_veos_eth_udp_transmit_BL/SParameter1', MyValueFactory.CreateUintVectorValue(orchestrator_ip_int))
                    MAPort.Write('ASM_Traffic/Model Root/RaceInterface/Triggered_ADTF_Send/If Action\nSubsystem/If Action\nSubsystem/ADTF_Send/For Iterator\nSubsystem/VEOS_UDP_TX_Block/dsa_veos_eth_udp_transmit_BL/SParameter1', MyValueFactory.CreateUintVectorValue(orchestrator_ip_int))
                    MAPort.Write('ASM_Traffic/Model Root/RaceInterface/While Iterator\nSubsystem/Triggered_ADTF_Send_RT/If Action\nSubsystem/ADTF_Send/For Iterator\nSubsystem/VEOS_UDP_TX_Block/dsa_veos_eth_udp_transmit_BL/SParameter1', MyValueFactory.CreateUintVectorValue(orchestrator_ip_int))
                    
                    port_send_sub = 5004
                    MAPort.Write('ASM_Traffic/Model Root/RaceInterface/Triggered_ADTF_Send/If Action\nSubsystem/If Action\nSubsystem1/ADTF_Send/For Iterator\nSubsystem/VEOS_UDP_TX_Block/dsa_veos_eth_udp_transmit_BL/SParameter2', MyValueFactory.CreateUintValue(port_send_sub))

                    port_send_data_rt = 5003
                    MAPort.Write('ASM_Traffic/Model Root/RaceInterface/While Iterator\nSubsystem/Triggered_ADTF_Send_RT/If Action\nSubsystem/ADTF_Send/For Iterator\nSubsystem/VEOS_UDP_TX_Block/dsa_veos_eth_udp_transmit_BL/SParameter2', MyValueFactory.CreateUintValue(port_send_data_rt))

                    port_send_data = 5002
                    MAPort.Write('ASM_Traffic/Model Root/RaceInterface/Triggered_ADTF_Send/If Action\nSubsystem/If Action\nSubsystem/ADTF_Send/For Iterator\nSubsystem/VEOS_UDP_TX_Block/dsa_veos_eth_udp_transmit_BL/SParameter2', MyValueFactory.CreateUintValue(port_send_data))

                    # Race interface
                    MAPort.Write('ASM_Traffic/Model Root/RaceInterface/UserInterface/PAR_Plant/Sw_ActivateRace[0|1]/IO.VISSIM.EnableInterface', MyValueFactory.CreateFloatValue(1))
                    MAPort.Write('ASM_Traffic/Model Root/RaceInterface/UserInterface/PAR_Plant/Sw_BlockingSimulation[0|1]/Value', MyValueFactory.CreateBooleanValue(1))
                    MAPort.Write('ASM_Traffic/Model Root/RaceControl/Sw_RaceControl[0Intern|1Extern|2Orchestrator]/Value', MyValueFactory.CreateFloatValue(2))

                    print("Race parametrization done.")
            
            elif command == "noclif":
                MAPort.Write('ASM_Traffic/Model Root/VesiInterface/Sw_Activate_CLIF[0|1]/Value', MyValueFactory.CreateFloatValue(0.0))
                print("CLIF deactivated.")
            
            elif command == "loadonly":
                print("Application loaded.")

            elif command == "manualflag":
                MAPort.Write('RaceControl/Model Root/Parameters/manual_mode', MyValueFactory.CreateFloatValue(1.0))
                print("Race control set to manual flag mode.")

            elif command == "deactivatepush2pass":
                MAPort.Write('RaceControl/Model Root/Parameters/push2pass_enable', MyValueFactory.CreateFloatValue(0.0))
                print("Push to pass deactivated.")

            elif command == "synchrace":
                MAPort.Write('ASM_Traffic/Model Root/RaceInterface/UserInterface/PAR_Plant/Sw_BlockingSimulation[0|1]/Value', MyValueFactory.CreateBooleanValue(True))
                print("Blocking race synchronisation activated.")

            elif "trackflag" in command:
                command_parts = command.split("_")
                if len(command_parts) > 1:
                    try:
                        trackflag_value = int(command_parts[1])
                        
                        MAPort.Write('RaceControl/Model Root/Parameters/track_flag_manual[1,1]', MyValueFactory.CreateFloatValue(trackflag_value))
                        print(f"Trackflag set to: {trackflag_value}")
                    except ValueError:
                        print("The part after 'trackflag_' is not a valid integer.")
                else:
                    print("The trackflag command does not contain an underscore followed by a value.")

            elif "vehicleflag" in command:
                command_parts = command.split("_")
                if len(command_parts) > 1:
                    try:
                        trackflag_value = int(command_parts[1])
                        
                        MAPort.Write('RaceControl/Model Root/Parameters/veh_flag_manual[1,1]', MyValueFactory.CreateFloatValue(trackflag_value))
                        print(f"Vehicle flag set to: {trackflag_value}")
                    except ValueError:
                        print("The part after 'vehicleflag_' is not a valid integer.")
                else:
                    print("The vehicleflag command does not contain an underscore followed by a value.")
                

    except TestbenchPortException as ex:
        #-----------------------------------------------------------------------
        # Display the vendor code description to get the cause of an error
        #-----------------------------------------------------------------------
        print("A TestbenchPortException occurred:")
        print("CodeDescription: %s" % ex.CodeDescription)
        print("VendorCodeDescription: %s" % ex.VendorCodeDescription)
        if command == "healthcheck":
            print("Healthcheck exited unexpected.\n")
            with open("/home/dspace/scripts/maneuverstatus.sh", "w") as fileStatus:
                fileStatus.write("exit 1")
        raise
    finally:
        #-----------------------------------------------------------------------
        # Attention: make sure to dispose the MAPort object in any case to free
        # system resources like allocated memory and also resources and services on the platform
        #-----------------------------------------------------------------------
        if MAPort != None:
            MAPort.Dispose()
            MAPort = None





 


