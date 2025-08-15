#pragma once
#ifdef _WIN32
    #if defined(VESILIBRARY_EXPORT) // inside DLL
        #define VESILIBRARYEXPORT   __declspec(dllexport)
    #else
        #define VESILIBRARYEXPORT   __declspec(dllimport)
    #endif
#else
    #define VESILIBRARYEXPORT
#endif
#include "AtomicSensorInformation.h"
#include "VESIRestbus.h"
#include <list>
#include <functional>


/*! \class VESIAPI
\brief API class to interface with the V-ESI application and the vehicle- and environmentsimulation

The V-ESI API class allows the user to connect to the V-ESI application.
The user can request sensor information of a specific sensor and request corresponding groundtruth- and restbus-information.
The user can apply feedback to the physical sensor simulation (e.g. dynamic adaptation of the camera exposure time).
The user can propagate perception results or control commands to the environment simulation.
*/
class VESILIBRARYEXPORT VESIAPI
{

    public:
        static constexpr uint32_t MAJOR_VERSION = 1;
        static constexpr uint32_t MINOR_VERSION = 0;
        static constexpr uint32_t PATCH_VERSION = 0;

        VESIAPI();
        ~VESIAPI();
        //! Function to set the hostname of the SimManager, i.e. the V-ESI application.
        /*!
        \param simManagerHost The hostname or IP adress of the system on which the SimManager, i.e. the V-ESI application, is running.
        */
        void setSimManagerHost(std::string simManagerHost);
        //! Function to get the configured hostname of the SimManager, i.e. the V-ESI application.
        /*!
        \returns The hostname or IP adress of the system on which the SimManager, i.e. the V-ESI application, is running.
        */
        std::string getSimManagerHost();
        //! Function to set the port of the SimManager, i.e. the V-ESI application.
        /*!
        \param simManagerPort The port(TCP) on which the SimManager, i.e. the V-ESI application, is supposed to be reached.
        */
        void setSimManagerPort(uint16_t simManagerPort);
        //! Function to get the configured port of the SimManager, i.e. the V-ESI application.
        /*!
        \returns The port(TCP) on which the connection to the SimManager, i.e. the V-ESI application, is supposed to be established.
        */
        uint16_t getSimManagerPort();
        //! Function to set the hostname of the traffic-,vehicle- and environment-simulation.
        /*!
        \param asmHostName The hostname or IP adress of the system on which the traffic-,vehicle- and environment-simulation is running.
        */
        void setASMHost(std::string asmHostName);
        //! Function to get the configured hostname of the traffic-,vehicle- and environment-simulation.
        /*!
        \returns The hostname or IP adress of the system on which thetraffic-,vehicle- and environment-simulation is supposed to be reached.
        */
        std::string getASMHost();
        //! Function to set the list of required sensor IDs, i.e. a list of all sensors that are supposed to be requested within one simulation step.
        /*!
        \param requiredSensorIds list of required sensor IDs, i.e. a list of all sensors that are supposed to be requested within one simulation step.
        */
        void setRequiredSensorIDs(std::list<uint8_t> requiredSensorIds);
        //! Function to get the list of configured required sensor IDs, i.e. a list of all sensors that are supposed to be requested within one simulation step.
        /*!
        \returns list of required sensor IDs, i.e. a list of all sensors that are supposed to be requested within one simulation step.
        */
        std::list<uint8_t> getRequiredSensorIDs();
        //! Function to set the requirement of restbus simulation.
        /*!
        \param data_required Option whether or not restbus signals are required by the client.
        */
        void setRestbusDataRequired(bool data_required);
        //! Function to get the configured option whether of not restbus simulation is required.
        /*!
        \returns Option whether or not restbus signals are required by the client.
        */
        bool getRestbusDataRequired();
        //! Function to set the requirement of groundtruth information.
        /*!
        \param data_required Option whether or not groundtruth information is required by the client.
        */
        void setGroundtruthDataRequired(bool data_required);
        //! Function to get the configured option whether of not groundtruth information is required by the client.
        /*!
        \returns Option whether or not groundtruth information is required by the client.
        */
        bool getGroundtruthDataRequired();
        //! Function to set the requirement of custom signals.
        /*!
        \param data_required Option whether or not custom signals are required by the client.
        */
        void setCustomDataRequired(bool data_required);
        //! Function to get the configured option whether of not custom signals are required.
        /*!
        \returns Option whether or not custom signals are required by the client.
        */
        bool getCustomDataRequired();
        //! Function to set the list of provided control data IDs, i.e. a list of all closed loop interfaces that are supposed to be provided with data.
        /*!
        \param providedControlDataIds list of provided control data IDs. Each ID corresponds to the port of the V-ESI Closed-Loop-Interface.
        */
        void setProvidedControlDataIDs(std::list<uint16_t> providedControlDataIds, bool useTCPConnection = false);
        //! Function to get the list of configured provided control data IDs, i.e. a list of all closed loop interfaces that are supposed to be provided with data.
        /*!
        \returns list of configured provided control data IDs. Each ID corresponds to the port of the V-ESI Closed-Loop-Interface.
        */
        std::list<uint16_t> getProvidedControlDataIDs();
        //! Function to establish all necessary connections to the SimManager as well as to all closed-loop interfaces. It is mandatory to establish a connection before requesting sensor information or sending feedback-/control/perception-signals. Throws if a connection error occurs.
        /*!
        \sa disconnect()
        */
        void connect();
        //! Function to achieve a clean termination of all existing connections. Should only be used after a connection has been established by the connect() function.
        /*!
        \sa connect()
        */
        void disconnect();
        //! Function to request the sensor data of a specific sensor by its unique ID.
        /*!
        \param id The unique id of the specific sensor. This ID has to correspond with the ID set in the used MotionDesk/SensorSim project.
        \param atomic_sensor_information Reference to an object of the class AtomicSensorInformation. This class will contain the requested sensor data as well as relevant meta information.
        \sa AtomicSensorInformation
        */
        void requestSensorData(uint8_t id, AtomicSensorInformation* atomic_sensor_information);
        //! Function to request restbus signals, e.g. vehicle-speed, steering-angle, ...(Provided that restbus simulation is enabled and the model is prepared for restbus simulation).
        /*!
        \param restbus_signals Reference to a struct that stores the requested restbus signals.
        */
        void requestRestbusData(VESIRestbus::VESIRestbus* restbus_signals);
        //! Function to request groundtruth information in the Open-Simulation-Interface (OSI) format (Provided that groundtruth data simulation has been enabled and the model is prepared for OSI-GT data exchange).
        /*!
        \param ground_truth_data Byte-Vector that stores the requested data.
        */
        void requestGroundTruthData(std::vector<uint8_t>* ground_truth_data);
        //! Function to request custom data from the environment/traffic simulation (Provided that the custom data channel has been enabled and the model is prepared for custom data exchange).
        /*!
        \param custom_data Byte-Vector that stores the requested data.
        */
        void requestCustomData(std::vector<uint8_t>* custom_data);
        //! Function to send perception results to a Closed-Loop-Interface
        /*!
        \param controlDataId ID of the V-ESI Closed-Loop-Interface to which the result is supposed to be send
        \param controlData Pointer to the result that is supposed to be send to the Closed-Loop-Interface
        \param size Size of the result data in byte
        */
        void sendControlData(int controlDataId, void* controlData, uint64_t size);
        //! Function to increment the current API simulation time.
        /*!
        \param timeDelta Time-increment in seconds
        */
        void increaseSimulationTime(double timeDelta);
        //! Function to get the current API simulation time
        /*!
        \returns Current API simulation time
        */
        double getSimulationTime();

    private:
        class api_implementation;
        api_implementation* implementation;


};

