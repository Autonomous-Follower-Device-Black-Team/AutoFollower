#ifndef ESP_NOW_NODE
#define ESP_NOW_NODE

#include <Arduino.h>
#include <ESP32_NOW.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <esp_wifi.h>

#define TaskDelayLength pdMS_TO_TICKS(100)
#define ACK_TIMEOUT_MS 10000     // Transmission timeout length (ms).

typedef BaseType_t (* ProcessDataCallback)(const char *);

const uint8_t ESPNOW_WIFI_CHANNEL = 6;      // Wi-Fi channel that system transmission occurs in.
const uint8_t ESPNOW_DATA_SIZE = 8;         // Size of system data payload sent (in bytes)
const int ESPNOW_TASK_DEPTH = 8192;         // Stack size of ESP-NOW tasks.

// ESP32-S3 Mac addrresses.
const uint8_t dev_S3_A[] = {0x24, 0xEC, 0x4A, 0x09, 0xC8, 0x00};
const uint8_t dev_S3_B[] = {0x24, 0xEC, 0x4A, 0x09, 0xC8, 0xC8};

// ESP32 Mac addresses.
const uint8_t dev_C[] = {0x10, 0x06, 0x1C, 0x97, 0x94, 0x38};
const uint8_t dev_D[] = {0x10, 0x06, 0x1C, 0x98, 0x56, 0x28};

enum _mode : char {
    Transmitter = 'T',  // Transmitter mode.
    Receiver = 'R',     // Receiver mode.
    Unassigned = 'U'    // Unassigned.
};
typedef enum _mode Mode;

enum _header : uint8_t {
    ACK = 10,            // Header indidcating this is an acknowledgement of the previously received message. Might be uselss.
    HANDSHAKE = 11,      // Header indicating this is a connection establishing message.
    WAVE = 13,           // Header indicating this is a connection terminating message.
    TRIGGER_PING = 14    // Header indicating that a trigger event is about to happen.
};
typedef enum _header Header;

enum _ack_messages : char {
    Received_Handshake = 'H',
    Received_Wave = 'W',
    Received_Ping = 'G'
};
typedef enum _ack_messages AckMessage;

#define HS_MSG "Received Handshake Request"
#define WV_MSG "Received Wave Request"

struct _esp_now_packet {
    Header header;                      // Holds info on the kind of data being exchanged.
    AckMessage ack;                     // Holds info on the kind of data last received by the sending node.
    char data[ESPNOW_DATA_SIZE * 8];    // Holds data currently being exchanged.
};
typedef struct _esp_now_packet ESP_NOW_PACKET;

extern TaskHandle_t esp_now_tx_rx_handle;           // Transmission and Reception task handle.
extern TaskHandle_t esp_now_process_data_handle;    // Data processing task handle.

void esp_now_tx_rx_task(void *pvParams);            // Transmit and receive information from peer.
void esp_now_process_data_task(void *pvParams);     // Process data received.

class EspNowNode : ESP_NOW_Peer {
    private:
        Mode mode = Mode::Unassigned;               // Is this node is a transmitter or receiver.
        bool esp_now_setup = false;                 // Is ESP Now setup for this node.        
        bool waitingForData = false;                // Is this node waiting for data.
        bool justStarted = true;                    // Has this node just begun in network.
        bool isPaused = false;                      // Has this node (transmission or reception) been paused.
        bool hasFoundPeer = false;                  // Has this node found its peer. 
        bool ackRequired = false;
        
        uint8_t peerMacAddress[6];                  // Address of this nodes peer.
        inline static ESP_NOW_PACKET outgoingData;  // Storage for the data to be transmitted from this node.
        inline static ESP_NOW_PACKET incomingData;  // storage for the data received by this node.

        /**
         * Intializes Wi-Fi on the ESP, specifically begins
         * Wi-Fi in station mode a required by ESP-NOW.
         */
        void initWifi();

        /**
         * Initializes ESP Now for use on the ESP.
         */
        void initESPNOW();

        /**
         * Transmits this nodes message over ESP NOW.
         */
        bool send_message();

        /**
         * Creates this nodes message to be transmitted over 
         * ESP-NOW.
         */
        void buildTransmission();

        /**
         * Re regsiters the peer of this node.
         */
        void reRegisterPeer();

        /**
         * Grab the header of the packet to be processed.
         */
        Header getHeaderToProcess();

        /**
         * Grab the payload of the packet to be processed.
         */
        const char* getDataToProcess();

        /**
         * Initialize all system tasks.
         */
        void initTasks();

        /**
         * Start the system Tx/Rx task.
         */
        BaseType_t beginCommunicationTask();

        /**
         * Start the system data processing task.
         */
        BaseType_t beginProcessDataTask();

        ProcessDataCallback handshakeCallback = NULL;
        ProcessDataCallback waveCallback = NULL;
        ProcessDataCallback infoReceivedCallback = NULL;
        ProcessDataCallback dataSentCallBack = NULL;

    public:
        EspNowNode( 
                const uint8_t* peerMacAddress,      // Mac address of the device to be registered as this nodes peer.
                Mode nodeMode,                      // This nodes Mode in network (tranmitter/receiver).
                bool ackRequired = false            // Does this node require EXPLICIT acknowledgement. Defaults to `false`.
            ) :
            ESP_NOW_Peer(peerMacAddress, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL)
        {   
            // Set the peer mac address.
            for(int i = 0; i < 6; i++) this->peerMacAddress[i] = peerMacAddress[i];

            // Initialize outgoing data packet. First to be transmitted by the Master. 
            outgoingData.header = Header::HANDSHAKE;
            outgoingData.ack = AckMessage::Received_Handshake;
            String tmp = "Nothing outgoing";
            sprintf(outgoingData.data, "%s", tmp.c_str());
            
            // Initialize incoming data packet.
            incomingData.header = Header::HANDSHAKE;
            incomingData.ack = AckMessage::Received_Handshake;
            tmp = "Nothing incoming";
            sprintf(incomingData.data, "%s", tmp.c_str());

            // Select mode. Transmitter begins ready to transmit. Receiver begins waiting.
            waitingForData = (nodeMode == Mode::Transmitter) ? false : true;
            this->ackRequired = ackRequired;
            this->mode = nodeMode;
        }
        
        // Destructor to preserve memory integrity when ending ESP-NOW transmission.
        ~EspNowNode() { remove(); }
        
        // Methods to register appropriate callbacks. To be further fleshed out and changed.        
        bool registerProcessHandshakeCallBack(ProcessDataCallback pcb);
        bool registerProcessWaveCallBack(ProcessDataCallback pcb);
        bool registerProcessInfoReceivedCallBack(ProcessDataCallback pcb);
        bool registerDataSentCallBack(ProcessDataCallback pcb);

        // Methods to facilitate ESP-NOW transmission between nodes.
        bool start();
        bool end();
        void pause();
        void unpause();
        bool isTransmissionPaused();
        bool is_esp_now_setup();
        bool isNodeTransmitter();
        
        // Methods for processing transmission and reception events between nodes.
        void onReceive(const uint8_t *data, size_t len, bool broadcast) override;
        void onSent(bool success) override;

        bool transmit();
        bool readyToTransmit(); 
        void setReadyToTransmit(bool status);
        Header determineNextHeader();
        AckMessage determineNextAck();
        String determineNextData();

        void showDataReceived();
        void showDataTransmitted();
        bool proccessPacket();
        void reRegister();

        // Methods for identifying info on nodes in the network.
        String getThisMacAddress();
        String getPeerMacAddress();
    };


#endif