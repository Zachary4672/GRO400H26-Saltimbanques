import time
import paho.mqtt.client as mqtt

BROKER_IP = "localhost" #Indique que le host est le PI
BROKER_PORT = 1883  #Port de connection que le HMI utilise

TOPIC_CLIENTS = "$SYS/broker/clients/connected"
TOPIC_START = "start"
TOPIC_STOP = "stop"

#NOTE: Ce programme est en anglais puisque les acronymes (dont les variables) pour ce type de programme sont plus commun en anglais
#NOTE: USING hungarian notation
# --------------------------------------------------
# Global variables
# --------------------------------------------------
iNb_clients = 2 #Number of clients detected (it is initially set to 2 because it takes time to be updated so a value of 2 will make it so that it wont break the code)
bStart_flag = False #Start button
bStop_flag = False #Stop button
iError_Code = 0 #This code is changed when the PI is trying to connect

# --------------------------------------------------
# on_connect: Minor function connecting the broker !!THIS FUNCTION SHOULD NOT BE CALLED EXTERNALLY!!
# --------------------------------------------------
def on_connect(client, userdata, flags, rc):
    global iError_Code
    if rc == 0: #error code
        client.subscribe(TOPIC_CLIENTS) #Connecting to each topics that will receive variables from the HMI
        client.subscribe(TOPIC_START)
        client.subscribe(TOPIC_STOP)
        iError_Code = rc
    else:
        iError_Code = rc

# --------------------------------------------------
# on_message: Minor function doing an action on message received !!THIS FUNCTION SHOULD NOT BE CALLED EXTERNALLY!!
# --------------------------------------------------
def on_message(client, userdata, msg):
    global iNb_clients, bStart_flag, bStop_flag

    payload = msg.payload.decode("utf-8").strip().lower() #decode the message and assign it to a variable

    if msg.topic == TOPIC_CLIENTS:
        iNb_clients = int(payload)

    if msg.topic == TOPIC_START:
        if payload == "true":
            bStart_flag = True
        else:
            bStart_flag = False

    elif msg.topic == TOPIC_STOP:
        if payload == "true":
            bStop_flag = True
        else:
            bStop_flag = False
# ---------------------------------------------------
# is_hmi_connected: Minor function that return the number of devices connected !!THIS FUNCTION SHOULD NOT BE CALLED EXTERNALLY!!
# ---------------------------------------------------
def is_hmi_connected():

    if iNb_clients >= 2: # The number 2 is chosen because the actual number of clients is also counting the PI so the minimum is 2 connection
        return True
    else:
        return False
    


# ---------------------------------------------------
# consume_flag: return the value according to what was the flag name !!THIS FUNCTION SHOULD NOT BE CALLED EXTERNALLY!!
# ---------------------------------------------------
def consume_flag(flag_name):
    global bStart_flag,bStop_flag

    if not is_hmi_connected():
        return -1
    
    if flag_name == "start":
        value = bStart_flag
        return value
    
    elif flag_name == "stop":
        value = bStop_flag
        return value
    
    return False

# --------------------------------------------------
# is_started: This function return if the start button is pushed or not
# --------------------------------------------------
def is_started():
    return consume_flag("start") 
    #IF THE VALUE IS -1: THE CONNECTION IS LOST WITH THE HMI AND THE ROBOT SHOULD BE STOPPED
    #IF THE VALUE IS TRUE: THE START BUTTON GOT PUSHED AND THE ROBOT SHOULD START
    #IF THE VALUE IS FALSE: THE START BUTTON WASN'T PUSHED, VERIFY WITH THE STOP BUTTON TO KNOW IF THE ROBOT SHOULD STOP

# ---------------------------------------------------
# is_stopped: This function return if the stop button is pushed or not
# ---------------------------------------------------
def is_stopped():
    return consume_flag("stop")
    #IF THE VALUE IS -1: THE CONNECTION IS LOST WITH THE HMI AND THE ROBOT SHOULD BE STOPPED
    #IF THE VALUE IS TRUE: THE STOP BUTTON GOT PUSHED AND THE ROBOT SHOULD STOP
    #IF THE VALUE IS FALSE: THE STOP BUTTON WASN'T PUSHED, VERIFY WITH THE START BUTTON TO KNOW IF THE ROBOT SHOULD START

# ---------------------------------------------------
# Connect: This function call for a connection of the mqtt ONLY CALL THIS TO START A CONNECTION
# ---------------------------------------------------
def connect():
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER_IP,BROKER_PORT,60)
    client.loop_start()
    if iError_Code != 0:
        return iError_Code
    else:
        return 0
    
# ---------------------------------------------------
# Disconnect: This function call for a disconnection of the mqtt ONLY CALL THIS WHEN A FULL STOP IS NECESSARY
# ---------------------------------------------------
def disconnect():
    client.disconnect()
    client.loop_stop()

# ---------------------------------------------------
# PublishMessage: This function send a message that will be collected by the HMI
# !!! REALLY IMPORTANT !!!: The first parameter is a string containing the TOPIC of the message and the second parameter is the value. 
# Here is a list of TOPIC: (angles in rad and speed in rad per sec)
# 1. J1_angle       <-- The value of this topic is a FLOAT
# 2. J2_angle       <-- The value of this topic is a FLOAT
# 3. J3_angle       <-- The value of this topic is a FLOAT
# 4. J1_speed       <-- The value of this topic is a FLOAT
# 5. J2_speed       <-- The value of this topic is a FLOAT
# 6. J3_speed       <-- The value of this topic is a FLOAT
# 7. Robot_state       <-- The value of this topic is a string informing what is the state of the robot (DROP, PICK, WAITING, SCANNING)
# 8. Item_dropped      <-- The value of this topic is a color followed by a boolean confirming that the pill is dropped in the corresponding container 
# *** Here are the 3 "colors" that would be accepted in the input: "Red", "Yellow", "Other". The input should look like that "Red 1", "Yellow 1" or "Other 1"
# ---------------------------------------------------
def PublishMessage(title, val):
    if title == "Item_dropped":
        client.publish(title, str(val),qos=0, retain=False)
    else:
        client.publish(title, str(val), qos=0, retain=True)
# ----------------------------------------------------
# This section is about setting the clients of MQTT
# ----------------------------------------------------
client = mqtt.Client()
disconnect()
connect()




