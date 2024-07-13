import serial
import time
import depthai as dai
import numpy as np
import paho.mqtt.client as mqtt

class procesos():
    def __init__(self) -> None:
        self.state = 0
        self.vel = 0
        self.flag = 1
        self.pipeline = dai.Pipeline()
        self.q = None
        self.config_camera()
        self.serial_comunication()
        self.mqtt_conection()
    
    def serial_comunication(self):
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.ser.reset_input_buffer()

    def mqtt_conection(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect("mqtt-dashboard.com", 1883, 60)

    def estados1_6(self, estado):
        self.state = self.flag = estado
        if self.state == 1:
            self.ser.write("1\n".encode())
            self.flag=0
        elif self.state == 2:
            self.ser.write("5\n".encode())
            self.ser.write("2\n".encode())
            time.sleep(0.5)
            self.ser.write("4\n".encode())
            self.ser.write((str(self.state+self.vel)+"\n").encode())
        elif self.state == 3:
            self.ser.write("5\n".encode())
            self.ser.write("3\n".encode())
            time.sleep(0.5)
            self.ser.write("4\n".encode())
            self.ser.write((str(self.state+self.vel)+"\n").encode())
        elif self.state == 4:
            self.ser.write("4\n".encode())
        elif self.state == 5:
            self.vel+=1
            if self.vel > 2:
                self.vel = 2
            self.ser.write((str(self.state+self.vel)+"\n").encode())
        elif self.state == 6:
            self.vel-=1
            if self.vel < 0:
                self.vel = 0
            self.ser.write((str(5+self.vel)+"\n").encode())
        if self.state != 0:
            self.state = 0

    def estados_evadir(self):
        lado = self.getObject()
        if lado == 1 and self.flag==1:
            self.ser.write("5\n".encode())
            self.ser.write("3\n".encode())
            time.sleep(0.5)
            self.ser.write("1\n".encode())
            time.sleep(0.5)
            self.ser.write("2\n".encode())
            time.sleep(0.7)
            self.ser.write("1\n".encode())
            time.sleep(0.6)
            self.ser.write("2\n".encode())
            time.sleep(0.7)
            self.ser.write("1\n".encode())
            time.sleep(0.3)
            self.ser.write("3\n".encode())
            time.sleep(0.7)
            self.ser.write("1\n".encode())
            self.ser.write((str(5+self.vel)+"\n").encode())
            lado = 0

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        client.subscribe("proyecto/estado")
    
    def on_message(self, client, userdata, msg):
        message = msg.payload.decode("utf-8").split('/')
        if message[0] != "r":
            self.estados1_6(int(message[1]))

    def config_camera(self):
        monoLeft = self.getMonoCamera(self.pipeline, True)
        monoRight = self.getMonoCamera(self.pipeline, False)
        stereo = self.getStereoPair(self.pipeline, monoLeft, monoRight)
        xoutdepth = self.pipeline.createXLinkOut()
        xoutdepth.setStreamName("depth")
        stereo.depth.link(xoutdepth.input)

    def get_Frame(self,queue):
        frame = queue.get()
        return(frame.getFrame())
    
    def getObject(self): 
        frame = self.get_Frame(self.q)
        frame = np.asarray(frame, np.float32)
        frame = frame/1000.
        x,y,z = np.split(frame, [206,414], axis=1)
        suma_x = np.round(np.sum(x)/x.size,2)
        suma_y = np.round(np.sum(y)/y.size,2)
        suma_z = np.round(np.sum(z)/z.size,2)
        if suma_y < 0.56 or suma_x < 0.56 or suma_z < 0.56:
            return 1
        else:
            return 0

    def getMonoCamera(self, pipeline, isLeft):
        mono = pipeline.createMonoCamera()
        mono.setBoardSocket(dai.CameraBoardSocket.CAM_B if isLeft else dai.CameraBoardSocket.CAM_C)
        mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono.setFps(120)
        return mono

    def getStereoPair(self, pipeline, monoLeft, monoRight):
        stereo = pipeline.createStereoDepth()
        stereo.setLeftRightCheck(True)
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        return stereo

    def start(self):
        try:
            self.client.loop_start()
            with dai.Device(self.pipeline) as device:
                self.q = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
                print("Connected")
                while True:
                    self.estados_evadir()
        except KeyboardInterrupt:
            self.ser.close()
            self.client.loop_stop()
            print("Fin del programa")

if __name__ == "__main__":
    proyecto = procesos()
    proyecto.start()
