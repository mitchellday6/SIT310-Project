from std_srvs.srv import Empty
from robot_interfaces.srv import Move, Turn

import rclpy
from rclpy.node import Node
import time
import speech_recognition as sr

class Client(Node):
    def __init__(self):
        super().__init__('robot_client_service')
        self.tasks = ["go","move", "turn", "stop"]
        self.subTasks = ["forward", "forwards", "backward", "backwards", "left", "right"]
        self.task = ""
        self.subTask = ""
        self.r = sr.Recognizer()
        self.mic = sr.Microphone(device_index=0)
            

    def getAudio(self):
        with self.mic as source:
            self.r.adjust_for_ambient_noise(source)
            print("Please give me a command")
            audio = self.r.listen(source)
            print("processing")

            try:
                textResult = self.r.recognize_google(audio)
                print("Input: "+textResult)
                return textResult
            except sr.UnknownValueError:
                return "Unkown Text"
            except Exception as e:
                print("Error: "+e)
                return False

    def request(self):
        trigService = ""
        if(self.task == "move" or self.task == "go" and (self.subTask == "forward" or self.subTask == "forwards")):
            trigService = "move_forward"
            self.cli = self.create_client(Move, trigService)
            self.req = Move.Request()
            self.req.distance = 50
        if((self.task == "move" or self.task == "go") and (self.subTask == "backward" or self.subTask == "backwards")):
            trigService = "move_backward"
            self.cli = self.create_client(Move, trigService)
            self.req = Move.Request()
            self.req.distance = 50
        if(self.task == "turn" and self.subTask == "right"):
            trigService = 'turn_right'
            self.cli = self.create_client(Turn, trigService)
            self.req = Turn.Request()
            self.req.deg = 90
        if(self.task == "turn" and self.subTask == "left"):
            trigService = 'turn_left'
            self.cli = self.create_client(Turn, trigService)
            self.req = Turn.Request()
            self.req.deg = 90
        if(self.task == "stop"):
            trigService = 'stop'
            self.cli = self.create_client(Move, trigService)
            self.req = Move.Request()

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        try:
            result = self.future.result()
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
        else:
            self.get_logger().info(trigService+" successful")

        time.sleep(1)



    def checkAudioText(self, text):
        # iterate over task words
        for word in self.tasks:
            #if the word is in the speech text
            if word in text:
                # print("Task Word: "+word)
                self.task = word
                if(word == "stop"):
                    self.subTask = "now"
                    return True
                for subTask in self.subTasks:
                    if subTask in text:
                        # print("Sub Task Word: "+subTask)
                        self.subTask = subTask
                        return True
                return False
        return False
        
    
        
def main(args=None):
    rclpy.init(args=args)

    client = Client()
    while(True):
        response = client.getAudio()
        if(response == False):
           break
        if(client.checkAudioText(response)):
            client.request()

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
