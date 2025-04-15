'''
Copied from previous CSE team's AI model
'''

from abc import ABC, abstractmethod
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
from time import sleep
import pygame
import cv2
import pyautogui

# Get screen dimensions
screen_width, screen_height = pyautogui.size()

# abstract class each state will implement
class AbstractState(ABC):
    # these are all variables that must persist when updating
    # these are default values, not used (see constructor)
    currentServoX = 90
    currentServoY = 90
    increment = 1
    waitTime = 0.5
    goalWindowWidth = 0.1
    camera = None
    model = None
    xServo = None
    yServo = None
    iterations = 0

    @abstractmethod
    def ManualOverride(self):
        pass

    @abstractmethod
    def Execute(self, deltaTime):
        pass


class EvaluatingAnatomyState(AbstractState):
    def __init__(self, oldState=None, increment=None, waitTime=None, goalWindowWidth = None,camera=None, model=None, xServo=None, yServo=None):
        if oldState is not None:
            # transfer all variables that must be retained when switching states
            self.currentServoX = oldState.currentServoX
            self.currentServoY = oldState.currentServoY
            self.increment = oldState.increment
            self.waitTime = oldState.waitTime
            self.goalWindowWidth = oldState.goalWindowWidth
            self.camera = oldState.camera
            self.model = oldState.model
            self.xServo = oldState.xServo
            self.yServo = oldState.yServo
            self.iterations = oldState.iterations
        else:
            # instantiate a new state for machine startup
            self.currentServoX = 90
            self.currentServoY = 90
            self.increment = increment
            self.waitTime = waitTime
            self.goalWindowWidth = goalWindowWidth
            self.camera = camera
            self.model = model
            self.xServo = xServo
            self.yServo = yServo
            self.iterations = 0

    def ManualOverride(self):
        return ManualState(self)

    # for the anatomy evaluation state, it will gather camera input, detect anatomy, and determine direction to
    # move based on bounding box locations of detected anatomy
    def Execute(self, deltaTime):
        # read in most recent camera frame
        ret, frame = self.camera.read()

        # use model to detect anatomy in frame
        results = self.model.predict(frame, verbose=True, imgsz=352)

        # annotate and display the frame with the detected bounding boxes
        for r in results:
            annotator = Annotator(frame)

            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                c = box.cls
                annotator.box_label(b, self.model.names[int(c)] + " " + str(box.conf))
        
        # Resize the frame to fit the screen
        h, w, _ = frame.shape
        scale = min(screen_width / w, screen_height / h)  # Calculate the scaling factor
        new_width = int(w * scale)
        new_height = int(h * scale)
        resized_frame = cv2.resize(annotator.result(), (new_width, new_height))
        
        # Display the resized frame
        cv2.imshow('frame', resized_frame)

        # get the normalized bounding box locations to analyze anatomy positions during frame
        # currently only reacts to tracheas (box.cls == 0)
        # trachea=0, epiglottis=1, uvula=2
        normBox = None
        for box in results[0].boxes:
            if box.cls == 0:
                normBox = box.xywhn

        if normBox is not None:
            centerX = normBox[0][0]
            centerY = normBox[0][1]
            stepX = 0
            stepY = 0

            # add increment to occur if bounding box is far enough away from the center of the frame
            # note that increment is scaled down if the box is near the center
            # this was the last thing we did before the semester ended, so these values are just hardcoded
            if centerX > 0.6 + self.goalWindowWidth:
                stepX = self.increment
            elif centerX < 0.4 - self.goalWindowWidth:
                stepX = self.increment * -1.0
            elif centerX > 0.5 + self.goalWindowWidth:
                stepX = self.increment * 0.5 
            elif centerX < 0.5 - self.goalWindowWidth:
                stepX = self.increment * 0.5 * -1.0

            if centerY > 0.6 + self.goalWindowWidth:
                stepY = self.increment * -1.0
            elif centerY < 0.4 - self.goalWindowWidth:
                stepY = self.increment
            elif centerY > 0.5 + self.goalWindowWidth:
                stepY = self.increment * 0.5 * -1.0 
            elif centerY < 0.5 - self.goalWindowWidth:
                stepY = self.increment * 0.5

            # put bounds on the servo angles being set (must be between 0 and 180)
            targetX = self.currentServoX + stepX
            if targetX >= 180:
                targetX = 180
            elif targetX <= 0:
                targetX = 0

            targetY = self.currentServoY + stepY
            if targetY >= 180:
                targetY = 180
            elif targetY <= 0:
                targetY = 0
            self.currentServoX = targetX
            self.currentServoY = targetY
            return AimingState(self, targetX, targetY)

        return AimingState(self, self.currentServoX, self.currentServoY)

# this state takes in a target x and y position for the respective servos and sets the value, then waits for the
# determined waiting period before action increments
class AimingState(AbstractState):
    goalServoX = 0
    goalServoY = 0

    def __init__(self, oldState, targetX, targetY):
        # transfer all variables that must be retained when switching states
        self.currentServoX = oldState.currentServoX
        self.currentServoY = oldState.currentServoY
        self.increment = oldState.increment
        self.waitTime = oldState.waitTime
        self.goalWindowWidth = oldState.goalWindowWidth
        self.camera = oldState.camera
        self.model = oldState.model
        self.xServo = oldState.xServo
        self.yServo = oldState.yServo
        self.iterations = oldState.iterations

        self.goalServoX = targetX
        self.goalServoY = targetY

    def ManualOverride(self):
        return ManualState(self)

    def Execute(self, deltaTime):
        if self.xServo is not None:
            self.xServo.angle = self.goalServoX
        if self.yServo is not None:
            self.yServo.angle = self.goalServoY
        sleep(self.waitTime)
        return EvaluatingAnatomyState(self)

# UNCOMPLETED
# this state is meant to control the linear actuator and move the device further down the throat.
# intended to be called once the machine is aimed correctly based on the anatomy it has detected
class MovingState(AbstractState):
    currentDistance = 0
    goalDistance = 0

    def ManualOverride(self):
        return ManualState(self)

    def Execute(self, deltaTime):
        # LOGIC HERE WILL MOVE THE LINEAR ACTUATOR FURTHER DOWN THE THROAT A SPECIFIED DISTANCE,
        # THEN RETURN AN EvaluatingAnatomyState
        return EvaluatingAnatomyState()

# UNCOMPLETED
# meant to control the linear actuator out of the throat in the case something goes wrong (opposite of MovingState)
class RetractingState(AbstractState):
    currentDistance = 0
    goalDistance = 0

    def ManualOverride(self):
        return ManualState(self)

    def Execute(self, deltaTime):
        # LOGIC HERE WILL MOVE THE LINEAR ACTUATOR OUT OF THE THROAT A SPECIFIED DISTANCE,
        # THEN RETURN AN EvaluatingAnatomyState
        return EvaluatingAnatomyState()

# this state is when the machine has been toggled to run through manual controls
# currently done through keyboard inputs
# NOT CURRENTLY FUNCTIONAL
class ManualState(AbstractState):
    def __init__(self, oldState):
        self.currentServoX = oldState.currentServoX
        self.currentServoY = oldState.currentServoY
        self.increment = oldState.increment
        self.waitTime = oldState.waitTime
        self.goalWindowWidth = oldState.goalWindowWidth
        self.camera = oldState.camera
        self.model = oldState.model
        self.xServo = oldState.xServo
        self.yServo = oldState.yServo
        self.iterations = oldState.iterations
        
        pygame.init()
    
    def ManualOverride(self):
        return self

    def Execute(self, deltaTime):
        stepX = 0
        stepY = 0
        
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s:
                    stepY = self.increment
                elif event.key == pygame.K_w:
                    stepY = self.increment * -1.0
                elif event.key == pygame.K_a:
                    stepX = self.increment
                elif event.key == pygame.K_d:
                    stepX = self.increment * -1.0
                elif event.key == pygame.K_e:
                    pass
                    #return_servo1()
                elif event.key == pygame.K_q:
                    pass
                    #return_servo2()
                elif event.key == pygame.K_UP:
                    pass
                    #stepper_key_up_pressed = True
                elif event.key == pygame.K_DOWN:
                    pass
                    #stepper_key_down_pressed = True
                
        targetX = self.currentServoX + stepX
        if targetX >= 180:
            targetX = 180
        elif targetX <= 0:
            targetX = 0

        targetY = self.currentServoY + stepY
        if targetY >= 180:
            targetY = 180
        elif targetY <= 0:
            targetY = 0
        self.currentServoX = targetX
        self.currentServoY = targetY
        if self.xServo is not None:
            self.xServo.angle = targetX
        if self.yServo is not None:
            self.yServo.angle = targetY
        sleep(self.waitTime)
        return self