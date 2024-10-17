"""monde1_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor
import time

vitesse_max = 10.0
vitesse_sharp = 9.0
vitesse_min = 8.0

class Motors():
    right_wheel = None
    left_wheel = None
        
    def __init__(self, robot:Robot):
        self.right_wheel:Motor = robot.getDevice('left wheel')
        self.left_wheel:Motor = robot.getDevice('right wheel')

        self.right_wheel.setPosition(float('inf'))
        self.left_wheel.setPosition(float('inf'))

        self.right_wheel.setVelocity(0.0)
        self.left_wheel.setVelocity(0.0)

    # Aller tout droit 
    def straight(self):
        self.right_wheel.setVelocity(vitesse_max)
        self.left_wheel.setVelocity(vitesse_max)

    # Aller en arrière
    def back(self):
        self.right_wheel.setVelocity(-vitesse_max)
        self.left_wheel.setVelocity(-vitesse_max)

    # Aller sur la gauche
    def left(self):
        self.right_wheel.setVelocity(vitesse_max)
        self.left_wheel.setVelocity(vitesse_min)

    # Aller sur la droite
    def right(self):
        self.right_wheel.setVelocity(vitesse_min)
        self.left_wheel.setVelocity(vitesse_max)

    # Pour le pont
    def sharp_right(self):
        self.right_wheel.setVelocity(vitesse_sharp)
        self.left_wheel.setVelocity(vitesse_max)

class Sensors(Robot):
    sensors_names = [f'so{i}' for i in range(16)]

    sensors = {}

    def __init__(self, robot:Robot):
        for i, name in enumerate(self.sensors_names):
            sensor:DistanceSensor = robot.getDevice(name)
            sensor.enable(int(robot.getBasicTimeStep()))
            self.sensors[f'SO_{i}'] = sensor

    def get_Distance_Value_SO_0 (self):
        return self.sensors['SO_0'].getValue() # Capteur droit, Valeurs empiriques entre 830 et 870
    
    def get_Distance_Value_SO_7 (self):
        return self.sensors['SO_7'].getValue() # Capteur gauche, Valeurs empiriques entre 815 et 855
    
    def get_Distance_Value_SO_2 (self):
        return self.sensors['SO_2'].getValue() # Capteur avant gauche
    
    def get_Distance_Value_SO_5 (self):
        return self.sensors['SO_5'].getValue() # Capteur avant droit


class RobotAnnexe(Robot):

    def __init__(self):
        super().__init__()
        self.motors = Motors(self)
        self.sensors = Sensors(self)

    def run(self):
        #self.motors.straight()
        val_SO_0 = self.sensors.get_Distance_Value_SO_0()
        val_SO_7 = self.sensors.get_Distance_Value_SO_7()
        val_SO_2 = self.sensors.get_Distance_Value_SO_2()
        val_SO_5 = self.sensors.get_Distance_Value_SO_5()
        print({'Avant gauche': val_SO_2, 
              'Avant droit': val_SO_5, 
              'Droit': val_SO_0,
              'Gauche': val_SO_7})
        #if val_SO_2 > 0:
        #    self.motors.left()
        #elif val_SO_5 > 0:
        #    self.motors.right()
        if val_SO_0 < 830 or val_SO_7 > 870 or val_SO_0 == 0 or val_SO_5 > 0:
            self.motors.right()
        elif val_SO_0 > 870 or val_SO_7 < 830 or val_SO_7 == 0 or val_SO_2 > 0:
            self.motors.left()
        elif 830 < val_SO_0 < 870 and 830 < val_SO_7 < 870 or val_SO_2 == 0 and val_SO_5 == 0:
            self.motors.straight()
        elif val_SO_7 == 0 and val_SO_5 < 850 and val_SO_0 == 0 and val_SO_2 == 0:
            self.motors.left()
        elif val_SO_7 < 850 and val_SO_5 == 0 and val_SO_0 == 0 and val_SO_2 == 0:
            self.motors.right()
        elif val_SO_7 == 0 and val_SO_5 == 0 and val_SO_0 == 0 and val_SO_2 == 0:
            self.motors.straight()
        
    

# create the Robot instance.
robot = RobotAnnexe()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:

#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    robot.run()
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    

# Enter here exit cleanup code.
