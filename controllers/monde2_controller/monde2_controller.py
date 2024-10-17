"""monde2_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, PositionSensor, DistanceSensor, RangeFinder, Camera, GPS, Compass
from math import sqrt


class Motors():

    ###### Roues
    F_R_wheel = None
    F_L_wheel = None
    R_L_wheel = None
    R_R_wheel = None

    def __init__(self, robot:Robot):
        #roues AVANT
        self.F_L_wheel:Motor = robot.getDevice('fl_wheel_joint')   # le :Motor permet de typer la variable en type "Motor"
        self.F_R_wheel:Motor = robot.getDevice('fr_wheel_joint')

        #roues ARRIERES
        self.R_L_wheel:Motor = robot.getDevice('rl_wheel_joint')
        self.R_R_wheel:Motor = robot.getDevice('rr_wheel_joint')

        #### GESTION DES ROTATIONS DE ROUES
            # roue avant gauche
        self.F_L_wheel.setPosition(float('inf'))  # inf pour infini, ici on autorise le motor a tourner à l'infini autour de son axe de rotation . 
        self.F_L_wheel.setVelocity(0.0)  # la vitesse de rotation est initialisée ici à 0
            # roue avant droite
        self.F_R_wheel.setPosition(float('inf')) 
        self.F_R_wheel.setVelocity(0.0)
            # roue arriere gauche 
        self.R_L_wheel.setPosition(float('inf')) 
        self.R_L_wheel.setVelocity(0.0)
            # roue arriere droite
        self.R_L_wheel.setPosition(float('inf')) 
        self.R_L_wheel.setVelocity(0.0)

    def go_straight(self):
        self.F_L_wheel.setVelocity(20.0)
        self.F_R_wheel.setVelocity(20.0)
        self.R_L_wheel.setVelocity(20.0)
        self.R_R_wheel.setVelocity(20.0)

    def back_straight(self):
        self.F_L_wheel.setVelocity(-2.0)
        self.F_R_wheel.setVelocity(-2.0)
        self.R_L_wheel.setVelocity(-2.0)
        self.R_R_wheel.setVelocity(-2.0)

    def go_droite(self):
        self.F_L_wheel.setVelocity(20.0)
        self.F_R_wheel.setVelocity(5.0)
        self.R_L_wheel.setVelocity(20.0)
        self.R_R_wheel.setVelocity(5.0)

    def back_droite(self):
        self.F_L_wheel.setVelocity(-20.0)
        self.F_R_wheel.setVelocity(-5.0)
        self.R_L_wheel.setVelocity(-20)
        self.R_R_wheel.setVelocity(-5.0)

    def go_gauche(self):
        self.F_L_wheel.setVelocity(5.0)
        self.F_R_wheel.setVelocity(20.0)
        self.R_L_wheel.setVelocity(5.0)
        self.R_R_wheel.setVelocity(20.0)

    def back_gauche(self):
        self.F_L_wheel.setVelocity(-5.0)
        self.F_R_wheel.setVelocity(-10.0)
        self.R_L_wheel.setVelocity(-5.0)
        self.R_R_wheel.setVelocity(-10.0)

    def stop(self):
        self.F_L_wheel.setVelocity(0.0)
        self.F_R_wheel.setVelocity(0.0)
        self.R_L_wheel.setVelocity(0.0)
        self.R_R_wheel.setVelocity(0.0)



class Captors():  #PositionSensor, DistanceSensor, RangeFinder, Camera, GPS

    ####### CAPTEURS

    # de position
    F_L_motor_sensor = None
    F_R_motor_sensor = None
    R_L_motor_sensor = None
    R_R_motor_sensor = None

    # de distance
    F_L_range = None
    F_R_range = None
    R_L_range = None
    R_R_range = None

    # de profondeur
    Cam_depth = None

    # camera
    cam= None

    # gps
    gps = None
    x_gps = None
    y_gps = None
    

    ######## autre variable
    

    def __init__(self, robot:Robot):

        time = int(robot.getBasicTimeStep())
        

        # de position   
        self.F_R_motor_sensor:PositionSensor = robot.getDevice('front right wheel motor sensor')
        self.F_R_motor_sensor.enable(time)
        self.F_L_motor_sensor:PositionSensor = robot.getDevice('front left wheel motor sensor')
        self.F_L_motor_sensor.enable(time)
        self.R_R_motor_sensor:PositionSensor = robot.getDevice('rear right wheel motor sensor')
        self.R_R_motor_sensor.enable(time)
        self.R_L_motor_sensor:PositionSensor = robot.getDevice('rear left wheel motor sensor') 
        self.R_L_motor_sensor.enable(time)

        # de distance
        self.F_L_range:DistanceSensor = robot.getDevice('fl_range')
        self.F_L_range.enable(time)
        self.F_R_range:DistanceSensor = robot.getDevice('fr_range')
        self.F_R_range.enable(time)
        self.R_L_range:DistanceSensor = robot.getDevice('rl_range')
        self.R_L_range.enable(time)
        self.R_R_range:DistanceSensor = robot.getDevice('rr_range')
        self.R_R_range.enable(time)

        # de profondeur
        self.Cam_depth:RangeFinder = robot.getDevice('camera depth')
        self.Cam_depth.enable(time)

        # gps
        self.gps:GPS = robot.getDevice('gps')
        self.gps.enable(time)

        # camera
        self.cam:Camera = robot.getDevice('camera rgb')
        self.cam.enable(time)
    
    def getCoordo_gps(self):
        coordo = (self.gps.getValues() )  
        print(coordo)
        self.x_gps = coordo[1]
        self.y_gps = coordo[2]
        return coordo

    def calcDistance_A_B(self):
        dist_A_B = ( sqrt( ( (self.x_gps - 43 )*(self.x_gps - 43 ) ) + ((self.y_gps - 51 )*(self.y_gps - 51 )) )  )  
        print(dist_A_B)
        return dist_A_B
    
    def calc_dist_x(self):
        self.getCoordo_gps()
        dist_en_x = sqrt( ( (self.x_gps - 43 ) * (self.x_gps - 43 ) ) ) 
        return dist_en_x

    def calc_dist_y(self):
        self.getCoordo_gps()
        dist_en_y = sqrt (((self.y_gps - 51 ) * (self.y_gps - 51 )) )
        return dist_en_y
        


class DuduRobot(Robot):

    def __init__(self):
        super().__init__()

        self.motors = Motors(self)
        self.captors = Captors(self)
        #self.mode = 0 # 0 = "exploration "   sinon sera  1 = " evitement d'obstacle"

    
    #### controles des obstacle dans l environnement 
    def verif_gh(self):
        dist_obs_gh = self.captors.F_L_range.getValue()
        
        return dist_obs_gh
    
    def verif_dt(self):
        dist_obs_dt = self.captors.F_R_range.getValue()
        
        return dist_obs_dt
    
    def verif_ar_gh(self):
        dist_obs_ar_gh = self.captors.R_L_range.getValue()
        
        return dist_obs_ar_gh

    def verif_ar_dt(self):
        dist_obs_ar_dt = self.captors.R_R_range.getValue()
        
        #print(dist_obs_ar_dt)
        return dist_obs_ar_dt

    #### Gestion GPS

    def position_robot(self):
        coordo_gps_robot =  self.captors.gps.getCoordinateSystem()
        print(coordo_gps_robot)
        return coordo_gps_robot
    
    #### comportement

    def eviter_obstacle(self):

        if (robot.verif_dt() < 2) and (robot.verif_gh() < 2):
            print('detection mur avant')
            robot.motors.stop()
            robot.motors.back_straight()

        elif (robot.verif_ar_dt() < 0.5) and (robot.verif_ar_gh() < 0.5):
            print('detection mur arrière')
            
            robot.motors.stop()
            robot.motors.go_straight()

        elif robot.verif_gh() < 2:
            print('obstacle a gauche')


            robot.motors.go_droite()

        elif robot.verif_dt() < 2 :
            print('obstacle a droite')


            robot.motors.go_gauche()

        elif robot.verif_ar_dt() < 1:
            print('obstacle a droite')


            robot.motors.back_gauche()

        elif robot.verif_ar_gh() < 1:
            print('obstacle a gauche')


            robot.motors.back_droite()

        else:
            
            robot.motors.go_straight()

        


####################### MAIN #######################################

# create the Robot instance.
robot = DuduRobot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:


        robot.eviter_obstacle()
    
    

    

    


    
    
    

    
    



    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
