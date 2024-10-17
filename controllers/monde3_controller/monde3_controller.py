"""monde3_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, InertialUnit, GPS, Gyro

# Fonction utile pour la rotation des hélices -> sécurité permettant d'éviter de se retourner dans les axes
def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

# Classe d'interface pour les moteurs du drone
class Motors():
    rear_left_motor = None #Moteur arrière gauche
    rear_right_motor = None #Moteur arrière droit
    front_left_motor = None #Moteur avant gauche
    front_right_motor = None #Moteur avant droit
    

     # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 50.0           # P constant of the roll PID.
    K_PITCH_P = 30.0          # P constant of the pitch PID.

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    # Precision between the target position and the robot position in meters
    target_precision = 0.5

    def __init__(self, robot:Robot):
        # Définitions des variables moteurs en objets Motor
        self.rear_left_motor:Motor = robot.getDevice('rear left propeller')
        self.rear_right_motor:Motor = robot.getDevice('rear right propeller')

        self.front_left_motor:Motor = robot.getDevice('front left propeller')
        self.front_right_motor:Motor = robot.getDevice('front right propeller')

        # Définition des axes des moteurs pour tourner à l'infini
        motors = [self.rear_left_motor, self.rear_right_motor, self.front_left_motor, self.front_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)
        
        self.sensors = Sensors(robot)

    ### Méthode permettant aux moteurs d'effectuer un décollage    
    def take_off(self):
        self.target_altitude = 5 # Altitude à atteindre

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        # Lire les capteurs
        roll, pitch, yaw = self.sensors.imu.getRollPitchYaw()
        x_pos, y_pos, altitude = self.sensors.gps.getValues()
        roll_acceleration, pitch_acceleration, _ = self.sensors.gyro.getValues()

        roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
        pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
        vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

        front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
        front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
        rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
        rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

        #Front
        self.front_left_motor.setVelocity(front_left_motor_input)
        self.front_right_motor.setVelocity(-front_right_motor_input)
        # Rear 
        self.rear_left_motor.setVelocity(-rear_left_motor_input)
        self.rear_right_motor.setVelocity(rear_right_motor_input)
        
# Classe d'interface pour les capteurs (Centrale inertielle, GPS, Gyroscope)
class Sensors():
    imu = None # Capteur 'inertial unit' pour obtenir les données de Yaw , Pitch et Roll
    gps = None # Capteur GPS 
    gyro = None # Capteur Gyroscope

    ### Constructeur par défaut
    def __init__(self, robot:Robot):
        self.time_step = int(robot.getBasicTimeStep())

        # Initialisation de la centrale inertielle
        self.imu:InertialUnit = robot.getDevice('inertial unit')
        self.imu.enable(self.time_step)

        # Initialisation du GPS
        self.gps:GPS = robot.getDevice('gps')
        self.gps.enable(self.time_step)

        # Initialisation du gyroscope
        self.gyro:Gyro = robot.getDevice('gyro')
        self.gyro.enable(self.time_step)

# Classe du robot Mavic
class Mavic(Robot):

    ### Constructeur par défaut
    def __init__(self):
        super().__init__()
        self.motors = Motors(self)
    
    ### Méthode permettant au drone de décoller
    def take_off(self):
        self.motors.take_off()


# create the Robot instance.
robot = Mavic()

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


    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    robot.take_off()
    pass

# Enter here exit cleanup code.
