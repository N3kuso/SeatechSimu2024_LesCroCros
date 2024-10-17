"""monde3_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, InertialUnit, GPS, Gyro
import numpy as np

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

    ### Constructeur par défaut
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
        
        # Définitions de quelques variables pour des positions
        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0

        self.sensors = Sensors(robot)

        self.t1 = robot.getTime()

    ### Méthode permettant de définir la postion du robot 
    def set_position(self, pos):
        """
        Set the new absolute position of the robot
        Parameters:
            pos (list): [X,Y,Z,yaw,pitch,roll] current absolute position and angles
        """
        self.current_pose = pos
        print(self.current_pose)
    
    def move_to_target(self, waypoints, verbose_movement=False, verbose_target=False):
        """
        Move the robot to the given coordinates
        Parameters:
            waypoints (list): list of X,Y coordinates
            verbose_movement (bool): whether to print remaning angle and distance or not
            verbose_target (bool): whether to print targets or not
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
        """

        if self.target_position[0:2] == [0, 0]:  # Initialization
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):

            self.target_index += 1
            if self.target_index > len(waypoints) - 1:
                self.target_index = 0
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                print("Target reached! New target: ",
                      self.target_position[0:2])

        # This will be in ]-pi;pi]
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1], self.target_position[0] - self.current_pose[0])
        # This is now in ]-2pi;2pi[
        angle_left = self.target_position[2] - self.current_pose[5]
        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if (angle_left > np.pi):
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        # non proportional and decreasing function
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + (
                (self.target_position[1] - self.current_pose[1]) ** 2))
            print("remaning angle: {:.4f}, remaning distance: {:.4f}".format(
                angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance

    ### Méthode permettant aux moteurs d'effectuer un décollage    
    def take_off(self, robot:Robot):

        # Specify the patrol coordinates
        waypoints = [[0,0]]
        self.target_altitude = 5 # Altitude à atteindre

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        # Lire les capteurs
        roll, pitch, yaw = self.sensors.imu.getRollPitchYaw()
        x_pos, y_pos, altitude = self.sensors.gps.getValues()
        roll_acceleration, pitch_acceleration, _ = self.sensors.gyro.getValues()
        # Set la position du robot
        self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

        if altitude > self.target_altitude - 1:
            # as soon as it reach the target altitude, compute the disturbances to go to the given waypoints.
            print(f"Robot : {robot.getTime()} || t1 : {self.t1}")
            if robot.getTime() - self.t1 > 0.1:
                print("UwU")
                yaw_disturbance, pitch_disturbance = self.move_to_target(
                    waypoints,verbose_movement=True,verbose_target=True)
                self.t1 = robot.getTime()

        # Calcule les inputs de Roll, Pitch, Yaw
        roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
        pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
        vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

        # Calcule la vitesse pour les différents moteurs
        front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
        front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
        rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
        rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

        # Affectation de la vitesse calculée aux moteurs avants
        self.front_left_motor.setVelocity(front_left_motor_input)
        self.front_right_motor.setVelocity(-front_right_motor_input)
        # Affectation de la vitesse calculée aux moteurs arrières
        self.rear_left_motor.setVelocity(-rear_left_motor_input)
        self.rear_right_motor.setVelocity(rear_right_motor_input)
        
# Classe d'interface pour les capteurs (Centrale inertielle, GPS, Gyroscope)
class Sensors():
    imu = None # Capteur 'inertial unit' pour obtenir les données de Yaw , Pitch et Roll
    gps = None # Capteur GPS 
    gyro = None # Capteur Gyroscopenp

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
        self.motors.take_off(self)


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
