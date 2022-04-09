import numpy as np
import math as mt
import sys

from SLAM import SLAM
from GridWorld import GridWorld
from controller import Robot


# =============================================
#          Inizializzazione mondo
#==============================================

robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000

# Motori
motorLeft = robot.getDevice('left wheel motor')
motorRight = robot.getDevice('right wheel motor')
motorLeft.setPosition(float("inf"))
motorRight.setPosition(float("inf"))
motorLeft.setVelocity(0)
motorRight.setVelocity(0)

# Sensori
tof = []
for i in range(1, 4): tof.append(robot.getDevice('tof'+str(i)))
for ds in tof: ds.enable(timestep)

# Camera
camera = robot.getDevice("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)



# Gestione dei plot
# 'no'        -> No plot
# 'map'       -> Mappa
# 'slam cell' -> Slam per cella
# 'slam cont' -> Slam continuo

try:
    plot = sys.argv[1]

    if plot == 'no':
        print('Nessun grafico verrà mostrato')
    elif plot == 'map':
        print('Viene mostrato il grafico della mappa')
    elif plot == 'slam cell':
        print('Viene mostrato il grafico dello slam con aggiornamento per ogni cella')
    elif plot == 'slam cont':
        print('Viene mostrato il grafico dello slam con aggiornamento continuo')
    else:
        raise Exception('Argomento controller non riconosciuto: non verrà mostrato nessun grafico')
    
except Exception as exc: 
    if type(exc) is IndexError:
        print('Nessun argomento in input: non verrà mostrato nessun grafico')
    elif type(exc) is Exception:
        print(exc)

    plot = 'no'



# =============================================
#               Variabili
#==============================================

# Misure Ambiente
cellSize = 0.35
bodyRadius = 0.1675
maxSpeed = motorLeft.getMaxVelocity() 
turnSpeed = 3
linearVelocity = 0.191
angularVelocity = 0.625

# Dati utili
distanceBetweenWheels = 2 * bodyRadius 
rateOfRotation = 2 * (angularVelocity * bodyRadius) / distanceBetweenWheels
angleOfRotation = np.pi/2
durationRotation = angleOfRotation / rateOfRotation + dt
durationSide = cellSize / linearVelocity 

# Dizionario mosse
# 
moves = {
    "right": {
        "motorLeft": turnSpeed,
        "motorRight": -turnSpeed,
        "linearVelocity": 0,
        "angularVelocity": -angularVelocity,
        "duration": durationRotation
    },
    "left": {
        "motorLeft": -turnSpeed,
        "motorRight": turnSpeed,
        "linearVelocity": 0,
        "angularVelocity": angularVelocity,
        "duration": durationRotation
    },
    "front": {
        "motorLeft": maxSpeed,
        "motorRight": maxSpeed,
        "linearVelocity": linearVelocity,
        "angularVelocity": 0,
        "duration": durationSide
    }
}

# Rappresentazioni
slam = SLAM()
gridWorld = GridWorld()



# =============================================
#           Metodi principali
#==============================================

# Controllo dello stato attuale
def controlState():
    global gridWorld
    gridWorld.updateMap()
    measureObstacles()

    [x, y] = slam.getMu()[0:2, 0] 
    startCell = gridWorld.getStartCell()
    position = gridWorld.getPosition()
    xTarget = (position[0]-startCell[0]) * cellSize 
    yTarget = (position[1]-startCell[1]) * cellSize 
    dist_x = xTarget - x
    dist_y = yTarget - y
    distance = np.sqrt(dist_x**2 + dist_y**2)

    if distance > cellSize/4: 
        theta = mt.atan2(dist_y, dist_x)
        correctPosition(distance, theta)
        measureObstacles()

    if plot == 'map':
        gridWorld.plotMap()

# Decisione sulla mossa da effettuare
def elaborateAction():
    if gridWorld.getRightValue() == 0:
        return "right"

    if gridWorld.getFrontValue() == 0 or (gridWorld.getFrontValue() == -1 and gridWorld.getLeftValue() == 1):
        return "front"

    if gridWorld.getRightValue() == gridWorld.getFrontValue() == gridWorld.getLeftValue() == -1:
        return "path"

    else:
        return "left"

# Attuazione della mossa
def doAction(action):
    global gridWorld

    if action == "path":
        path = gridWorld.searchPath(0)
        if path: 
            followPath(path)
            return True
        else:
            path = gridWorld.goToBase()
            followPath(path)
            return False

    makeMove(action)
    gridWorld.doAction(action)
    return True



# =============================================
#           Metodi secondari
#==============================================

# Movimento
def makeMove(action, duration = 0, mapping = True):
    global slam

    if plot == 'slam cell':
        slam.plotSlam()

    param = moves[action]

    if duration == 0:
        duration = param["duration"]
    
    startTime = robot.getTime()

    while robot.step(timestep) != -1:
        currentTime = robot.getTime()
        if currentTime < startTime + duration:
            motorLeft.setVelocity(param["motorLeft"])
            motorRight.setVelocity(param["motorRight"])

            spaceDone = (currentTime - startTime) * param["linearVelocity"]
            if duration == 0: spaceLeft = (cellSize - spaceDone) * 1000
            else: spaceLeft = (duration*param["linearVelocity"] - spaceDone) * 1000

            if param["linearVelocity"] != 0 and tof[0].getValue() < spaceLeft*0.8:
                motorLeft.setVelocity(0)
                motorRight.setVelocity(0)
                duration = startTime + duration - currentTime

                while robot.step(timestep) != -1:

                    if tof[0].getValue() > spaceLeft + cellSize*100:
                        robot.step(2000)
                        break

                    if not mapping and robot.getTime() - currentTime > 10:
                        return False
                    
                startTime = robot.getTime()

            slam.predictionMove([param["linearVelocity"], param["angularVelocity"], dt])
            
            if camera.getRecognitionNumberOfObjects() != 0:
                measurement = measureLandmark()
                slam.controlLandmark(measurement)

            if plot == 'slam cont':
                slam.plotSlam()

        else:
            motorLeft.setVelocity(0)
            motorRight.setVelocity(0)
            break

    return True

# Movimento precalcolato
def followPath(path):
    global gridWorld

    for action in path:
        success = makeMove(action, mapping = False)

        if not success: break
        
        gridWorld.doAction(action)
        controlState()
        
    if not success:
        doAction("path")


# Correzione della posizione
def correctPosition(distance, theta):
    incInit = slam.getMu()[2, 0]

    angleRotation = theta - slam.getMu()[2, 0]
    angleRotation = adjustAngle(angleRotation)

    durationRotation = getDurationRotation(abs(angleRotation))
    durationSide = getDurationSide(abs(distance))

    if angleRotation > 0: action = "left"
    else: action = "right"

    makeMove(action, durationRotation)
    makeMove("front", durationSide)

    correctAngle = nearestAngle(incInit)

    returnAngle = correctAngle - slam.getMu()[2, 0]
    returnAngle = adjustAngle(returnAngle)
    durationRotation = getDurationRotation(abs(returnAngle))

    if action == "left": action = "right"
    elif action == "right": action = "left"

    makeMove(action, durationRotation)


# Rilevamneto ostacoli
def measureObstacles():
    global gridWorld

    if tof[0].getValue() < cellSize*900: gridWorld.addObstacle("front")
    if tof[1].getValue() < cellSize*900: gridWorld.addObstacle("right")
    if tof[2].getValue() < cellSize*900: gridWorld.addObstacle("left")


# Misurazione dei landmark
def measureLandmark():
    measures = []
    landmarks = camera.getRecognitionObjects()
    for landmark in landmarks:
        position = landmark.get_position()
        position[0] += 0.172
        measures.append(np.vstack(( [mt.sqrt(position[0]**2 + position[1]**2)] , [mt.atan2(position[1], position[0])] )))

    return measures



# =============================================
#                 Utility
#==============================================

def getDurationRotation(angleOfRotation):
    return angleOfRotation / rateOfRotation + dt

def getDurationSide(distance):
    return distance / linearVelocity 

def nearestAngle(angle):
    angles = [0, 1.57, 3.14]
    distances = []
    
    for el in angles:
        if angle >= 0:
            distances.append(abs(angle - el))
        else:
            distances.append(abs(angle + el))

    if angle >= 0:
        return angles[distances.index(min(distances))]
    else:
        return -angles[distances.index(min(distances))]

def adjustAngle(angle):
    while angle >  np.pi: angle -= 2*np.pi
    while angle < -np.pi: angle += 2*np.pi
    return angle



# =============================================
#               MAIN
#==============================================

def main():
    while robot.step(timestep) != -1:            
        controlState()
        action = elaborateAction()
        next = doAction( action )

        if not next: break

    motorLeft.setVelocity(0)
    motorRight.setVelocity(0)


if __name__ == '__main__':
    main()