import numpy as np
from Search import search
from Plot import plot_map


class GridWorld:

    #====================================
    #       Inizializzazione
    #====================================
    #
    def __init__(self):
        self.map = np.array([[0, 1, 0], [0, -1, 0], [0, 0, 0]])
        self.startCell = [1, 1]
        self.position = [1, 1]
        self.front = [2, 1]
        self.right = [1, 0]
        self.left = [1, 2]
        self.direction = 2


    
    #====================================
    #       Aggiornamento
    #====================================
    
    # Aggiornamento mappa
    def updateMap(self):
        (row, col) = self.map.shape
    
        if self.position[0] == 1: 
            self.map = np.vstack((np.zeros((1,col)) , self.map)) 
            self.position[0] += 1 
            self.startCell[0] += 1
            self.front[0] += 1
            self.right[0] += 1
            self.left[0] += 1
            row += 1 
    
        elif self.position[0] == row - 2: 
            self.map = np.vstack((self.map , np.zeros((1,col)))) 
            row += 1 
    
        if self.position[1] == 1: 
            self.map = np.hstack((np.zeros((row,1)) , self.map)) 
            self.position[1] += 1 
            self.startCell[1] += 1 
            self.front[1] += 1
            self.right[1] += 1
            self.left[1] += 1
            col += 1 
    
        elif self.position[1] == col - 2: 
            self.map = np.hstack((self.map , np.zeros((row,1)))) 
            col += 1


    # Aggiunta ostacoli
    def addObstacle(self, position):
        if position == "front":
            self.map[self.front[0], self.front[1]] = 1

        elif position == "right":
            self.map[self.right[0], self.right[1]] = 1

        else:
            self.map[self.left[0], self.left[1]] = 1
        

    # Attuazione mossa
    def doAction(self, action):
        if action == "left":
            self.direction = (self.direction - 1) % 4
            self.right = self.front
            self.front = self.left
            dX = self.right[0] - self.position[0]
            dY = self.right[1] - self.position[1]
            self.left = [self.position[0] - dX, self.position[1] - dY]

        elif action == "right":
            self.direction = (self.direction + 1) % 4
            self.left = self.front
            self.front = self.right
            dX = self.left[0] - self.position[0]
            dY = self.left[1] - self.position[1]
            self.right = [self.position[0] - dX, self.position[1] - dY]

        else:
            dX = ((self.direction+1)%2*(self.direction-1))
            dY = ((self.direction)%2*(2-self.direction))
            self.front = [self.front[0] + dX , self.front[1] + dY]
            self.left = [self.left[0] + dX , self.left[1] + dY]
            self.right = [self.right[0] + dX , self.right[1] + dY]
            self.position = [self.position[0] + dX , self.position[1] + dY]

        self.map[self.position[0], self.position[1]] = -1

    
    #====================================
    #            Ricerca
    #====================================

    def cells2movements(self, path):
        index = len(path) - 1
        tDir = self.direction
        newPath = []
        newDirection = [path[index-1][0] - path[index][0], path[index-1][1] - path[index][1]]
        if newDirection == [-1, 0]: newDirection = 0
        elif newDirection == [0, 1]: newDirection = 1
        elif newDirection == [1, 0]: newDirection = 2
        else: newDirection = 3
        if abs(newDirection-self.direction) == 2: 

            newPath.append("right")
            tDir = (self.direction + 1) % 4

        while index != 0:
            newDirection = [path[index-1][0] - path[index][0], path[index-1][1] - path[index][1]]

            if newDirection == [-1, 0]: newDirection = 0
            elif newDirection == [0, 1]: newDirection = 1
            elif newDirection == [1, 0]: newDirection = 2
            else: newDirection = 3

            if tDir == newDirection:
                newPath.append("front")

            elif (newDirection - tDir) % 4 == 1:
                newPath.append("right")
                newPath.append("front")
                tDir = (tDir + 1) % 4
            
            else:
                newPath.append("left")
                newPath.append("front")
                tDir = (tDir - 1) % 4

            index -= 1
        
        return newPath


    def searchPath(self, goal):
        path = search(self.position, self.direction, goal, self.map)
        if path:
            path = self.cells2movements(path)
        return path
         

    def goToBase(self):
        path = search(self.position, self.direction, self.startCell, self.map)
        return self.cells2movements(path)


    #====================================
    #             Utility
    #====================================

    def plotMap(self):
        plot_map(self.map)

    def getFrontValue(self):
        return self.map[self.front[0], self.front[1]]

    def getRightValue(self):
        return self.map[self.right[0], self.right[1]]

    def getLeftValue(self):
        return self.map[self.left[0], self.left[1]]

    def getStartCell(self):
        return self.startCell

    def getPosition(self):
        return self.position

    def getDirection(self):
        return self.direction
