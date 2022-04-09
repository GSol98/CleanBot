#
# ELEMENTO MAPPA:
#
# - Posizione padre
# - Posizione dei figli
# - Costo totale
# - Direzione 
#

class Map:

    # Creazione mappa
    def __init__(self, dim):
        self.map = [[[] for j in range(dim[1])] for i in range(dim[0])]

    # Inizializzazione radice
    def root(self, position, direction):
        self.map[position[0]][position[1]] = [None, [], 0, direction]

    # Lettura nodo
    def getNode(self, node):
        return self.map[node[0]][node[1]]

    # Definizione nodo
    def setNode(self, node, parent, cost, direction):
        parentCost = self.getCost(parent)
        self.map[node[0]][node[1]] = [parent, [], parentCost+cost, direction]

    # Controllo cella
    def isEmpty(self, node):
        return self.getNode(node) == []

    # Lettura padre
    def getParent(self, node):
        return self.map[node[0]][node[1]][0]

    # Definizione padre
    def setParent(self, node, parent):
        self.map[node[0]][node[1]][0] = parent

    # Lettura nodi figli
    def getChildren(self, node):
        return self.map[node[0]][node[1]][1]

    # Aggiunta nodo figlio
    def addChild(self, node, child):
        self.map[node[0]][node[1]][1].append([child[0], child[1]])

    # Rimozione nodo figlio
    def removeChild(self, node, child):
        self.map[node[0]][node[1]][1].remove([child[0], child[1]])

    # Lettura costo totale nodo
    def getCost(self, node):
        return self.map[node[0]][node[1]][2]

    # Definizione costo totale nodo
    def setCost(self, node, cost):
        self.map[node[0]][node[1]][2] = cost

    # Lettura direzione
    def getDirection(self, node):
        return self.map[node[0]][node[1]][3]

    # Definizione direzione
    def setDirection(self, node, direction):
        self.map[node[0]][node[1]][3] = direction

    # Cambio puntatore al padre
    def updateParentAndChildren(self, node, newParent):
        oldParent = self.getParent(node)
        self.removeChild(oldParent, node)
        self.setParent(node, newParent)
        newParentCost = self.getCost(newParent)
        directionParent = self.getDirection(newParent)
        move = [node[0] - newParent[0], node[1] - newParent[1]]

        if move == [-1, 0]: newDirection = 0
        if move == [0, 1]: newDirection = 1
        if move == [1, 0]: newDirection = 2
        else: newDirection = 3

        if directionParent == newDirection: cost = 1
        else: cost = 2

        self.setCost(node, newParentCost+cost)
        self.setDirection(node, newDirection)
        children = self.getChildren(node)
        for child in children:
            self.updateParentAndChildren(child, node)

    # Cammino dal nodo di partenza
    def getPath(self, node):
        path = [node]

        while self.getParent(node) != None:
            node = self.getParent(node)
            path.append(node)

        return path