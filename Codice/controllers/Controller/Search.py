from MapSearch import Map


# Restituisce i successori validi
def getNextNodes(node, dim, dir, map):
    nextNodes = [[node[0]-1, node[1], 0], [node[0], node[1]+1, 1], [node[0]+1, node[1], 2], [node[0], node[1]-1, 3]]

    for i in range(3, -1, -1):
        nextNode = nextNodes[i]
        
        if nextNode[0] < 0 or nextNode[1] < 0 or nextNode[0] >= dim[0] or nextNode[1] >= dim[1] or map[nextNode[0]][nextNode[1]] == 1: 
            nextNodes.pop(i)
            continue
        
        if dir == nextNode[2]:
            nextNodes[i].append(1)
        elif dir == (nextNode[2]+1)%4 or dir == (nextNode[2]-1)%4:
            nextNodes[i].append(2)
        else:
            nextNodes[i].append(4)

    return nextNodes

# Metodo della ricerca
def search(position, direction, goal, map):
    dim = map.shape
    map_search = Map(dim)
    map_search.root(position, direction)
    open = [position]

    while open:
        node = open.pop(0)

        if (type(goal) is list and node == goal) or (type(goal) is int and map[node[0], node[1]] == goal):
            return map_search.getPath(node)

        iNextNodes = getNextNodes(node, dim, map_search.getDirection(node), map)   

        for iNextNode in iNextNodes:
            nextNode = iNextNode[0:2]
            direction = iNextNode[2]
            cost = iNextNode[3]

            if map_search.isEmpty(nextNode):
                map_search.addChild(node, nextNode)
                map_search.setNode(nextNode, node, cost, direction)
                open.append(nextNode)

            else:
                if map_search.getCost(nextNode) > map_search.getCost(node) + cost:
                    map_search.addChild(node, nextNode)
                    map_search.updateParentAndChildren(nextNode, node)
        
        open.sort(key = lambda node: map_search.getCost(node))
    
    return None