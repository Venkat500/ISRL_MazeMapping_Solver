def calculate_direction(path):
    direction=[]
    print("Path Received: ", path)
    for i in range(0,len(path)-1):
        print("=========================================")
        print("Calculating Direction ")
        print("From Path 1: ", path[i])
        print("From Path 2: ", path[i+1])
        print("=========================================")
        if(path[i][0] < path[i+1][0]):
            print("Go Down")
            direction.append("down")
        elif (path[i][0] > path[i+1][0]):
            print("Go UP")
            direction.append("up")
        elif (path[i][1] < path[i+1][1]):
            print("Go left")
            direction.append("left")
        elif (path[i][1] > path[i+1][1]):
            print("Go right")
            direction.append("right")
    print("=========================================")
    print("Calculated Direction :", direction)
    print("=========================================")

