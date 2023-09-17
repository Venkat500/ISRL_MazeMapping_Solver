
def check_maze(matrix):
    rows = len(matrix)
    columns = len(matrix[0])
    
    for i in range(rows):
        for j in range(columns):
            if matrix[i][j] == 0:
                # Check top
                if i > 0 and matrix[i-1][j] == -1:
                    return True
                # Check bottom
                if i < rows-1 and matrix[i+1][j] == -1:
                    return True
                # Check left
                if j > 0 and matrix[i][j-1] == -1:
                    return True
                # Check right
                if j < columns-1 and matrix[i][j+1] == -1:
                    return True
    return False


# a = [[0,1,2],[3,4,5],[6,7,8]]
# print(check_maze(a))