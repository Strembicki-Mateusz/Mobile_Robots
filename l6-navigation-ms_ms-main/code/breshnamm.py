def bresenham(x1, y1, x2, y2):
    # Calculate differences
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    
    # Calculate the steps for both directions
    if x1 < x2:
        sx = 1
    else:
        sx = -1
        
    if y1 < y2:
        sy = 1
    else:
        sy = -1

    # Initialize error value
    err = dx - dy

    # Result list to store points of the line
    points = []

    while True:
        # Append the current point (x, y)
        points.append((x1, y1))
        
        # If we reach the destination, break the loop
        if x1 == x2 and y1 == y2:
            break
        
        # Calculate error for X and Y directions
        e2 = 2 * err
        
        # Update x and y based on error
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

    return points

# Function to handle all 4 directions
def draw_line(x1, y1, x2, y2):
    if x1 == x2:
        # Vertical line
        if y1 < y2:
            return bresenham(x1, y1, x2, y2)
        else:
            return bresenham(x2, y2, x1, y1)

    elif y1 == y2:
        # Horizontal line
        if x1 < x2:
            return bresenham(x1, y1, x2, y2)
        else:
            return bresenham(x2, y2, x1, y1)

    elif abs(x2 - x1) == abs(y2 - y1):
        # Diagonal line (positive slope)
        if x1 < x2:
            return bresenham(x1, y1, x2, y2)
        else:
            return bresenham(x2, y2, x1, y1)

    else:
        # Diagonal line (negative slope)
        if x1 < x2:
            return bresenham(x1, y1, x2, y2)
        else:
            return bresenham(x2, y2, x1, y1)
