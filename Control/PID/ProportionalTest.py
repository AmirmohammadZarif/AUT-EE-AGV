def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    
    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)
    
    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)



while True:
    # theta > 0 right, theta < 0 left
    error = int((input()))
    print(error)
    if(error >= 0): 
        print("v_l", translate(error, 0, 90, 20, 50))
        print("v_r", translate(error, 0, 90, 20, -50))

    if(error <= 0): 
        print("v_l", translate(error, 0, -90, 20, -50))
        print("v_r", translate(error, 0, -90, 20, 50))
    
