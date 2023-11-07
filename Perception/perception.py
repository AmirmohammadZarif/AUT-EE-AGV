import cv2
import numpy as np
import matplotlib.pyplot as plt

from simple_pid import PID
pid = PID(1, 0.5, 0.05, setpoint=0)


# Initialize lists to store the control signal and error over time
control_signal_history = []
error_history = []

# Create a figure and axis for the plot
plt.figure()
plt.xlabel('Time')
plt.ylabel('Value')
plt.legend()
cap = cv2.VideoCapture(0)  

# Define color range
lower_color = np.array([66, 84, 0])  # Lower color threshold (blue in this example)
upper_color = np.array([132, 255, 255])  # Upper color threshold


while True:
    # Read frame from camera
    ret, frame = cap.read()

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Apply color filter
    mask = cv2.inRange(hsv, lower_color, upper_color)
    cv2.imshow('mask', mask)

    # Perform morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the binary image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Calculate error from line position
    error = 0
    if len(contours) > 0:
        # Find the largest contour 
        largest_contour = max(contours, key=cv2.contourArea)

        # Find the centroid 
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Draw centroid on the frame
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

            # Calculate error 
            frame_center = frame.shape[1] // 2
            error = cx - frame_center


            # Calculate the control signal using the PID controller
            control_signal = pid(error)
                
                
            text = f"Error{error}"
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            text_control = f"Control Signal{control_signal}"
            cv2.putText(frame, text_control, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    
            print(text, text_control)

            # Append the control signal and error to the history lists
            control_signal_history.append(control_signal)
            error_history.append(error)

            # Plot the control signal and error over time
            plt.plot(control_signal_history, label='Control Signal',  color='blue')
            plt.plot(error_history, label='Error',  color='red')
            

            # Show the plot
            plt.pause(0.0001)
            plt.draw()

            

    cv2.imshow("Perception", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
