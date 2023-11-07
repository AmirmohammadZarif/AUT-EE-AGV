import cv2
import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.error_integral = 0
        self.previous_error = 0

    def calculate(self, error):
        # Proportional term
        P = self.Kp * error

        # Integral term
        self.error_integral += error
        I = self.Ki * self.error_integral

        # Derivative term
        D = self.Kd * (error - self.previous_error)
        self.previous_error = error

        # Total control signal
        control_signal = P + I + D

        return control_signal

cap = cv2.VideoCapture(0)  

# Define color range
lower_color = np.array([66, 84, 0])  # Lower color threshold (blue in this example)
upper_color = np.array([132, 255, 255])  # Upper color threshold

pid_controller = PIDController(Kp=1, Ki=0, Kd=0)

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
            control_signal = pid_controller.calculate(error)

            text = f"Error{error}"
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            text_control = f"Control Signal{control_signal}"
            cv2.putText(frame, text_control, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    
            print(text, text_control)
            

    cv2.imshow("Perception", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()