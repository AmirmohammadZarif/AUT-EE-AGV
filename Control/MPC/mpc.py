import cv2
import numpy as np
from scipy.optimize import minimize

# Initialize the camera
cap = cv2.VideoCapture(0)
# Check if the camera is opened successfully
if not cap.isOpened():
    print("Failed to open the camera")
    exit()
# Define the line detection parameters
low_threshold = 50
high_threshold = 150
rho = 1
theta = np.pi/180
threshold = 50
min_line_length = 10
max_line_gap = 50

# Define the image size and line properties
image_width = 800
image_height = 600
line_length = 300
line_thickness = 5

# Define the initial position and orientation of the robot
x = image_width // 2
y = image_height // 2
theta = 0

# Define the MPC control parameters
N = 10  # Prediction horizon
dt = 0.1  # Time step

# Define the objective function for optimization
def objective_function(u):
    x_pred = x
    y_pred = y
    theta_pred = theta

    for i in range(N):
        v, w = u[2*i:2*i+2]
        x_pred += v * np.cos(theta_pred) * dt
        y_pred += v * np.sin(theta_pred) * dt
        theta_pred += w * dt

    return np.sqrt((x_pred - line_length) ** 2 + y_pred ** 2)

# Main loop
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Check if the frame is empty
    if not ret:
        print("Failed to capture frame")
        break
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Canny edge detection
    edges = cv2.Canny(gray, low_threshold, high_threshold)
    edges = edges.astype(np.uint8)
    edges = cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)
    # Perform Hough line detection
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), minLineLength=min_line_length, maxLineGap=max_line_gap)
    # # Draw the detected lines on the frame
    # if lines is not None:
    #     for line in lines:
    #         x1, y1, x2, y2 = line[0]
    #         cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

    #         # Calculate desired orientation based on the detected line
    #         desired_theta = np.arctan2(y2 - y1, x2 - x1)

    #         # Optimize the control sequence using MPC
    #         u0 = np.zeros(2 * N)
    #         res = minimize(objective_function, u0, bounds=[(-2, 2), (-2, 2)] * N)
    #         u_opt = res.x.reshape((-1, 2))

    #         # Apply the first control input from the optimized sequence
    #         v_opt, w_opt = u_opt[0]
    #         x += v_opt * np.cos(theta) * dt
    #         y += v_opt * np.sin(theta) * dt
    #         theta += w_opt * dt

    # # Draw the robot position
    # robot_radius = 10
    # robot_end_x = int(x + robot_radius * np.cos(theta))
    # robot_end_y = int(y + robot_radius * np.sin(theta))
    # cv2.circle(frame, (int(x), int(y)), robot_radius, (0, 0, 255), -1)
    # cv2.line(frame, (int(x), int(y)), (robot_end_x, robot_end_y), (0, 0, 255), 2)

    # # Display the resulting frame
    cv2.imshow('Line Tracking', edges)

    # # Check for key press
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()