import cv2
import numpy as np

# Callback function for trackbar changes
def on_trackbar_change(_):
    pass

def main():
    # Load the image
    image = cv2.imread("test.png")

    # Create a window for trackbars
    cv2.namedWindow("Trackbars")

    # Initialize trackbar values
    h_min, h_max = 0, 179
    s_min, s_max = 0, 255
    v_min, v_max = 0, 255

    # Create trackbars
    cv2.createTrackbar("Hue Min", "Trackbars", h_min, 179, on_trackbar_change)
    cv2.createTrackbar("Hue Max", "Trackbars", h_max, 179, on_trackbar_change)
    cv2.createTrackbar("Saturation Min", "Trackbars", s_min, 255, on_trackbar_change)
    cv2.createTrackbar("Saturation Max", "Trackbars", s_max, 255, on_trackbar_change)
    cv2.createTrackbar("Value Min", "Trackbars", v_min, 255, on_trackbar_change)
    cv2.createTrackbar("Value Max", "Trackbars", v_max, 255, on_trackbar_change)

    while True:
        # Get current trackbar values
        h_min = cv2.getTrackbarPos("Hue Min", "Trackbars")
        h_max = cv2.getTrackbarPos("Hue Max", "Trackbars")
        s_min = cv2.getTrackbarPos("Saturation Min", "Trackbars")
        s_max = cv2.getTrackbarPos("Saturation Max", "Trackbars")
        v_min = cv2.getTrackbarPos("Value Min", "Trackbars")
        v_max = cv2.getTrackbarPos("Value Max", "Trackbars")

        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create lower and upper bounds for color range
        lower_color = np.array([h_min, s_min, v_min])
        upper_color = np.array([h_max, s_max, v_max])

        # Apply color filter
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Apply the mask to the original image
        result = cv2.bitwise_and(image, image, mask=mask)

        # Display the original image and the result
        cv2.imshow("Original Image", image)
        cv2.imshow("Result", result)

        # Exit loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Close windows
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()