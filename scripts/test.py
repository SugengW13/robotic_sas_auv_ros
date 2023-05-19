import cv2

# Create a VideoCapture object to read from the camera
cap = cv2.VideoCapture(1)  # Use 0 to indicate the default camera (you can change the index if you have multiple cameras)

# Check if the camera was successfully opened
if not cap.isOpened():
    print("Failed to open the camera")
    exit()

# Loop to continuously read frames from the camera
while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # Check if the frame was successfully read
    if not ret:
        print("Failed to read frame from the camera")
        break

    # Display the frame
    cv2.imshow("Camera", frame)

    # Wait for 'q' key to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close the window
cap.release()
cv2.destroyAllWindows()
