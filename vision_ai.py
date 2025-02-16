import openai
import os
import http.server
import socketserver
import threading
import requests
import serial
import time
import cv2

__version__=1.0

# Configuration
DIRECTORY = "C:\\Pictures" # Desired folder
PORT = 8000  # Server Ethernet port
COM_PORT = 7 #Serial COM port
BAUDRATE = 115200 #Serial COM Baudrate

def capture_photo():
    # Open the default camera (camera ID 0)
    camera = cv2.VideoCapture(0)

    if not camera.isOpened():
        raise Exception("Could not open camera!")

    # Read a frame from the camera
    ret, frame = camera.read()

    if not ret:
        camera.release()
        raise Exception("Failed to capture image!")

    # Generate a unique file name based on the current date and time
    folder_path=DIRECTORY
    file_name=time.strftime("photo_%Y%m%d_%H%M%S.jpeg")
    file_name_path = os.path.join(folder_path, file_name )

    # Save the captured frame as an image
    cv2.imwrite(file_name_path, frame)

    # Release the camera
    camera.release()

    # Return the file name
    return file_name, file_name_path


class HTTPFileServer:
    def __init__(self, directory, port=8000):
        self.directory = directory
        self.port = port
        self.server = None
        self.link = None

    def start_server(self):
        """Starts a simple HTTP server to share files from the specified directory."""
        if not os.path.exists(self.directory):
            raise FileNotFoundError(f"Directory '{self.directory}' does not exist.")

        os.chdir(self.directory)  # Change to the desired directory

        # Create a simple HTTP handler
        handler = http.server.SimpleHTTPRequestHandler
        self.server = socketserver.TCPServer(("", self.port), handler)

        # Get local IP address
        public_ip = requests.get('https://api.ipify.org').text

        # Construct the link
        self.link = f"http://{public_ip}:{self.port}/"

        # Start server in a separate thread
        thread = threading.Thread(target=self.server.serve_forever)
        thread.daemon = True  # Stops server when the program exits
        thread.start()

        print(f"Serving HTTP on {public_ip}:{self.port}")
        print(f"Access files locally: http://localhost:{self.port}/")
        print(f"Access files on your network: {self.link}")

    def stop_server(self):
        """Stops the HTTP server."""
        if self.server:
            print("Stopping server...")
            self.server.shutdown()
            print("Server stopped.")

class OpenAIHandler:
    def __init__(self):
        _api_key_ =""
        openai.api_key = _api_key_

    def send_request(self, message):
        """Sends a message to OpenAI and returns the response."""
        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {
                    "role": "user",
                    "content": message
                }
            ],
            max_tokens=300,
        )
        return response.choices[0]

if __name__ == "__main__":
    
    # Configurate of serial comm port
    comm=serial.Serial(port=f"COM{COM_PORT}", baudrate=BAUDRATE, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    
    """
    # Print Serial connection
    try:
        try:
            while 1:
                print(comm.readline().decode("utf8"))
        except Exception as e:
            print(f"Error reading from serial port: {e}")
    except KeyboardInterrupt:
        print("Exiting communication due to keyboard interrupt.")
    """
    #get an iamge and length
    try:
        file_name,file_full_name = capture_photo()
        depth=comm.readline().decode("utf8")
        print(f"Photo saved as: {file_full_name} and depth is {depth}")
    except Exception as e:
        print(f"Error: {e}")

    try:
        # Start the HTTP file server
        file_server = HTTPFileServer(DIRECTORY, PORT)
        file_server.start_server()

        # Construct the image URL
        image_url = f"{file_server.link}{file_name}"
        print(image_url)

        # Prepare the message for OpenAI
        message_content = [
            {"type": "text", "text": "if the length from camera to page is" f"{depth}" "camrea horizontal and vertiacal angles are 106.26 degres please estimate the size of square perimeter? please return final answer without calculations as number you assum"},
            {
              "type": "image_url", "image_url": { "url": f"{image_url}", },
            },
          ]   
    
        # Interact with OpenAI
        openai_handler = OpenAIHandler()
        response = openai_handler.send_request(message_content)

        # Display OpenAI response
        print("OpenAI Response:", response)

    except Exception as e:
        print("Error:", e)   
    finally:
        # Stop the server
        if 'file_server' in locals():
            file_server.stop_server()
