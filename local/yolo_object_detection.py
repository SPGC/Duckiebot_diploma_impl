# Networking
from flask import Flask, request
import io
# ML
import torch
from PIL import Image


app = Flask(__name__)

model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt', device='mps')

@app.route('/detect_objects', methods=['POST'])
def detect_objects():
    # Extract the incoming image
    try:
        # Read the image
        img = request.files['image'].read()
        img = Image.open(io.BytesIO(img))
        
        # Run the model
        pred = model(img)
        
        # Show results
        pred.print() # profiling
        pred.show() # display
        print(pred.xywh) # print
        
    except Exception as e:
        print(e)
        return "Could not process the image", 400


if __name__ == '__main__':
    app.run(debug=True)