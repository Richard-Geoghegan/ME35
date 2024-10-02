#Code to run on PC for Teachable Machine Joystick
from pyscript.js_modules import teach, ble_library
import time
from pyscript.js_modules import mqtt_library

mqtt1=mqtt_library.myClient

broker_address='broker.hivemq.com'
topic = "ME35-24/Richard"



async def run_model(URL2):
    s = teach.s
    s.URL2 = URL2
    await s.init()
    
def get_predictions(num_classes):
    predictions = []
    for i in range (0,num_classes):
        divElement = document.getElementById('class' + str(i))
        if divElement:
            divValue = divElement.innerHTML
            predictions.append(divValue)
    return predictions


import asyncio
await run_model("https://teachablemachine.withgoogle.com/models/0WUKuPv-F/") #Change to your model link


while True:
    predictions = get_predictions(3)
    
    print(predictions)
    class1=predictions[0]
    class2=predictions[1]
    class3=predictions[2]
    if class1[9]=="1":
        pass
    if class2[11]=="9":
        mqtt1.publish("ME35-24/Richard", "start")
    if class3[11]=="9":
        mqtt1.publish("ME35-24/Richard", "stop")
    await asyncio.sleep(2)
