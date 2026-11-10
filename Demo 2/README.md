Demo 2: Why the LCD isn't working

What is happening:
The Pi uses the same two wires (SDA and SCL) to talk to both the LCD and the Arduino. When something is wrong on those wires, the LCD stops working too.
With only the Pi connected, the LCD works fine and the Pi sends its data fine. The moment the Arduino is wired in, the LCD stops working. So the problem starts when the Arduino joins the wires, not with the LCD or the Pi on their own.

The problem:
When the Arduino was off but still plugged into those wires, it pulled them down and blocked the LCD. Once we unplugged the Arduino, the Pi found the LCD right away, so the LCD itself is fine.
The script only tries to start the LCD once, right when it begins. If the Arduino isn't on yet at that moment, the LCD fails and stays off for the whole run.
When the LCD does start, writing text to it is slow. That slows down the main loop, so the Arduino doesn't get new data fast enough and the robot stops moving.

How to check it:
Turn on both the Pi and the Arduino, leave all the wires connected, and run i2cdetect -y 1.
If you see 08 and 20, the wiring is fine and the fixed code will solve it.
If you see 08 but not 20, check the Arduino wires on the LCD plate. They should go to SDA, SCL and GND, not one pin over.
If you see nothing, the Arduino wires are probably swapped (A4 is SDA, A5 is SCL), or the Arduino isn't powered.

Checking on the pi:
Disconnect all connections from the pi, and re-run the code. The LCD will light up. 

How to run the pi
1. Pi and computer its running on should be connected the same wifi (eduroam).
2. When ready, connect via screen-sharing
3. To acess the code:
  - Got to the yellow folder (Double click)
    - Demo 2 (Double click)
      - use detect_aruco.py (Double click)
4. To run the code
  - Go to the terminal (Black (>) icon should be next to the folder icon) (Double click)
    - Call the Folder
      - cd "Demo 2"  <- exactly like this
    - Call the file in the folder
      - python3 detect_aurco.py <- exactly like this
* Takes a while to get the camera on and the code to start running, so chill back.

How to copy and paste
!! Do not do command C when editing the file it will delete everything, copy and paste commands do not work on the pi!!

When copying from the Pi 
- Select all and hit (copy from remote)
When pasting to the Pi
- Select and area and hit the button on the top right (paste to remote)
