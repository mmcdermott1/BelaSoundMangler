# Bela Sound Mangler User Manual

### *What Is Bela Sound Mangler?*

Bela Sound Mangler is a simple & intuitive, quiant & chaotic tool for sound exploration.  

Record anything (the sound of your voice, the radio, your keyboard, your iPhone) and use the power of touch to morph, mangle, and manipulate your sound in a variety of ways.  

Pitch shift it, robotise it, reverse it, slow it down and chop it all at the swipe of a finger.

### *Program Requirements*
- Bela Starter Kit -or- Bela Mini Starter Kit
- Computer with USB 3.0 port
- (2) Molex Style Audio Connectors
- Trill Square Sensor
- Audio Input Source [3.5mm connection]
    - Microphone, Synthesizer, iPhone, etc.
    - Anything that can connect to a 3.5mm audio port
- Audio Monitoring Source [3.5mm connection]
    - Headphones or Speakers

### *Configuring Your Bela System*
1. Connect your Trill Square Sensor your Bela System's I2C Pins
    - For additional help setting up your sensor visit: [Get Started With Trill](https://learn.bela.io/products/trill/get-started-with-trill/)
2. Connect your Molex Style Audio Connectors to your Bela System's Stereo Audio Input and Stereo Audio Output Pins
3. Connect the USB cable (included in your Bela or Bela Mini Starter Kit) to your computer
4. In a web browser (Google Chrome or Safari reccommended), launch the [Bela IDE](bela.local/) by typing `bela.local` into the URL search bar
5. Navigate to `Project Settings` by clicking on the gear-shaped icon in the Bela Tabs (far right vertical sidebar)
6. Make sure your settings match the ones listed below:

`Block Size (audioframes): 16`

`Analog Channels: 8`

`Analog Sample Rate (Hz): 22050`

`Digital Channels: 8`

`Headphone level (dB): -30`

`Use Analog: Off`

`Use Digital: Off`

`DAC level (dB): 0`

`PGA Gain left (dB): 10`

`PGA Gain right (dB): 10`

### *Testing Audio Playback*
1. Run the program by clicking the `Build & run` icon on the very left of the Bela Toolbar (horizontal toolbar)
2. After the program is built, you should hear audio output of the last recorded file
3. If playback volume is too loud or too soft, return to `Project Settings` and adjust the `Headphone level` to a comfortable level
    - You will need to stop the program by clicking the `Stop` icon on the Bela Toolbar and repeat *Step 1* to hear the change in level
    - You may need to do this a few times to get the level just right
4. If you are not hearing any audio playback:
    - Double check to make sure the `Project Settings` match the ones listed previously
    - Double check to ensure a strong connection between your Audio Input and Audio Output Sources and the Molex Style Audio Connectors
    - Double check to ensure your Audio Input device is in fact sending an audio signal, possibly by monitoring it via another output device (headphones, amplifier, etc.)
    - Test the cables your are using to connect your devices on other gear not currently used in this system to rule out the possibility of a faulty cable

### *Testing Audio Recording Levels*
1. Run the program by clicking the `Build & run` icon on the very left of the Bela Toolbar (horizontal toolbar)
2. After the program is built, you should hear audio output of the last recorded file
3. Open the GUI by clicking the `Launch Gui` icon on the Bela Toolbar
4. Drag the `Play or Record` slider from left to right to begin a new recording
    - Recording is a destructive process and will overwrite any previously recorded material
    - To avoid program errors or crashes, the length of each recording should not exceed 30 seconds in length
5. Drag the `Play or Record` slider from right to left to playback the material you just recorded
    - During this step users may experience a `segmentation fault`, especially during the very first build & run
    - If this occurs, do not panic - simply stop the program and run it again and you will hear what you just recorded is still in tact
6. You should now hear the audio output of the last recorded file
7. If playback volume is too loud or too soft, return to `Project Settings` and adjust the `Headphone level` to a comfortable level
8. If your recorded material is either too loud/distorted or too soft or you hear an excessive amount of system noise (clicks, buzzes, static, etc.), the problem may be solved by adjusting the `PGA Gain left/right` levels
    - When making these adjustments, the left and right channels should always be equal
    - If your Audio Input Source has volume control, you may want to experiment with adjusting its level, as well, to find a good, clean, balanced overall level
9. If you are not hearing audio playback of your newly recorded material, revisit *Step 4* in the **Testing Audio Playback Levels** section of this user manual

### *Using The Bela Sound Mangler GUI Controller*
1. Run the program by clicking the `Build & run` icon on the very left of the Bela Toolbar (horizontal toolbar)
2. After the program is built, you should hear audio output of the last recorded file
3. Open the GUI by clicking the `Launch Gui` icon on the Bela Toolbar
4. Manipulate the parameters by clicking and dragging on the sliders
5. `Play or Record` switch between Play and Record Mode
    - Left Position = `Play Mode` Enabled (Default)
        - Plays back most recent recording
    - Right Position = `Record Mode` Enabled
        - Creates a new recording, overwriting previous material
        - Each recording should not exceed a length of 30 seconds
        - Sound recordings between 5 and 15 seconds are ideal for this program
6. `Pitch or Time` switch between functions of Trill Square Sensor
    - Left Position = `Pitch Mode` Enabled (Default)
    - Right Position = `Time Mode` Enabled
7. `Hop Size` determines the overlap size for FFT processing
    - To avoid program errors or crashes, this value should not exceed the value of `FFT Size`
8. `FFT Size` determines the window size for FFT processing
    - To avoid program errors or crashes, this value should always be greater than the value of `Hop Size`
    - WARNING: Excessive manipulation of `FFT Size` and `Hop Size` may result in unwanted latency, errors, or program crashes

### *Using The Bela Sound Mangler Trill Controller*
1. Run the program by clicking the `Build & run` icon on the very left of the Bela Toolbar (horizontal toolbar)
2. After the program is built, you should hear audio output of the last recorded file
3. Use your finger to explore the function of the touch sensor
4. Remove your finger to find the program "latches" and remembers your last touch point
5. `Pitch Mode` (Default)
    - **Y-Axis** position determines the function of the **X-Axis**
    - Touching the **upper half** of the sensor will place the program in `Robotisation Mode` and convert all incoming frequencies to a single robotic pitch set by **X-Axis** position
        - Pitch increases as you move from **left to right**
        - Extreme far left **X-Axis** position will place the program in `Whisperisation Mode` randomizing the phase of all incoming samples
    - Touching the **lower half** of the sensor will place the program in `Pitch Scaling Mode` which will allow the user to manipulate the pitch of independent of time by the **X-Axis** position
    - **Touch Pressure** will change loop length of the recorded material
        - The **harder** you press, the shorter the length of thhe loop
6. `Time Mode` (Set via GUI)
    - **Y-Axis** position determines the function of the **X-Axis**
    - Touching the **upper half** of the sensor will change the playback speed of recorded material based on **X-Axis** position (Note: FFT Timescaling is not yet supported so this will also introduce a change in pitch)
        - Recording slows down as you move from **left to right**
    - Touching the **lower half** of the sensor will change loop length of the recorded material based on **X-Axis** position
        - Loop shortens as you move from **left to right**
    - **Touch Pressure** controls playback direction
        - A **harder** press will reverse the recorded material

### *Plans For Future Improvements*
 - Incorporate more Trill Sensors to remove need for GUI
 - FFT Time Scaling
 - Automatically stop `Record Mode` and begin `Play Mode` when Maximum Record Time is realized
 - User ability to change the Maximum Fractional Division of Length of the audio file
 - User ability to change the Maximum Record Time
 - Support for Stereo Audio
 - Fix occasional bugs from changing `Hop Size` and `FFT Size`
 - Fix occasional segmentation fault which sometimes occurs when switching between `Play Mode` and `Record Mode`
 - Fix silence that is sometimes introduced after a number of new recordings have been made

### *Acknowledgement*
Thank you to Akito von Troyer and Gabriel Ball for their guidance and patient assistance in helping me problem solve through the creation of this program.

Thank you to Andrew McPherson and the Augmented Instruments Laboratory at Queen Mary University of London for their thorough [Tutorial Series](https://www.youtube.com/playlist?list=PLCrgFeG6pwQmdbB6l3ehC8oBBZbatVoz3) on Bela and Phase Vocoder Techniques.  The programs detailed in the Phase Vocoder mini-series within these tutorials were the source material for which I built my program around.

### *Bela Resources*
Visit [https://bela.io/] to explore products, educational resources, and other projects created with Bela.
