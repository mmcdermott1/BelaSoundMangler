# Bela Sound Mangler Design Documentation

### *Multi-Threading For FFT Processing*

This program relies on the concept of multi-threading for FFT processing.  The function ` process_fft` is computationally expensive so when using FFT tehcniques it is always best to place them on a separate thread.  Here, `process_fft` is called once each time gHopSize is filled and runs in the background separate from the main 'render' function.  This allows for more efficient computing, but does introduce more latency.  For this project, the added latency is not an issue and the total latency for the program comes out to equal the sum of gHopSize and gFftSize.

            if (++gHopCounter >= gHopSize)
            {
                gHopCounter = 0;
                gCachedInputBufferPointer = gInputBufferPointer;
                Bela_scheduleAuxiliaryTask(gFftTask);
            }

### *Multi-Threading For Write Function*

I also use a separate thread for writing recorded samples to disk because writing to disk is not generally considered "real-time safe" and running it within the `render` function will result in dropped blocks.

I use an AND logic gate to determine that the user has switched back to **Play Mode** `gPlayRecord == 0` and has just finished making a recording `gRecorded == 1`.  When this criteria is met, `gWriteTask` is called and the recorded material is written to disk.

    else if (gPlayRecord == 0 && gRecorded == 1)
    {
        gRecorded = 0;
        Bela_scheduleAuxiliaryTask(gWriteTask);
    }

`gRecorded` is a variable used to determine when a new recording has been made and is essential in determining that the user has stopped recording and tells the program to write to disk using the auxiliary task.

### *GUI Design*

Ideally, all parameters should be controllable with several tangible devices (Trill Sensors, Pots, etc).  For this project, however, I needed to use basic Bela Gui Controllers to give the user more control over the program until I can incorprate more physical elements.

#### *Play or Record*

The user can switch between `Play Mode` and `Record Mode`.  These sliders are powered by simple if statements that determine whether a value `gPlayRecord` is equal to 0 or 1.

    //	PLAYBACK MODE (Set via GUI)
    // -----------------
    if (gPlayRecord == 0)
    {
        [OMMITED - FFT PLAYBACK CODE LIVES HERE]
    }

    //	RECORD MODE (Set via GUI)
    // -----------------
    if (gPlayRecord == 1)
    {
        record(context, userData);
        gRecorded = 1;
    }

Within the `Play Mode` if statement, I also included an else statement that allows for real-time monitoring of input as the user records sound.  So even when the user is in `Record Mode` they are still getting audio output.

        else
        {
            for (unsigned int channel = 0; channel < context->audioOutChannels; channel++) // monitor what is being recording
            {
                float out = audioRead(context, n, channel) * gVolume;
                audioWrite(context, n, channel, out);
            }
        }

#### *Pitch or Time*

The user can select the function of the Trill Square sensor.  Again, using a simple if statement, the specific type of processing applied within `process_fft` is chosen by the status of `gTrillMode`.

    if (gRoboPitch >= 1 && gRoboFrequency >= 20.0)
    {
        [OMMITTED - ROBOTISE FUNCTION]
    }

    else if (gRoboPitch >= 1 && gRoboFrequency < 20.0)
    {
        [OMMITTED - WHISPERISE FUNCTION]
    }

    else if (gRoboPitch < 1)
    {
        [OMMITTED - PITCH SCALE FUNCTION]
    }

A "hidden" feature is revealed here.  To incorporate another FFT technique, I tucked it into an existing function.  The range of `gRoboFrequency` is set from 10.0 to 880.0, but when the user sets the value below 20.0 (via the Trill Sensor), the program is no longer robotising.  When `gRoboFrequency < 20.0` the FFT process goes into **Whisperisation** mode.

#### *Hop Size & FFT Size*

I wanted the user to be able to manipulate the lengths of the Hop Size and FFT Window Size in real time to explore their effects, fidelities, and qualities, and use them to shape the sound.

The user can choose a Hop Size between 128 and 1024, and the slider increments in values of 128.  Ideally, the choices should be exponential (ex. 128, 256, 512, 1024) but I did not have the time to explore how to make the slider values this way so I settled on setting the increment value equal to the minimum value for FFT Size and Hop Size to have a more coarse value.

The user can choose an FFT Size between 256 and 2048, incremented in values of 256.

Each time a change is made to the Hop Size, however, new windows need to be calculated for use in process_fft.  An if statement determines whether the slider Hop Size `gNewHopSize` is equal to or different from the previously set `gHopSize`.  New windows are recalculated when these values are unequal.

    if (gNewHopSize != gHopSize)
    {
        int newLength = gNewHopSize * 4;
        if (newLength > gFftSize)
            newLength = gFftSize;
        recalculate_window(newLength);
        gHopSize = gNewHopSize;
    }

Now would also be a good time to mention the output scaling.  Each time these values are altered, the resulting amplitude of the output will change because we are changing the number of overlapping samples.  More overlapping samples means increased amplitude.  To compensate for this, `gScaleFactor` is updated in the `process_fft` function as a ratio equal gHopSize divided by gFftSize.  This helps to maintain a consistent output volume while switching FFT Size and Hop Size.

	gScaleFactor = (float)gHopSize / (float)gFftSize;

I do believe, however, there is a better way to track this.  Occasionally, the value is not updated properly and I do experience inconsistencies in the output level while switching these values. 

#### *Volume*

The user can control the master output amplitude.  This is a basic scaling value between 0.0 and 1.0 which is multiplied by the various outputs in our code.

    out *= gScaleFactor * gVolume;
//

    float out = audioRead(context, n, channel) * gVolume;

### *Trill Design*

The Trill Square Sensor is at the heart of this project and is the main source for manipulating its parameters.

With only a single sensor, I need to maximize its potential.  I did this by using the Y-Axis position to determine the function of the X-Axis. 

You do lose something, though, when you do not have direct access to all of these functions at once.  To compensate for this shortcoming, I used Touch Pressure to incorporate an additional parameter for each.  So even in `Pitch Mode` the use can manipulate time by pressing harder to change the loop length.  And in `Time Mode` Touch Pressure adds the abilitiy to reverse the playback direction.

In `Time Mode`, currently the user can only slow down the playback speed.  Attempts to speed up playback result in segmnetation fault.

Loop length is calculated as a fraction gSampleBuffer.size() over gLength to give values ranging from the whole clip to 1/32 of the clip.

#### *Pitch Mode*

- **Y-Axis** position determines the function of the **X-Axis**
- Touching the **upper half** of the sensor will place the program in `Robotisation Mode` and convert all incoming frequencies to a single robotic pitch set by **X-Axis** position
    - Pitch increases as you move from **left to right** with a base frequency range of 10 Hz to 880 Hz (although values below 20 Hz are never realized due to feature listed next)
    - Extreme far left **X-Axis** position (which means `gRoboFrequency < 20.0`) will place the program in `Whisperisation Mode` randomizing the phase of all incoming samples
- Touching the **lower half** of the sensor will place the program in `Pitch Scaling Mode` which will allow the user to manipulate the pitch of independent of time by the **X-Axis** position
- **Touch Pressure** will change loop length of the recorded material
    - The **harder** you press, the shorter the length of the loop

#### *Time Mode*

- **Y-Axis** position determines the function of the **X-Axis**
- Touching the **upper half** of the sensor will change the playback speed of recorded material based on **X-Axis** position (Note: FFT Timescaling is not yet supported so this will also introduce a change in pitch)
    - Recording slows down as you move from **left to right**
- Touching the **lower half** of the sensor will change loop length of the recorded material based on **X-Axis** position
    - Loop shortens as you move from **left to right**
- **Touch Pressure** controls playback direction
    - A **harder** press will reverse the recorded material

#### *Remembering X-Y Position*

Although I am certain it is possible, I was unable to figure out how to set the Trill Sensor to latch onto the last recorded X-Y position and keep its effects even when the user stops touching the device.

As a work around, I implemented an if statement that seems to work to remember the X-Y values when the user stops touching the sensor.

    if (gTouchSize == 0.0)
    {
        gRoboPitch;
        gPitchShiftSemitones;
        gRoboFrequency;
        gSpeed;
        gLength;
        gReverse;
        gLengthSpeed;
    }

This method is a bit clunky and causes the Bela IDE to deliver errors of `expression result unused` but for the most part it solves my problem.

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

### Bela Resources
Visit [https://bela.io/] to explore products, educational resources, and other projects created with Bela.
