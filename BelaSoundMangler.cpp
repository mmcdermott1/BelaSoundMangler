/*

*/
#include <Bela.h>
#include <libraries/Fft/Fft.h>
#include <libraries/AudioFile/AudioFile.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <libraries/Trill/Trill.h>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

//	RECORD + PLAYBACK DECLARATIONS
//============================================================================================================
std::vector<std::vector<float> > gInputs;
const double gDurationSec = 30;   // maximum length to record -- used to allocate memory
unsigned int gWrittenFrames = 0;  // keep track of frames written to gInputs
std::vector<float> gSampleBuffer; // holds the sound file
int gReadPointer = 0;             // position of the last frame we played
int gPlayRecord = 0;              // switch to decide whether to play or make new recording
int gReverse = 0;                 // switch to reverse playback
int gRecorded = 0;                //																	???
int gLengthSpeed = 0;             // if 0, X-Axis controls gLength -- if 1, X-axis controls gSpeed
float gSpeed = 1.0;               // set to value between 0.1 and 1.0 to avoid segfault
int gLength = 1;                  // shorten playback file length, fractionally
float gVolume = 0.8;

//	PROCESS FFT DECLARATIONS
//============================================================================================================
Fft gFft;                         // declare FFT processing object
int gFftSize = 2048;              // FFT window size in samples
int gHopSize = 256;               // How often we calculate a window of samples
int gNewHopSize;                  //
float gScaleFactor = 0.125;       // how much to scale the output based on window type + overlap
float gPitchShift = 1.0;          // ratio of output to input frequency
float gPitchShiftSemitones = 0.0; // semitone control of pitch shift
float gRoboFrequency = 110;       // fundamental frequency of the robot effect
float gAudioSampleRate = 44100;   // sample rate
int gRoboPitch = 0;               // if 0, X-Axis controls gRoboFrequency -- if 1, X-axis controls gPitchShiftSemitones

std::vector<float> gInputBuffer; // circular buffer for assembling a window of samples
int gInputBufferPointer = 0;     // pointer to sample in gInputBuffer
const int gBufferSize = 44100;   // arbitrary large number for maximum amount of samples needed
int gHopCounter = 0;             // how many hops have processed

std::vector<float> gOutputBuffer;                        // Circular buffer for collecting the output of samples from the overlap-add process
int gOutputBufferWritePointer = gFftSize + 2 * gHopSize; // Start the write pointer ahead of the read pointer by at least window + hop, with some margin
int gOutputBufferReadPointer = 0;                        //												???

std::vector<float> gAnalysisWindowBuffer;             // Buffer to hold the windows for FFT analysis
std::vector<float> gSynthesisWindowBuffer;            // Buffer to hold the windows for FFT synthesis
std::vector<float> gBinFrequencies(gFftSize / 2 + 1); //									???

std::string gFilename = "inputs.wav"; // Name of the sound file (in project folder)

// THREAD DECLARATIONS
//============================================================================================================
AuxiliaryTask gFftTask;
int gCachedInputBufferPointer = 0;
void process_fft_background(void *);

AuxiliaryTask gWriteTask;
void write_background(void *);

//	GUI DECLARATIONS
//============================================================================================================
Gui gGui;
GuiController gGuiController;

//	TRILL DECLARATIONS
//============================================================================================================
Trill touchSensor;                   // declare Trill object
unsigned int gTaskSleepTime = 12000; // Sleep time for auxiliary task

float gTouchPosition[2] = {0.0, 0.0};               // declare X and Y axis points
float gTouchSize = 0.0;                             // declare touch pressure value
int gTrillMode = 0;                                 // if 0, Trill will pitch morph || if 1, Trill will time mangle
int gRoboPitchRange[2] = {0, 2};                    // Range for Mode mapping (Robot vs. Pitch) -- Y AXIS
int gLengthSpeedRange[2] = {0, 2};                  // Range for Mode mapping (Robot vs. Pitch) -- Y AXIS
float gPitchShiftSemitonesRange[2] = {-24.0, 24.0}; //Range for PitchShift mapping -- X AXIS UPPER HALF
float gRoboFrequencyRange[2] = {10.0, 880.0};       // Range for Robotisation mapping -- X AXIS LOWER HALF
float gLengthRange[2] = {1.0, 32.0};                // Range for gLength mapping -- X AXIS LOWER HALF
float gSpeedRange[2] = {1.0, 0.1};                  // Range for Speed mappingg
int gReverseRange[2] = {0, 2};                      // Range for Reverse mapping

//	TRILL FUNCTION PROTOTYPE
//============================================================================================================
void trill(void *)
{
    while (!Bela_stopRequested())
    {
        touchSensor.readI2C(); // read locations from Trill sensor
        gTouchSize = touchSensor.compoundTouchSize();
        gTouchPosition[0] = touchSensor.compoundTouchHorizontalLocation();
        gTouchPosition[1] = touchSensor.compoundTouchLocation();
        usleep(gTaskSleepTime);
    }
}

//	BELA SETUP FUNCTION
//============================================================================================================
bool setup(BelaContext *context, void *userData)
{

    //	RECORD + PLAYBACK SETUP
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    gSampleBuffer = AudioFileUtilities::loadMono(gFilename); // load file from project folder and check for error
    if (gSampleBuffer.size() == 0)
    {
        rt_printf("Error loading audio file '%s'\n", gFilename.c_str());
        return false;
    }

    rt_printf("Loaded the audio file '%s' with %d frames (%.1f seconds)\n", // print audio file info
              gFilename.c_str(), gSampleBuffer.size(),
              gSampleBuffer.size() / context->audioSampleRate);

    unsigned int numFrames = context->audioSampleRate * gDurationSec; // allocate memory needed to store recorded audio data
    gInputs.resize(context->audioInChannels);

    try // too many channels or too many frames = fail to start -- lack of RAM
    {
        for (auto &c : gInputs)
            c.resize(numFrames);
    }
    catch (std::exception e)
    {
        fprintf(stderr, "Error while allocating memory. Maybe you are asking to record too many frames and/or too many channels\n");
        return false;
    }

    //	PROCESS FFT SETUP
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    gFft.setup(gFftSize); // Set up the FFT and its buffers
    gInputBuffer.resize(gBufferSize);
    gOutputBuffer.resize(gBufferSize);
    gAudioSampleRate = context->audioSampleRate; // Cache the sample rate for use in process_fft()

    gAnalysisWindowBuffer.resize(gFftSize); // Calculate Hanning windows
    gSynthesisWindowBuffer.resize(gFftSize);
    for (int n = 0; n < gFftSize; n++)
    {
        gAnalysisWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gFftSize - 1)));
        gSynthesisWindowBuffer[n] = gAnalysisWindowBuffer[n];
    }

    //	THREAD SETUP
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    gWriteTask = Bela_createAuxiliaryTask(write_background, 70, "bela-process-write");
    gFftTask = Bela_createAuxiliaryTask(process_fft_background, 50, "bela-process-fft");

    //	GUI SETUP
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    gGui.setup(context->projectName);
    gGuiController.setup(&gGui, "BelaSoundMangler Controller");

    gGuiController.addSlider("Play or Record", 0, 0, 1, 1); // name, def, min, max, inc
    gGuiController.addSlider("Pitch or Time", 0, 0, 1, 1);
    gGuiController.addSlider("Hop Size", 256, 128, 1024, 128);
    gGuiController.addSlider("FFT Size", 2048, 256, 2048, 256);
    gGuiController.addSlider("Volume", 0.8, 0, 1, 0.05);

    //	TRILL SETUP
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (touchSensor.setup(1, Trill::SQUARE) != 0)
    { // Setup a Trill Square on i2c bus 1, using the default mode and address
        fprintf(stderr, "Unable to initialise Trill Square\n");
        return false;
    }
    touchSensor.printDetails();
    Bela_runAuxiliaryTask(trill); // Set and schedule auxiliary task for reading sensor data from the I2C bus
    return true;
}

//	WRITE FUNCTION PROTOTYPE
//============================================================================================================
void write()
{
    for (auto &i : gInputs)
        i.resize(gWrittenFrames);
    AudioFileUtilities::write(gFilename, gInputs, gAudioSampleRate);
    gSampleBuffer.resize(gInputs[0].size());

    for (int n = 0; n < gInputs[0].size(); n++)
    {
        gSampleBuffer[n] = gInputs[0][n];
    }
    gWrittenFrames = 0;
}

//	WRITE CALLBACK FUNCTION
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void write_background(void *)
{
    write();
}

//	PLAY FUNCTION PROTOTYPE
//============================================================================================================
float play(BelaContext *context, void *userData)
{
    float out;

    //	NORMAL PLAYBACK MODE
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (gReverse == 0)
    {
        out = gSampleBuffer[gReadPointer * gSpeed];
        gReadPointer++;
        if (gReadPointer >= gSampleBuffer.size() / gLength)
            gReadPointer = 0;
    }

    //	REVERSE PLAYBACK MODE
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    else
    {
        out = gSampleBuffer[gReadPointer * gSpeed];
        gReadPointer--;
        if (gReadPointer <= 1)
            gReadPointer = gSampleBuffer.size() / gLength;
    }
    return out;
}

//	RECORD FUNCTION PROTOTYPE
//============================================================================================================
void record(BelaContext *context, void *userData)
{
    for (unsigned int n = 0; n < context->audioFrames; ++n)
    {
        for (unsigned int c = 0; c < context->audioInChannels; ++c)
            gInputs[c][gWrittenFrames] = audioRead(context, n, c); // store recorded audio samples to gInputs
        ++gWrittenFrames;
    }
}

//	RECALCULATE WINDOW FUNCTION PROTOTYPE
//============================================================================================================
void recalculate_window(unsigned int length)
{
    if (length > gAnalysisWindowBuffer.size()) // check for running off end of buffer
        length = gAnalysisWindowBuffer.size();
    if (length > gSynthesisWindowBuffer.size())
        length = gSynthesisWindowBuffer.size();
}

//	WRAP PHASE FUNCTION PROTOTYPE
//============================================================================================================
float wrapPhase(float phaseIn) // Wrap the phase to the range -pi to pi
{
    if (phaseIn >= 0)
        return fmodf(phaseIn + M_PI, 2.0 * M_PI) - M_PI;
    else
        return fmodf(phaseIn - M_PI, -2.0 * M_PI) + M_PI;
}

//	PROCESS FFT FUNCTION PROTOTYPE
//============================================================================================================
void process_fft(std::vector<float> const &inBuffer, unsigned int inPointer, std::vector<float> &outBuffer, unsigned int outPointer)
{
    //	FFT LOCAL CONTAINERS
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    static std::vector<float> unwrappedBuffer(gFftSize);  // hold the unwrapped time-domain values
    static std::vector<float> lastInputPhases(gFftSize);  // hold  phases from previous hop of input signal
    static std::vector<float> lastOutputPhases(gFftSize); // and output (synthesised) signal

    static std::vector<float> analysisMagnitudes(gFftSize / 2 + 1); // hold converted data from magnitude-phase to magnitude-frequency
    static std::vector<float> analysisFrequencies(gFftSize / 2 + 1);
    static std::vector<float> synthesisMagnitudes(gFftSize / 2 + 1);
    static std::vector<float> synthesisFrequencies(gFftSize / 2 + 1);
    static std::vector<int> synthesisCount(gFftSize / 2 + 1);

    gScaleFactor = (float)gHopSize / (float)gFftSize;

    //	CHANGE FFT SIZE AND HOP SIZE
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (gNewHopSize != gHopSize) // if hop size changes, recalculate window based on overlap factor of 4
    {
        int newLength = gNewHopSize * 4;
        if (newLength > gFftSize)
            newLength = gFftSize;
        recalculate_window(newLength);
        gHopSize = gNewHopSize;
    }

    //	RUN THE FFT
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    for (int n = 0; n < gFftSize; n++) // copy buffer into FFT input
    {
        int circularBufferIndex = (inPointer + n - gFftSize + gBufferSize) % gBufferSize; // use modulo arithmetic to calculate the circular buffer index
        unwrappedBuffer[n] = inBuffer[circularBufferIndex] * gAnalysisWindowBuffer[n];
    }

    gFft.fft(unwrappedBuffer); // Process the FFT based on the time domain input

    //	ROBOTISE
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (gRoboPitch >= 1 && gRoboFrequency >= 20.0)
    {
        //	ANALYSIS
        // ---------------------------------------------------------------------------------------------------
        for (int n = 0; n <= gFftSize / 2; n++) // analysis of only lower half of spectrum
        {
            float amplitude = gFft.fda(n); // Turn real and imaginary components into amplitude and phase
            float phase = atan2f(gFft.fdi(n), gFft.fdr(n));
            float phaseDiff = phase - lastInputPhases[n];                                            // phase difference in bin between the last hop and current hop = exact frequency
            phaseDiff = wrapPhase(phaseDiff - gBinFrequencies[n] * gHopSize);                        // subtract amt of phase inc based on bin center freq for given hop size and wrap range
            float frequencyDeviation = phaseDiff / (float)gHopSize;                                  // find deviation center frequency
            analysisFrequencies[n] = ((float)n * 2.0 * M_PI / (float)gFftSize) + frequencyDeviation; // add original bin number to get fractional bin where this partial belongs
            analysisMagnitudes[n] = amplitude;                                                       // save magnitude for later
            lastInputPhases[n] = phase;                                                              // save phase for next hop
        }

        //	CLEAR SYNTHESIS BINS FOR NEW DATA
        // ---------------------------------------------------------------------------------------------------
        for (int n = 0; n <= gFftSize / 2; n++)
        {
            synthesisMagnitudes[n] = synthesisFrequencies[n] = synthesisCount[n] = 0;
        }

        //	PERFORM ROBOTISATION IN FREQUENCY DOMAIN
        // ---------------------------------------------------------------------------------------------------
        for (int n = 0; n <= gFftSize / 2; n++) // Handle the robotisation effect, storing frequencies into new bins
        {
            float harmonicRaw = analysisFrequencies[n] * gAudioSampleRate / (2.0 * M_PI) / gRoboFrequency; // calculate closest harmonic to this frequency
            float harmonicBelow = floorf(harmonicRaw);                                                     // by dividing by the fundamental frequency and rounding
            float harmonicAbove = harmonicBelow + 1;
            float harmonicFraction = harmonicRaw - harmonicBelow; // Round the frequency to the nearest multiple of the fundamental

            if (harmonicBelow > 0) // only resynth if freq greater than 0
            {
                float newFrequency = gRoboFrequency * (2.0 * M_PI / gAudioSampleRate) * harmonicBelow;
                int newBin = floorf(newFrequency * (float)gFftSize / (2.0 * M_PI) + 0.5); // find nearest bin to shifted frequency

                if (newBin <= gFftSize / 2) // ignore new bins that have shifted above nyquist
                {
                    synthesisMagnitudes[newBin] += analysisMagnitudes[n] * (1.0 - harmonicFraction);
                    synthesisFrequencies[newBin] = newFrequency;
                }
            }

            float newFrequency = gRoboFrequency * (2.0 * M_PI / gAudioSampleRate) * harmonicAbove;
            int newBin = floorf(newFrequency * (float)gFftSize / (2.0 * M_PI) + 0.5);

            if (newBin <= gFftSize / 2)
            {
                synthesisMagnitudes[newBin] += analysisMagnitudes[n] * (1.0 - harmonicFraction);
                synthesisFrequencies[newBin] = newFrequency;
            }
        }

        //	SYNTHESIS
        // ---------------------------------------------------------------------------------------------------
        for (int n = 0; n <= gFftSize / 2; n++) // Synthesise frequencies into new magnitude and phase values for FFT bins
        {
            float frequencyDeviation = synthesisFrequencies[n] - ((float)n * 2.0 * M_PI / (float)gFftSize); // Get the fractional offset from the bin centre frequency
            float phaseDiff = frequencyDeviation * (float)gHopSize;                                         // Multiply to get back to a phase value
            phaseDiff += gBinFrequencies[n] * gHopSize;                                                     // Add the expected phase increment based on the bin centre frequency
            float outPhase = wrapPhase(lastOutputPhases[n] + phaseDiff);                                    // Advance the phase from the previous hop
            gFft.fdr(n) = synthesisMagnitudes[n] * cosf_neon(outPhase);                                     // Now convert magnitude and phase back to real and imaginary components
            gFft.fdi(n) = synthesisMagnitudes[n] * sinf_neon(outPhase);

            if (n > 0 && n < gFftSize / 2) // Also store the complex conjugate in the upper half of the spectrum
            {
                gFft.fdr(gFftSize - n) = gFft.fdr(n);
                gFft.fdi(gFftSize - n) = -gFft.fdi(n);
            }
            lastOutputPhases[n] = outPhase; // Save the phase for the next hop
        }
    }

    //	WHISPERISE
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    else if (gRoboPitch >= 1 && gRoboFrequency < 20.0)
    {
        for (int n = 0; n < gFftSize; n++)
        {
            float amplitude = gFft.fda(n);
            float phase = 2.0 * M_PI * (float)rand() / (float)RAND_MAX; // randomize phase for each sample sample
            gFft.fdr(n) = amplitude * cosf_neon(phase);
            gFft.fdi(n) = amplitude * sinf_neon(phase);
        }
    }

    //	PITCH SCALE
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    else if (gRoboPitch < 1)
    {

        //	ANALYSIS
        // ---------------------------------------------------------------------------------------------------
        for (int n = 0; n <= gFftSize / 2; n++) // analysis of only lower half of spectrum
        {
            float amplitude = gFft.fda(n); // Turn real and imaginary components into amplitude and phase
            float phase = atan2f(gFft.fdi(n), gFft.fdr(n));
            float phaseDiff = phase - lastInputPhases[n]; // phase difference in bin between the last hop and current hop = exact frequency
            float binCentreFrequency = 2.0 * M_PI * (float)n / (float)gFftSize;
            phaseDiff = wrapPhase(phaseDiff - binCentreFrequency * gHopSize);
            float binDeviation = phaseDiff * (float)gFftSize / (float)gHopSize / (2.0 * M_PI); // find deviation in (fractional) number of bins from the centre frequency
            analysisFrequencies[n] = (float)n + binDeviation;                                  // add original bin number to get fractional bin where this partial belongs
            analysisMagnitudes[n] = amplitude;                                                 // save magnitude for later
            lastInputPhases[n] = phase;                                                        // save phase for next hop
        }

        //	CLEAR SYNTHESIS BINS FOR NEW DATA
        // ---------------------------------------------------------------------------------------------------
        for (int n = 0; n <= gFftSize / 2; n++)
        {
            synthesisMagnitudes[n] = synthesisFrequencies[n] = 0;
        }

        //	PERORM PITCH SCALING IN FREQUENCY DOMAIN
        // ---------------------------------------------------------------------------------------------------
        for (int n = 0; n <= gFftSize / 2; n++)
        {
            int newBin = floorf(n * gPitchShift + 0.5); // find the nearest bin to the shifted frequency
            if (newBin <= gFftSize / 2)
            { // Ignore any bins that have shifted above Nyquist
                synthesisMagnitudes[newBin] += analysisMagnitudes[n];
                synthesisFrequencies[newBin] = analysisFrequencies[n] * gPitchShift; // scale the frequency by the pitch shift ratio
            }
        }

        //	SYNTHESIS
        // ---------------------------------------------------------------------------------------------------
        for (int n = 0; n <= gFftSize / 2; n++)
        {
            float amplitude = synthesisMagnitudes[n];

            float binDeviation = synthesisFrequencies[n] - n;                                // Get the fractional offset from the bin centre frequency
            float phaseDiff = binDeviation * 2.0 * M_PI * (float)gHopSize / (float)gFftSize; //  Multiply to get back to a phase value
            float binCentreFrequency = 2.0 * M_PI * (float)n / (float)gFftSize;              //  Add the expected phase increment based on the bin centre frequency
            phaseDiff += binCentreFrequency * gHopSize;
            float outPhase = wrapPhase(lastOutputPhases[n] + phaseDiff); //  Advance the phase from the previous hop
            gFft.fdr(n) = amplitude * cosf(outPhase);                    //  Now convert magnitude and phase back to real and imaginary components
            gFft.fdi(n) = amplitude * sinf(outPhase);

            if (n > 0 && n < gFftSize / 2)
            { // Also store the complex conjugate in the upper half of the spectrum
                gFft.fdr(gFftSize - n) = gFft.fdr(n);
                gFft.fdi(gFftSize - n) = -gFft.fdi(n);
            }
            lastOutputPhases[n] = outPhase; //  Save the phase for the next hop
        }
    }

    //	RUN THE INVERSE FFT
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    gFft.ifft();
    for (int n = 0; n < gFftSize; n++) // Add timeDomainOut into the output buffer
    {
        int circularBufferIndex = (outPointer + n - gFftSize + gBufferSize) % gBufferSize;
        outBuffer[circularBufferIndex] += gFft.td(n) * gSynthesisWindowBuffer[n];
    }
}

//	PROCESS FFT CALLBACK FUNCTION
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void process_fft_background(void *)
{
    for (int n = 0; n <= gFftSize / 2; n++) // Precompute the bin frequencies
    {
        gBinFrequencies[n] = 2.0 * M_PI * (float)n / (float)gFftSize;
    }

    process_fft(gInputBuffer, gCachedInputBufferPointer, gOutputBuffer, gOutputBufferWritePointer);
    gOutputBufferWritePointer = (gOutputBufferWritePointer + gHopSize) % gBufferSize; // Update the output buffer write pointer to start at the next hop
}

//	MAIN FUNCTION
//============================================================================================================
void render(BelaContext *context, void *userData)
{
    // RECIEVE GUI VALUES FROM SLIDERS
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    gPlayRecord = gGuiController.getSliderValue(0);
    gTrillMode = gGuiController.getSliderValue(1);
    gNewHopSize = (int)gGuiController.getSliderValue(2);
    gFftSize = (int)gGuiController.getSliderValue(3);
    gVolume = gGuiController.getSliderValue(4);
    gPitchShift = powf(2.0, gPitchShiftSemitones / 12.0); // convert semitone values to frequency values

    //	REMEMBER LAST X-Y POSITION
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

    //	MAP PARAMETERS TO TRILL & CHOOSE MODE
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    else
    {
        //	TRILL PITCH MORPH MODE (Set via GUI)
        // ---------------------------------------------------------------------------------------------------
        if (gTrillMode == 0)
        {
            gRoboPitch = map(gTouchPosition[1], 0, 1, gRoboPitchRange[0], gRoboPitchRange[1]);                               // Map Y-Axis to RoboPitch (Select X-axis function)
            gPitchShiftSemitones = map(gTouchPosition[0], 0, 1, gPitchShiftSemitonesRange[0], gPitchShiftSemitonesRange[1]); // Map X-Axis to gPitchShiftSemitones and gRoboFrequency
            gRoboFrequency = map(gTouchPosition[0], 0, 1, gRoboFrequencyRange[0], gRoboFrequencyRange[1]);
            gLength = map(gTouchSize, 0, 1, gLengthRange[0], gLengthRange[1]); // Map Pressure to Length
        }

        //	TRILL TIME MANGLE MODE (Set via GUI)
        // ---------------------------------------------------------------------------------------------------
        else
        {
            gLengthSpeed = map(gTouchPosition[1], 0, 1, gLengthSpeedRange[0], gLengthSpeedRange[1]); // Map Y-Axis to LengthSpeed (Select X-axis function)

            if (gLengthSpeed >= 1) //Map X-Axis to either gLength or gSpeed depending on gLengthSpeed
            {
                gSpeed = map(gTouchPosition[0], 0, 1, gSpeedRange[0], gSpeedRange[1]);
            }
            else
            {
                gLength = map(gTouchPosition[0], 0, 1, gLengthRange[0], gLengthRange[1]);
            }

            gReverse = map(gTouchSize, 0, 1, gReverseRange[0], gReverseRange[1]); // Map Pressure to Reverse
        }
    }

    //	CALL RECORD, PLAYBACK + WRITE FUNCTIONS
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    for (unsigned int n = 0; n < context->audioFrames; n++)
    {
        //	PLAYBACK MODE (Set via GUI)
        // ---------------------------------------------------------------------------------------------------
        if (gPlayRecord == 0)
        {
            float in = play(context, userData);       // read the next sample from the buffer
            gInputBuffer[gInputBufferPointer++] = in; // store the sample ("in") in a buffer for the FFT

            if (gInputBufferPointer >= gBufferSize) // Wrap the circular buffer
            {
                gInputBufferPointer = 0;
            }

            float out = gOutputBuffer[gOutputBufferReadPointer]; // Get the output sample from the output buffer
            gOutputBuffer[gOutputBufferReadPointer] = 0;         // Then clear the output sample in the buffer so it is ready for the next overlap-add
            out *= gScaleFactor * gVolume;                       // Scale the output down by the scale factor, compensating for the overlap
            gOutputBufferReadPointer++;                          // Increment the read pointer in the output cicular buffer

            if (gOutputBufferReadPointer >= gBufferSize)
                gOutputBufferReadPointer = 0;

            if (++gHopCounter >= gHopSize) // Increment the hop counter and start a new FFT if we've reached the hop size
            {
                gHopCounter = 0;
                gCachedInputBufferPointer = gInputBufferPointer;
                Bela_scheduleAuxiliaryTask(gFftTask);
            }

            for (unsigned int channel = 0; channel < context->audioOutChannels; channel++) // write the audio to the output
            {
                audioWrite(context, n, channel, out);
            }
        }

        else
        {
            for (unsigned int channel = 0; channel < context->audioOutChannels; channel++) // monitor what is being recording
            {
                float out = audioRead(context, n, channel) * gVolume;
                audioWrite(context, n, channel, out);
            }
        }
    }

    //	RECORD MODE (Set via GUI)
    // ---------------------------------------------------------------------------------------------------
    if (gPlayRecord == 1)
    {
        record(context, userData);
        gRecorded = 1;
    }

    // WRITE FILE TO DISK AFTER SWITCHING FROM RECORD MODE TO PLAYBACK MODE
    // ---------------------------------------------------------------------------------------------------
    else if (gPlayRecord == 0 && gRecorded == 1)
    {
        gRecorded = 0;
        Bela_scheduleAuxiliaryTask(gWriteTask);
    }
}

void cleanup(BelaContext *context, void *userData)
{
}