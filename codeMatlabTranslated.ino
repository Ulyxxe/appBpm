/****************************************************************************
 * Best Implementation for Heart-Rate Detection on Tiva LaunchPad (Energia)
 * 
 * Author:   [Your Name]
 * Date:     [Date]
 *
 * Workflow:
 *  1) Acquire a block of samples from ADC.
 *  2) 4th-order Butterworth low-pass filter (single-pass).
 *  3) Moving average smoothing.
 *  4) Peak detection => BPM
 * 
 * NOTES:
 *  - Tweak SAMPLE_FREQ, SIZE_BUFF, and thresholds to match your setup.
 *  - For "filtfilt"-like zero-phase, see the optional second pass in iirFilter().
 ****************************************************************************/

// ======================== USER PARAMETERS =========================

// 1) Hardware configuration
#define SENSOR_PIN        PD_3         // or PD_3, etc., depending on your wiring
#define ADC_MAX_RES       4096       // 12-bit ADC => range [0..4095]
#define ADC_MAX_VOLT      3.3f       // Tiva LaunchPad: 3.3 V reference

// 2) Sampling
#define SAMPLE_FREQ       100        // in Hz
#define SAMPLE_PERIOD_MS  (1000 / SAMPLE_FREQ)
#define SIZE_BUFF         2000       // number of samples per batch

// 3) Filter Coefficients (from your MATLAB: 4th-order Butterworth)
static const float B[5] = {0.0004f, 0.0017f, 0.0025f, 0.0017f, 0.0004f};
static const float A[5] = {1.0000f, -3.1806f, 3.8612f, -2.1122f, 0.4383f};

// 4) Moving average window
#define MOV_AVG_WINDOW    50

// 5) Peak detection
//    - Threshold factor => fraction of max amplitude
//    - Min peak distance => ensures we don't detect 2 peaks in rapid succession.
//      For ~1 Hz heart rate, a value near 0.4-0.6s is a good start.
#define THRESHOLD_FACTOR  0.3f
#define MIN_PEAK_DIST_SEC 0.5f

// ================================================================

// Global buffers
float g_bufferRaw[SIZE_BUFF];
float g_bufferFilt[SIZE_BUFF];
float g_bufferSmooth[SIZE_BUFF];

// Function prototypes
void readSamples();
void iirFilter(const float *input, float *output, int length);
void movingAverage(const float *input, float *output, int length, int windowSize);
float computeHeartRate(const float *signal, int length, float fs);

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);
  pinMode(SENSOR_PIN, INPUT);

  Serial.println("=== Tiva LaunchPad Heart-Rate Detection ===");
  Serial.println("Sampling, filtering, smoothing, peak detection, BPM...");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop()
{
  // 1) Acquire raw samples into g_bufferRaw
  readSamples();

  // 2) Filter with 4th-order Butterworth (single-pass)
  //    If you want zero-phase, apply iirFilter in reverse as well (filtfilt).
  iirFilter(g_bufferRaw, g_bufferFilt, SIZE_BUFF);
  
  //   Optional "filtfilt" approach (commented out for simplicity):
  /*
  {
    float tempReverse[SIZE_BUFF];
    // Reverse
    for(int i = 0; i < SIZE_BUFF; i++) {
      tempReverse[i] = g_bufferFilt[SIZE_BUFF - 1 - i];
    }
    // Filter reversed
    iirFilter(tempReverse, tempReverse, SIZE_BUFF);
    // Reverse back
    for(int i = 0; i < SIZE_BUFF; i++) {
      g_bufferFilt[i] = tempReverse[SIZE_BUFF - 1 - i];
    }
  }
  */

  // 3) Moving average to smooth further
  movingAverage(g_bufferFilt, g_bufferSmooth, SIZE_BUFF, MOV_AVG_WINDOW);

  // 4) Peak detection & BPM calculation
  float bpm = computeHeartRate(g_bufferSmooth, SIZE_BUFF, (float)SAMPLE_FREQ);

  // Print out the BPM
  Serial.print("Computed BPM = ");
  Serial.println(bpm, 1);

  // Optional short delay before next batch
  delay(1000);
}

// ---------------------------------------------------------------------------
// readSamples() - Acquire SIZE_BUFF samples from ADC at ~SAMPLE_FREQ
// ---------------------------------------------------------------------------
void readSamples()
{
  unsigned long nextSampleTime = millis() + SAMPLE_PERIOD_MS;

  for(int i = 0; i < SIZE_BUFF; i++) {
    // Wait for sampling moment
    while(millis() < nextSampleTime) {
      // idle
    }
    nextSampleTime += SAMPLE_PERIOD_MS;

    // ADC read
    int valInt = analogRead(SENSOR_PIN);
    float valVolt = (valInt * ADC_MAX_VOLT) / (float)ADC_MAX_RES;
    g_bufferRaw[i] = valVolt;
  }
}

// ---------------------------------------------------------------------------
// iirFilter() - Single-pass IIR Direct Form I
// ---------------------------------------------------------------------------
void iirFilter(const float *input, float *output, int length)
{
  // Order 4 => B and A have 5 elements
  static float xHist[5] = {0,0,0,0,0};
  static float yHist[5] = {0,0,0,0,0};

  // Reset filter state for each block (or keep for continuity)
  for(int i = 0; i < 5; i++){
    xHist[i] = 0.0f;
    yHist[i] = 0.0f;
  }

  // Process each sample
  for(int n = 0; n < length; n++){
    // Shift history
    for(int i = 4; i > 0; i--){
      xHist[i] = xHist[i-1];
      yHist[i] = yHist[i-1];
    }
    xHist[0] = input[n];

    // y[n] = b0*x[n] + b1*x[n-1] + ... - a1*y[n-1] - ...
    float y = B[0]*xHist[0];
    for(int i = 1; i < 5; i++){
      y += B[i]*xHist[i] - A[i]*yHist[i];
    }

    // Update history
    yHist[0] = y;
    output[n] = y;
  }
}

// ---------------------------------------------------------------------------
// movingAverage() - Simple sliding window
// ---------------------------------------------------------------------------
void movingAverage(const float *input, float *output, int length, int windowSize)
{
  for(int i = 0; i < length; i++){
    float sum = 0.0f;
    int count = 0;

    int halfWin  = windowSize / 2;
    int startIdx = i - halfWin;
    int endIdx   = i + halfWin;

    if(startIdx < 0) startIdx = 0;
    if(endIdx >= length) endIdx = length - 1;

    for(int j = startIdx; j <= endIdx; j++){
      sum += input[j];
      count++;
    }
    output[i] = sum / (float)count;
  }
}

// ---------------------------------------------------------------------------
// computeHeartRate() - Detect peaks and compute BPM
// ---------------------------------------------------------------------------
float computeHeartRate(const float *signal, int length, float fs)
{
  // 1) Find global max => threshold
  float maxVal = 0.0f;
  for(int i = 0; i < length; i++){
    if(signal[i] > maxVal) {
      maxVal = signal[i];
    }
  }
  float threshold = THRESHOLD_FACTOR * maxVal;

  // 2) Convert min peak distance from sec => samples
  int minDistance = (int)(MIN_PEAK_DIST_SEC * fs);

  // 3) Peak detection
  int lastPeakIdx = -minDistance; 
  float peakLocs[50]; 
  float peakVals[50];
  int peakCount = 0;

  for(int i = 1; i < length - 1; i++){
    // local maxima test & threshold
    if(signal[i] > threshold &&
       signal[i] > signal[i - 1] &&
       signal[i] > signal[i + 1])
    {
      if(i - lastPeakIdx >= minDistance){
        // valid peak
        peakVals[peakCount] = signal[i];
        peakLocs[peakCount] = (float)i;
        peakCount++;
        lastPeakIdx = i;

        if(peakCount >= 50) break; // avoid array overflow
      }
    }
  }

  // 4) Compute average time between peaks => BPM
  if(peakCount < 2) {
    return 0.0f; // insufficient peaks to compute BPM
  }
  float sumRR = 0.0f;
  for(int p = 1; p < peakCount; p++){
    float tPrev = peakLocs[p - 1] / fs;
    float tCurr = peakLocs[p] / fs;
    sumRR += (tCurr - tPrev);
  }
  float meanRR = sumRR / (peakCount - 1); // in seconds
  float bpm    = 60.0f / meanRR;

  // (Optional) Debug: print peak info
  /*
  Serial.println("Peaks (sec, amplitude):");
  for(int p = 0; p < peakCount; p++){
    Serial.print("  t=");
    Serial.print(peakLocs[p]/fs, 3);
    Serial.print("s, A=");
    Serial.println(peakVals[p], 3);
  }
  */

  return bpm;
}
