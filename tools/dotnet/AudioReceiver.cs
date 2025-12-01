using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Windows.Forms;
using NAudio.Wave;
using OxyPlot;
using OxyPlot.Series;
using OxyPlot.WindowsForms;
using System.Drawing; // For layout/colors

namespace AudioReceiver
{
    static class Program
    {
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MainWindow());
        }
    }

    public class MainWindow : Form
    {
        // --- Configuration ---
        const int PORT = 5000;
        const int SAMPLE_RATE = 16000;
        const int CHANNELS = 2;
        const float DIGITAL_GAIN = 4.0f;
        
        // --- State ---
        private UdpClient udp;
        private BufferedWaveProvider audioBuffer;
        private WaveOutEvent waveOut;
        private Thread networkThread;
        private volatile bool isRunning = true;
        
        private enum RecordingState { Stopped, Recording, Paused }
        private volatile RecordingState currentRecState = RecordingState.Stopped;

        // --- UI Elements ---
        private PlotView plotView;
        private Button btnRecord;
        private Button btnPause;
        private Button btnStop;
        private Button btnExit;
        private Label lblStatus;

        // --- Plotting ---
        private LineSeries leftChannelSeries;
        private LineSeries rightChannelSeries;
        private int plotPointCount = 0;
        private const int MAX_POINTS = 500; 

        // --- Data Saving ---
        private List<short> recordedData = new List<short>();

        public MainWindow()
        {
            // 1. Setup Window
            this.Text = "ESP32 Audio Recorder";
            this.Size = new Size(900, 600);
            
            // Layout: Split container (Plot Top, Controls Bottom)
            var mainLayout = new TableLayoutPanel();
            mainLayout.Dock = DockStyle.Fill;
            mainLayout.RowCount = 2;
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 90F)); // Plot gets 90%
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 60F)); // Controls get 60px
            this.Controls.Add(mainLayout);

            // 2. Setup Plot
            plotView = new PlotView { Dock = DockStyle.Fill };
            var model = new PlotModel { Title = "Real-Time Audio Waveform" };
            
            leftChannelSeries = new LineSeries { Title = "Left", Color = OxyColors.Green, StrokeThickness = 1 };
            model.Series.Add(leftChannelSeries);

            rightChannelSeries = new LineSeries { Title = "Right", Color = OxyColors.Blue, StrokeThickness = 1 };
            model.Series.Add(rightChannelSeries);

            model.Axes.Add(new OxyPlot.Axes.LinearAxis { Position = OxyPlot.Axes.AxisPosition.Left, Minimum = -35000, Maximum = 35000, Title="Amplitude" });
            model.Axes.Add(new OxyPlot.Axes.LinearAxis { Position = OxyPlot.Axes.AxisPosition.Bottom, Title="Time (samples)" });
            
            plotView.Model = model;
            mainLayout.Controls.Add(plotView, 0, 0);

            // 3. Setup Control Panel
            var controlPanel = new FlowLayoutPanel();
            controlPanel.Dock = DockStyle.Fill;
            controlPanel.FlowDirection = FlowDirection.LeftToRight;
            controlPanel.Padding = new Padding(10);
            controlPanel.AutoSize = true;

            // Create Buttons
            btnRecord = CreateButton("● Record", Color.Red, (s, e) => StartRecording());
            btnPause = CreateButton("|| Pause", Color.Orange, (s, e) => PauseRecording());
            btnStop = CreateButton("■ Stop & Save", Color.Black, (s, e) => StopAndSave());
            btnExit = CreateButton("Exit App", Color.Gray, (s, e) => this.Close());

            btnPause.Enabled = false;
            btnStop.Enabled = false;

            lblStatus = new Label { Text = "Ready", AutoSize = true, Font = new Font("Segoe UI", 12, FontStyle.Bold), Margin = new Padding(10, 8, 0, 0) };

            controlPanel.Controls.Add(btnRecord);
            controlPanel.Controls.Add(btnPause);
            controlPanel.Controls.Add(btnStop);
            controlPanel.Controls.Add(new Label { Width = 20 }); // Spacer
            controlPanel.Controls.Add(btnExit);
            controlPanel.Controls.Add(new Label { Width = 20 }); // Spacer
            controlPanel.Controls.Add(lblStatus);

            mainLayout.Controls.Add(controlPanel, 0, 1);


            // 4. Setup Audio Engine
            var waveFormat = new WaveFormat(SAMPLE_RATE, 16, CHANNELS);
            audioBuffer = new BufferedWaveProvider(waveFormat);
            audioBuffer.BufferDuration = TimeSpan.FromMilliseconds(2000);
            audioBuffer.DiscardOnBufferOverflow = true;

            waveOut = new WaveOutEvent();
            waveOut.Init(audioBuffer);

            // 5. Start Network Thread
            networkThread = new Thread(NetworkLoop);
            networkThread.IsBackground = true;
            networkThread.Start();

            // 6. Setup Timer for UI Updates
            System.Windows.Forms.Timer uiTimer = new System.Windows.Forms.Timer();
            uiTimer.Interval = 33; // ~30 FPS
            uiTimer.Tick += (s, e) => {
                plotView.InvalidatePlot(true);
                // Update rec count label
                if(currentRecState == RecordingState.Recording) {
                    lock(recordedData) {
                        lblStatus.Text = $"Recording... ({recordedData.Count / 2} samples)";
                    }
                }
            };
            uiTimer.Start();

            this.FormClosing += OnClosing;
        }

        private Button CreateButton(string text, Color color, EventHandler onClick)
        {
            var btn = new Button();
            btn.Text = text;
            btn.ForeColor = color;
            btn.Font = new Font("Segoe UI", 10, FontStyle.Bold);
            btn.AutoSize = true;
            btn.Padding = new Padding(5);
            btn.Click += onClick;
            return btn;
        }

        // --- Control Logic ---

        private void StartRecording()
        {
            if (currentRecState == RecordingState.Stopped)
            {
                lock(recordedData) recordedData.Clear();
            }
            currentRecState = RecordingState.Recording;
            btnRecord.Enabled = false;
            btnPause.Enabled = true;
            btnStop.Enabled = true;
            lblStatus.Text = "Recording...";
            lblStatus.ForeColor = Color.Red;
        }

        private void PauseRecording()
        {
            currentRecState = RecordingState.Paused;
            btnRecord.Enabled = true;
            btnPause.Enabled = false;
            lblStatus.Text = "Paused";
            lblStatus.ForeColor = Color.Orange;
        }

        private void StopAndSave()
        {
            currentRecState = RecordingState.Stopped;
            btnRecord.Enabled = true;
            btnPause.Enabled = false;
            btnStop.Enabled = false;
            lblStatus.Text = "Stopped";
            lblStatus.ForeColor = Color.Black;

            // Save Dialog for WAV
            using (SaveFileDialog sfd = new SaveFileDialog())
            {
                sfd.Filter = "WAV File|*.wav";
                sfd.Title = "Save Training Data";
                // Default name helps you organize: Label_Number.wav
                sfd.FileName = $"recording_{DateTime.Now:HHmmss}.wav";

                if (sfd.ShowDialog() == DialogResult.OK)
                {
                    SaveToWav(sfd.FileName);
                }
            }
        }

        private void SaveToWav(string filename)
        {
            try 
            {
                // Use the format defined in your setup (16kHz, 16-bit, Stereo)
                var format = new WaveFormat(SAMPLE_RATE, 16, CHANNELS);

                using (var writer = new WaveFileWriter(filename, format))
                {
                    lock(recordedData)
                    {
                        // Convert List<short> back to byte[]
                        byte[] buffer = new byte[recordedData.Count * 2];
                        for (int i = 0; i < recordedData.Count; i++)
                        {
                            byte[] bytes = BitConverter.GetBytes(recordedData[i]);
                            buffer[i * 2] = bytes[0];
                            buffer[i * 2 + 1] = bytes[1];
                        }
                        writer.Write(buffer, 0, buffer.Length);
                    }
                }
                MessageBox.Show($"Saved {recordedData.Count / CHANNELS} samples to:\n{filename}");
            }
            catch(Exception ex)
            {
                MessageBox.Show($"Error saving file: {ex.Message}");
            }
        }

        private void SaveToCsv(string filename)
        {
            try 
            {
                using (StreamWriter writer = new StreamWriter(filename))
                {
                    writer.WriteLine("Index,Left,Right");
                    lock(recordedData)
                    {
                        for (int i = 0; i < recordedData.Count; i += 2)
                        {
                            if (i + 1 < recordedData.Count)
                            {
                                writer.WriteLine($"{i/2},{recordedData[i]},{recordedData[i+1]}");
                            }
                        }
                    }
                }
                MessageBox.Show($"Saved successfully to:\n{filename}");
            }
            catch(Exception ex)
            {
                MessageBox.Show($"Error saving file: {ex.Message}");
            }
        }

        // --- Network Loop ---
        private void NetworkLoop()
        {
            udp = new UdpClient(PORT);
            IPEndPoint ep = new IPEndPoint(IPAddress.Any, PORT);
            bool isPlaying = false;

            try
            {
                while (isRunning)
                {
                    byte[] packet = udp.Receive(ref ep);
                    
                    if (packet.Length > 12) 
                    {
                        int payloadLen = packet.Length - 12;
                        byte[] rawAudio = new byte[payloadLen];
                        Array.Copy(packet, 12, rawAudio, 0, payloadLen);

                        int sampleCount = payloadLen / 2;
                        short[] samples = new short[sampleCount];
                        byte[] amplifiedBytes = new byte[payloadLen];

                        for (int i = 0; i < sampleCount; i++)
                        {
                            short sample = BitConverter.ToInt16(rawAudio, i * 2);
                            
                            // Gain
                            int amplified = (int)(sample * DIGITAL_GAIN);
                            if (amplified > 32767) amplified = 32767;
                            if (amplified < -32768) amplified = -32768;
                            
                            samples[i] = (short)amplified;
                        }

                        // RECORDING LOGIC
                        if (currentRecState == RecordingState.Recording)
                        {
                            lock(recordedData) {
                                recordedData.AddRange(samples);
                            }
                        }

                        // PLAYBACK LOGIC (Always plays, regardless of recording state)
                        Buffer.BlockCopy(samples, 0, amplifiedBytes, 0, amplifiedBytes.Length);
                        audioBuffer.AddSamples(amplifiedBytes, 0, amplifiedBytes.Length);

                        if (!isPlaying && audioBuffer.BufferedDuration.TotalMilliseconds > 200)
                        {
                            waveOut.Play();
                            isPlaying = true;
                        }

                        // PLOTTING LOGIC
                        lock(plotView.Model.SyncRoot)
                        {
                            for(int i = 0; i < sampleCount; i+=2) 
                            {
                                if(i % 8 == 0) { // Downsample for GUI performance
                                    if (leftChannelSeries.Points.Count > MAX_POINTS) leftChannelSeries.Points.RemoveAt(0);
                                    if (rightChannelSeries.Points.Count > MAX_POINTS) rightChannelSeries.Points.RemoveAt(0);
                                    
                                    leftChannelSeries.Points.Add(new DataPoint(plotPointCount, samples[i])); 
                                    rightChannelSeries.Points.Add(new DataPoint(plotPointCount, samples[i+1]));
                                    plotPointCount++;
                                }
                            }
                        }
                    }
                }
            }
            catch (Exception) { }
        }

        private void OnClosing(object sender, FormClosingEventArgs e)
        {
            isRunning = false;
            udp?.Close();
            waveOut?.Stop();
            waveOut?.Dispose();
        }
    }
}