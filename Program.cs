
/* This code is deeply nested. As this is a small single-file application, I didn't bother refactoring it. 
    One could however create a new class for example, which would contain most of the accessed variables inside the two for-loops (line 207 and 211). 
    Then define object methods when needed. The methods are then called inside the for-loops. 
    Currently the csv-file is opened before initiating the measurements and it is only closed once the program ends. Better would be to first acquire all measurements,
    save them in a 2d-array and after all measurements have been taken, write that array to the csv file. */


using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.IO;
using Thorlabs.MotionControl.DeviceManagerCLI;
using Thorlabs.MotionControl.GenericMotorCLI;
using Thorlabs.MotionControl.GenericMotorCLI.ControlParameters;
using Thorlabs.MotionControl.GenericMotorCLI.AdvancedMotor;
using Thorlabs.MotionControl.GenericMotorCLI.Settings;
using Thorlabs.MotionControl.Benchtop.BrushlessMotorCLI;
using SwabianInstruments.TimeTagger;

namespace scan2d
{
    class Program
    {
        static void Main(string[] args)
        {
            // User Input 1 (more user input fields below)
            // ------------------------------------------- 
            string serialNo = "00000000"; // Thorlabs serial number
            decimal velocity = 5m; // Thorlabs stage max velocity
            int[] channelToBeUsed = new int[] { 1, 2 };  // Time Tagger channels
            // ------------------------------------------- 


            // Initialising Swabian Instruments Time Tagger

            TimeTagger tt = TT.createTimeTagger();
            Console.WriteLine($"Time Tagger serial number is: {tt.getSerial()}\n");
            Countrate countrate = new Countrate(tt, channelToBeUsed);


            // Initialising Thorlabs motion controller

            try
            {
                // Tell the device manager to get the list of all devices connected to the computer, such as the Brushless Motor Controller
                DeviceManagerCLI.BuildDeviceList();
            }
            catch (Exception ex)
            {
                // An error occurred - see ex for details
                Console.WriteLine("Exception raised by BuildDeviceList {0}", ex);
                Console.ReadKey();
                return;
            }
            // Get available Benchtop Brushless Motor and check our serial number is correct - by using the device prefix
            // (i.e. for serial number 73000123, the device prefix is 73)
            List<string> serialNumbers = 
            DeviceManagerCLI.GetDeviceList(BenchtopBrushlessMotor.DevicePrefix73);
            if (!serialNumbers.Contains(serialNo))
            {
                // The requested serial number is not a BBDxx3, or is not connected
                Console.WriteLine("{0} is not a valid serial number", serialNo);
                Console.ReadKey();
                return;
            }
            // Create the device - Brushless motor
            BenchtopBrushlessMotor device = BenchtopBrushlessMotor.CreateBenchtopBrushlessMotor(serialNo);
            if (device == null)
            {
                // An error occured
                Console.WriteLine("{0} is not a BenchtopBrushlessMotor", serialNo);
                Console.ReadKey();
                return;
            }
            // Open a connection to the device.
            try
            {
                Console.WriteLine("Opening device {0}", serialNo);
                device.Connect(serialNo);
            }
            catch (Exception)
            {
                // Connection failed
                Console.WriteLine("Failed to open device {0}", serialNo);
                Console.ReadKey();
                return;
            }
            BrushlessMotorChannel channelx = device.GetChannel(1);
            if (channelx == null)
            {
                // Connection failed
                Console.WriteLine("Channel unavailable {0}", serialNo);
                Console.ReadKey();
                return;
            }
            // Get the other channel - channel 2
            BrushlessMotorChannel channely = device.GetChannel(2);
            if (channely == null)
            {
                // Connection failed
                Console.WriteLine("Channel unavailable {0}", serialNo);
                Console.ReadKey();
                return;
            }
            // Wait for the device settings to initialize - timeout 5000ms
            if (!channelx.IsSettingsInitialized())
            {
                try
                {
                    channelx.WaitForSettingsInitialized(5000);
                }
                catch (Exception)
                {
                    Console.WriteLine("Settings failed to initialize");
                }
            }
            if (!channely.IsSettingsInitialized())
            {
                try
                {
                    channely.WaitForSettingsInitialized(5000);
                }
                catch (Exception)
                {
                    Console.WriteLine("Settings for channel 2 failed to initialize");
                }
            }

            // Start the device polling
            // The polling loop requests regular status requests to the motor to ensure the program keeps track of the device. 
            channelx.StartPolling(250);
            // Needs a delay so that the current enabled state can be obtained
            Thread.Sleep(500);
            // Enable the channel otherwise any move is ignored 
            channelx.EnableDevice();
            // Needs a delay to give time for the device to be enabled
            Thread.Sleep(500);

            //Same for channel 2
            channely.StartPolling(250);
            Thread.Sleep(500);
            channely.EnableDevice();
            Thread.Sleep(500);

            // Call LoadMotorConfiguration on the device to initialize the DeviceUnitConverter object required for real world unit parameters
            //  - loads configuration information into channel
            // Use the channel.DeviceID "73xxxxxx-1" to get the channel 1 settings. This is different to the serial number
            MotorConfiguration motorConfiguration = channelx.LoadMotorConfiguration(channelx.DeviceID);
            MotorConfiguration motorConfiguration2 = channely.LoadMotorConfiguration(channely.DeviceID);

            BrushlessMotorSettings currentDeviceSettings = channelx.MotorDeviceSettings as BrushlessMotorSettings;
            BrushlessMotorSettings currentDeviceSettings2 = channely.MotorDeviceSettings as BrushlessMotorSettings;

            // Display info about device
            DeviceInfo deviceInfo = channelx.GetDeviceInfo();
            Console.WriteLine("Device {0} = {1}", deviceInfo.SerialNumber, deviceInfo.Name);
            DeviceInfo deviceInfo2 = channely.GetDeviceInfo();
            Console.WriteLine("Device {0} = {1}", deviceInfo2.SerialNumber, deviceInfo2.Name);

            Console.WriteLine("Channel x homed?");
            Console.WriteLine(channelx.Status.IsHomed);
            //Home_Method(channelx);

            Console.WriteLine("Channel y homed?");
            Console.WriteLine(channely.Status.IsHomed);
            //Home_Method(channely);

            VelocityParameters velPars = channelx.GetVelocityParams();
            velPars.MaxVelocity = velocity;
            channelx.SetVelocityParams(velPars);

            VelocityParameters velPars2 = channely.GetVelocityParams();
            velPars2.MaxVelocity = velocity;
            channely.SetVelocityParams(velPars2);

            // Finished Initialising Thorlabs motion controller 
            Console.ReadKey();



            bool programTerminated = false;
            while (!programTerminated)
            {
            // User Input 2
            // -------------------------------------------------------------
            var filepath = "data.csv";
            // all positions and lengths are in mm
            decimal startPositionX = 55.000m;
            decimal startPositionY = 37.500m;
            decimal widthX = 0.010m;
            decimal widthY = 0.010m;
            decimal stepsize = 0.001m;
            decimal accuracy = 0.00005m;

            int measurementDuration = 3 * 100000000000; // Acquire for 0.3 seconds
            // -------------------------------------------------------------

            int ximax = Convert.ToInt32(widthX/stepsize);
            int yimax = Convert.ToInt32(widthY/stepsize);
            double[,] data = new double[ximax,yimax];
            string line = "";

            using (StreamWriter writer = new StreamWriter(new FileStream(filepath, FileMode.Create, FileAccess.Write)))
            {
                for (int xi = 0; xi < ximax; xi++)
                {
                    Move_With_highAccuracy(channelx, startPositionX + xi * stepsize, accuracy);
                    Thread.Sleep(50);
                    for (int yi = 0; yi < yimax; yi++)
                    {
                        Move_With_highAccuracy(channely, startPositionY + yi * stepsize, accuracy);
                        
                        Thread.Sleep(500); 
                        countrate.startFor(measurement_Duration); 
                        int index1 = 0;
                        while (countrate.isRunning())
                        {
                            Thread.Sleep(50);
                            if (index1 > 1 + measurement_Duration*0.000001/50)
                            {
                                Console.WriteLine("Time tagger stopped resopnding");
                                Console.WriteLine(tt.getOverflows());
                                Console.ReadKey();
                            }
                            index1++;
                        }

                        // Sum the countrates of detector 0 and 1
                        data[xi, yi] = countrate.getData()[0] + countrate.getData()[1];

                        // Add the measured value to the current row
                        if (yi > 0)
                        {
                            line = string.Join(",", line, data[xi, yi].ToString("0.0"));
                        }
                        else if (yi == 0)
                        {
                            line = data[xi, yi].ToString("0.0");
                        }
                    }
                    // Add the current row to the csv-file
                    writer.WriteLine(line);
                    line = "";
                }
                writer.Flush();
                writer.Close();
            } // closing the StreamWriter

            Console.WriteLine(tt.getOverflows());
            Console.WriteLine(tt.getConfiguration());
            Console.WriteLine(channelx.Position.ToString());
            Console.WriteLine(channely.Position.ToString());

            // Get user input
            String UserInput = Console.ReadLine();
            // Checks the input for the keyword "terminate"
            // This structure allows for modifications of the code while the program is running without having to reinitialise the devices each time
            if (UserInput == "terminate")
            {
                programTerminated = true;
            }
            } // end of while loop
            channelx.StopPolling();
            channely.StopPolling();
            device.Disconnect(true);
        } // end of Main method



        public static void Move_With_highAccuracy(BrushlessMotorChannel channel, decimal position, decimal accuracy)
        {
            Move_Method(channel, position + 0.1m);
            Thread.Sleep(30);
            decimal currentPosition = channel.Position;
            int numberOfIterations = 0;
            while (currentPosition > position + accuracy || currentPosition < position - accuracy)
            {
                // Console.WriteLine("start");
                if (currentPosition > position + accuracy)
                {
                    Move_Method(channel, position - accuracy * 2);
                    Thread.Sleep(30);
                    Move_Method(channel, position);
                    Thread.Sleep(30);
                    currentPosition = channel.Position;
                }
                if (currentPosition < position - accuracy)
                {
                    Move_Method(channel, position + accuracy * 2);
                    Thread.Sleep(30);
                    Move_Method(channel, position);
                    Thread.Sleep(30);
                    currentPosition = channel.Position;
                }

                numberOfIterations++;
                if (numberOfIterations >= 20)
                {
                    Console.WriteLine("Procedure did not converge.");
                    Console.WriteLine(position);
                    Console.WriteLine(currentPosition);
                    Console.ReadKey();
                }
            }
            return;
        }

        public static void Home_Method(IGenericAdvancedMotor device)
        {
            try
            {
                Console.WriteLine("Homing device");
                device.Home(60000);
            }
            catch (Exception)
            {
                Console.WriteLine("Failed to home device");
                Console.ReadKey();
                return;
            }
            Console.WriteLine("Device Homed");
        }

        public static void Move_Method(IGenericAdvancedMotor device, decimal position)
        {
            try
            {
                // Console.WriteLine("Moving Device to {0}", position);
                device.MoveTo(position, 60000);
            }
            catch (Exception)
            {
                Console.WriteLine("Failed to move to position {0}", position);
                Console.ReadKey();
                return;
            }
        }

    } // end of class 'Program'
} // end of namespace