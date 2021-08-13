
using System;
using System.IO.Ports;
using System.Windows;
using System.Collections.Generic;
//using System.Threading;
using System.Timers;

/* 
 Формат сообщения туда:
    1. Команда
    2. size
    3. LSB
    4. MSB

формат сообщения оттуда (ответ на SET)
    1. Команда
    2. Размер
    3. 0 или 1  как успешно или не успешно выполнилась команда
    

 */

namespace GV_V01
{
    class RS485_Interface
    {
        private const int TIMEOUT = 1000;
        private Queue<MessageToGV> queue;
        private Queue<byte> ReadBuffer;
        private SerialPort _serialPort;
        private GV_struct gV_struct;
        private Timer timeoutTimer;
        
        public RS485_Interface(GV_struct _gvstruct)
        {
            ReadBuffer = new Queue<byte>();
            gV_struct = _gvstruct;
            _serialPort = new SerialPort();
            _serialPort.BaudRate = 9600;
            _serialPort.Parity = Parity.None;
            _serialPort.StopBits = StopBits.One;
            queue = new Queue<MessageToGV>();
            _serialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);

            // устанавливаем метод обратного вызова
            
            // создаем таймер
            timeoutTimer = new Timer();
            timeoutTimer.Interval = TIMEOUT;
            timeoutTimer.Elapsed += TimeoutFunction;
            timeoutTimer.AutoReset = false;


        }
        ~RS485_Interface()
        {
            
            Disconect();
        }
        public string[] GetComPorts()
        {
            return SerialPort.GetPortNames();
        }
        public bool Connect(string comPortName)
        {
            
            _serialPort.Close();
            _serialPort.PortName = comPortName;
            //Открываю порт.
            try { _serialPort.Open(); }
            catch (Exception ex)
            {
                MessageBox.Show("Error: " + ex.ToString(), "ERROR");
                return false;
            }
            return true;

        }
        public void Disconect()
        {
            _serialPort.Close();
        }
        public void AddMsgToQueue(CommandsToGV cmd, uint param = 0)
        {
            //if (!_serialPort.IsOpen) { return; }
            queue.Enqueue(new MessageToGV(cmd, param));
            if (queue.Count == 1) 
                SendMsg();
        }
        private void SendMsg()
        {
            {
                if (!_serialPort.IsOpen) {return;}
                if ((queue.Count != 0))
                { 
                    _serialPort.Write(queue.Peek().GetBytes(), 0, 4);
                    timeoutTimer.Start();
                }
                
            }
        }
        private void GrabMSG(byte[] msg)
        {
            MessageFromGV.GetValues( gV_struct, msg);
            //mutexObj.ReleaseMutex();
            //msgGrabbed = false;
            ReadBuffer.Clear();
            timeoutTimer.Stop();
            queue.Dequeue();
            SendMsg();
        }
        private void DataReceivedHandler(
                    object sender,
                    SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;
            //int b;
            while (sp.BytesToRead > 0)
            {
                //b = sp.ReadByte();
                ReadBuffer.Enqueue((byte)sp.ReadByte()); 
            }

            if (ReadBuffer.Count > 3)
            {
                if (ReadBuffer.Count == ReadBuffer.ToArray()[1])
                {
                    GrabMSG(ReadBuffer.ToArray());
                }
            }
            if (ReadBuffer.Count > 0x22) { 
                ReadBuffer.Clear();
                _serialPort.DiscardInBuffer();
                timeoutTimer.Stop();
                //queue.Dequeue();
                SendMsg();
                // mutexObj.ReleaseMutex();
            }

        }
        private void TimeoutFunction(Object source, System.Timers.ElapsedEventArgs e)
        {
            //MessageBox.Show("TimeOut MSG"); 
            SendMsg();
        }


    }
}
