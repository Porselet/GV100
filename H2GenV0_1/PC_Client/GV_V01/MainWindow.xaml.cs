using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

using System.Threading;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Configuration;

namespace GV_V01
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        GV_struct gV_Struct;
        RS485_Interface rS485_Interface;
        Timer timer;
        public MainWindow()
        {
            InitializeComponent();
            string ComPortName = ConfigurationManager.AppSettings["ComPort"];
            gV_Struct = (GV_struct)this.Resources["Values"];
            if (gV_Struct == null) { MessageBox.Show("Dermo"); this.Close(); return; }
             rS485_Interface = new RS485_Interface(gV_Struct);
            rS485_Interface.Connect(ComPortName);
           // rS485_Interface.AddMsgToQueue(CommandsToGV.CMD_SET_ACTIVE,1);
            // устанавливаем метод обратного вызова
            TimerCallback tm = new TimerCallback(this.fu);
            // создаем таймер
            timer = new Timer(tm, null, 0, 1000);

            //Console.ReadLine();
        }
        void fu(object obj)
        {

           // rS485_Interface.AddMsgToQueue(CommandsToGV.CMD_SET_SETPOINT, 1000);
            rS485_Interface.AddMsgToQueue(CommandsToGV.CMD_GET_VALUES);
            //rS485_Interface.AddMsgToQueue(CommandsToGV.CMD_SET_ACTIVE);

        }

        private void buttonOFF_Click(object sender, RoutedEventArgs e)
        {
            rS485_Interface.AddMsgToQueue(CommandsToGV.CMD_SET_ACTIVE, 0);

        }
        private void buttonON_Click(object sender, RoutedEventArgs e)
        {
            rS485_Interface.AddMsgToQueue(CommandsToGV.CMD_SET_ACTIVE, 1);

        }

        private void button2_Click(object sender, RoutedEventArgs e)
        {
            ushort Setpoint;


            if (UInt16.TryParse(textBox.Text, out Setpoint))
            {
                if ((Setpoint > 0) && (Setpoint < 1101))
                rS485_Interface.AddMsgToQueue(CommandsToGV.CMD_SET_SETPOINT, Setpoint);
            }
            else 
            {
                MessageBox.Show("Не валидное значение");
            }
            //rS485_Interface.AddMsgToQueue(CommandsToGV.CMD_SET_SETPOINT, textBox.Text.to );
        }
    }
}
