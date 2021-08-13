using System;
using System.Collections.Generic;
using System.Text;
using System.ComponentModel;

namespace GV_V01
{
    class GV_struct : INotifyPropertyChanged
    {
        public enum Modes
        {
            MODE_IDLE,
            MODE_TRY_TO_START,
            MODE_LEAK_CHECK1,
            MODE_LEAK_CHECK2,
            MODE_NORMAL_WORK,
            MODE_TRY_TO_STOP,
            MODE_ERROR_LEAK_CHECK,
            MODE_ERROR_WQ,
            MODE_ERROR_PRESS,
            MODE_ERROR_TEMP,
            MODE_ERROR_LOW_OXYGEN_LEVEL,
            MODE_ERROR_HIGH_OXYGEN_LEVEL,
            MODE_ERROR_LOW_HYDROGEN_LEVEL,
            MODE_ERROR_HIGH_HYDROGEN_LEVEL,
            MODE_ERROR_CONNECT_TO_POWER_SUPPLY,
            MODE_ERROR_RESERVE_1,
            MODE_ERROR_RESERVE_2,
        };

        private bool levelSensorHiHydrogen;
        private bool levelSensorLowHydrogen;
        private bool levelSensorHiOxygen;
        private bool levelSensorLowOxygen;


        private uint amperage; 
        private uint voltage;
        private uint amperageSetpoint;

        private float press;
        private float temperature;
        private float wQ;


        private Modes mode;
	    private uint ahCounter;
        private uint sysCounter;

        public bool LevelSensorHiHydrogen
        {
            get { return levelSensorHiHydrogen; } 
            set { levelSensorHiHydrogen = value; OnPropertyChanged("LevelSensorHiHydrogen"); }
        }
        public bool LevelSensorLowHydrogen
        {
            get { return levelSensorLowHydrogen; }
            set { levelSensorLowHydrogen = value; OnPropertyChanged("LevelSensorLowHydrogen"); }
        }
        public bool LevelSensorHiOxygen
        {
            get { return levelSensorHiOxygen; }
            set { levelSensorHiOxygen = value; OnPropertyChanged("LevelSensorHiOxygen"); }
        }
        public bool LevelSensorLowOxygen
        {
            get { return levelSensorLowOxygen; }
            set { levelSensorLowOxygen = value; OnPropertyChanged("LevelSensorLowOxygen"); }
        }
        public uint Amperage
        {
            get { return amperage; }
            set { amperage = value; OnPropertyChanged("Amperage"); }
        }

        public uint Voltage
        {
            get { return voltage; }
            set { voltage = value; OnPropertyChanged("Voltage"); }
        }
        public uint AmperageSetpoint
        {
            get { return amperageSetpoint; }
            set { amperageSetpoint = value; OnPropertyChanged("AmperageSetpoint"); }
        }
        public float Press
        {
            get {return press; }
            set {press = value; OnPropertyChanged("Press");}
        }
        public float Temperature
        {
            get { return temperature; }
            set { temperature = value; OnPropertyChanged("Temperature"); }
        }
        public float WQ
        {
            get { return wQ; }
            set { wQ = value; OnPropertyChanged("WQ"); }
        }
        public Modes Mode
        {
            get { return mode; }
            set { mode = value; OnPropertyChanged("Mode");
                OnPropertyChanged("ModeAsString"); 
            }
        }
        public string ModeAsString
        {
            get 
            {
                switch (mode)
                {
                    case Modes.MODE_ERROR_CONNECT_TO_POWER_SUPPLY:
                        return "Ошибка связи с блоком питания";
                    case Modes.MODE_ERROR_HIGH_HYDROGEN_LEVEL:
                        return "Высокий уровень в водородной колонке";
                    case Modes.MODE_ERROR_HIGH_OXYGEN_LEVEL:
                        return "Высокий уровень в кислородной колонке";
                    case Modes.MODE_ERROR_LOW_HYDROGEN_LEVEL:
                        return "Низкий уровень в водородной колонке";
                    case Modes.MODE_ERROR_LOW_OXYGEN_LEVEL:
                        return "Низкий уровень в кислородной колонке";
                    case Modes.MODE_ERROR_LEAK_CHECK:
                        return "Не прошел тест на утечку";
                    case Modes.MODE_ERROR_PRESS:
                        return "Ошибка по двлению";
                    case Modes.MODE_ERROR_TEMP:
                        return "Ошибка по температуре";
                    case Modes.MODE_ERROR_WQ:
                        return "Ошибка по чистоте воды";
                    case Modes.MODE_IDLE:
                        return "Режим ожидания";
                    case Modes.MODE_LEAK_CHECK1:
                        return "Проверка на утечку 1";
                    case Modes.MODE_LEAK_CHECK2:
                        return "Проверка на утечку 2";
                    case Modes.MODE_NORMAL_WORK:
                        return "Нормальный режим работы";
                    case Modes.MODE_TRY_TO_START:
                        return "Запуск";
                    case Modes.MODE_TRY_TO_STOP:
                        return "Остановка";
                    case Modes.MODE_ERROR_RESERVE_1:
                        return "Резервная ошибка 1";
                    case Modes.MODE_ERROR_RESERVE_2:
                        return "Резервная ошибка 2";
                    default:
                        return "что то не так";
                }
            }
        }
        public uint AhCounter
        {
            get { return ahCounter; }
            set { ahCounter = value; OnPropertyChanged("AhCounter"); }
        }
        public uint SysCounter
        {
            get { return sysCounter; }
            set { sysCounter = value; OnPropertyChanged("SysCounter"); }
        }

        public event PropertyChangedEventHandler PropertyChanged;
        /*public void OnPropertyChanged([CallerMemberName] string prop = "")
        {
            if (PropertyChanged != null)
                PropertyChanged(this, new PropertyChangedEventArgs(prop));
        } */
        private void OnPropertyChanged(string propertyName)
        {
            var handler = PropertyChanged;
            if (handler != null)
            {
                handler(this, new PropertyChangedEventArgs(propertyName));
            }
        }



    }
}
