using System;
using System.Collections.Generic;
using System.Text;

namespace GV_V01
{

    public enum CommandsToGV
    {
        CMD_SET_ACTIVE,
        CMD_SET_SETPOINT,
        CMD_GET_VALUES,
        CMD_GET_RAW_VALUES,
    }
    class MessageToGV
    {
        private CommandsToGV cmd;
        //private byte size;
        private uint bytes;

        public MessageToGV(CommandsToGV _cmd, uint param)
        {
            cmd = _cmd;
            bytes = param;
        }
        public byte[] GetBytes()
        {
            //byte[] mess = { (byte)cmd, 4, (byte)bytes, (byte)(bytes >> 8) };
            byte a = (byte)bytes;
            byte b = (byte)(bytes >> 8);
            byte[] mess = { (byte)cmd, 4, a, b };
            return mess;
        }
    }
    class MessageFromGV
    {
        public static bool GetValues(GV_struct values, byte[] bytes)
        {
            /* 
struct AnswerValues		
{
	enum CommandsPC cmd;
	uint8_t size;
	uint32_t AhCounter;
	uint32_t sysCounter;
	float Press;
	float Temperature;
	float WQ;
	uint16_t AmperageSetpoint;
	uint16_t Amperage;
	uint16_t Voltage;
	char D_IN_1;
	char D_IN_2;
	char D_IN_3;
	char D_IN_4;	
	enum GENERATOR_MODES Mode;

};*/

            if (bytes[0] == (byte)CommandsToGV.CMD_GET_VALUES)
            {

                values.AhCounter = BitConverter.ToUInt32(bytes, 2);
                values.SysCounter = BitConverter.ToUInt32(bytes, 6);
                values.Press = BitConverter.ToSingle(bytes, 10);
                values.Temperature = BitConverter.ToSingle(bytes, 14);
                values.WQ = BitConverter.ToSingle(bytes, 18);
                values.AmperageSetpoint = BitConverter.ToUInt16(bytes, 22);
                values.Amperage = BitConverter.ToUInt16(bytes, 24);
                values.Voltage = BitConverter.ToUInt16(bytes, 26);

                values.LevelSensorHiHydrogen = Convert.ToBoolean(bytes[28]);
                values.LevelSensorLowHydrogen = Convert.ToBoolean(bytes[29]);
                values.LevelSensorHiOxygen = Convert.ToBoolean(bytes[30]);
                values.LevelSensorLowOxygen = Convert.ToBoolean(bytes[31]);
                values.Mode = (GV_struct.Modes)(bytes[32]);
            }
            return false;
        }
    }



}
