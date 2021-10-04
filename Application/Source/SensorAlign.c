#include "SensorAlign.h"


void SensorAlign(INT16S *Sensor, AlignDirec_e Align)
{
    const INT16S X = Sensor[IMUAxisX];
    const INT16S Y = Sensor[IMUAxisY];
    const INT16S Z = Sensor[IMUAxisZ];
    
    switch(Align)
    {
        case CW0_POSITIVE:
            Sensor[IMUAxisX] = X;
            Sensor[IMUAxisY] = Y;
            Sensor[IMUAxisZ] = Z;
            break;
        case CW90_POSITIVE:
            Sensor[IMUAxisX] = Y;
            Sensor[IMUAxisY] = -X;
            Sensor[IMUAxisZ] = Z;
            break;
        case CW180_POSITIVE:
            Sensor[IMUAxisX] = -X;
            Sensor[IMUAxisY] = -Y;
            Sensor[IMUAxisZ] = Z;
            break;
        case CW270_POSITIVE:
            Sensor[IMUAxisX] = -Y;
            Sensor[IMUAxisY] = X;
            Sensor[IMUAxisZ] = Z;
            break;
        case CW0_FLIP:
            Sensor[IMUAxisX] = -X;
            Sensor[IMUAxisY] = Y;
            Sensor[IMUAxisZ] = -Z;
            break;
        case CW90_FLIP:
            Sensor[IMUAxisX] = Y;
            Sensor[IMUAxisY] = X;
            Sensor[IMUAxisZ] = -Z;
            break;
        case CW180_FLIP:
            Sensor[IMUAxisX] = X;
            Sensor[IMUAxisY] = -Y;
            Sensor[IMUAxisZ] = -Z;
            break;
        case CW270_FLIP:
            Sensor[IMUAxisX] = -Y;
            Sensor[IMUAxisY] = -X;
            Sensor[IMUAxisZ] = -Z;
            break;
    }
}

