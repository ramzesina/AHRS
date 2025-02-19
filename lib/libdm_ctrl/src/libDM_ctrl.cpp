#include "libDM_ctrl.hpp"

/**************************************************************
 * [CTRL_DM::Integrator_DM Integrator with Min Max and Reset] *
 * @param Te        [Sample Time]                             *
 * @param X0        [Init Value]                              *
 * @param Reset     [Reset Trigger]                           *
 * @param Min       [Min Value of Integ_Val]                  *
 * @param Max       [Max Value of Integ_Val]                  *
 * @param Value     [Value to integrate]                      *
 * @param Integ_Val [Sum of integrated Values]                *
 **************************************************************/
void CTRL_DM::Integrator_DM(float Te,
                            float X0,
                            bool Reset,
                            float Min,
                            float Max,
                            float Value,
                            float *Integ_Val)
{

    if (Reset)
    {
        *Integ_Val = X0;
    }
    else
    {
        *Integ_Val += Te * Value;
    }

    *Integ_Val = std::max(std::min(*Integ_Val, Max), Min);
}

/***********************************************
 * [LPF1_DM First order filter with Reset]     *
 * @param Te           [Sample Time]           *
 * @param X0           [Init Value]            *
 * @param Reset        [Reset Trigger]         *
 * @param Tau          [Time Constant]         *
 * @param Value        [Value to Filter]       *
 * @param Filtered_Val [Filtered output value] *
 ***********************************************/
void CTRL_DM::LPF1_DM(float Te, float X0, bool Reset, float Tau, float Value, float *Filtered_Val)
{

    if (Reset)
    {
        *Filtered_Val = X0;
    }
    else
    {
        *Filtered_Val = (Value * Te + *Filtered_Val * Tau) / (Te + Tau);
    }
}

/**************************************
 * [LPF2_DM description]              *
 * @param Te            [description] *
 * @param X0            [description] *
 * @param Reset         [description] *
 * @param Tau1          [description] *
 * @param Tau2          [description] *
 * @param Value         [description] *
 * @param Filtered_Val1 [description] *
 * @param Filtered_Val2 [description] *
 **************************************/
void CTRL_DM::LPF2_DM(float Te,
                      float X0,
                      bool Reset,
                      float Tau1,
                      float Tau2,
                      float Value,
                      float *Filtered_Val1,
                      float *Filtered_Val2)
{
    LPF1_DM(Te, X0, Reset, Tau1, Value, Filtered_Val1);
    LPF1_DM(Te, X0, Reset, Tau2, *Filtered_Val1, Filtered_Val2);
}

/****************************************************************************
 * [CTRL_DM::RateLimiter_DM Rate Limiter with Up and Down Rate, Reset trig] *
 * @param Te        [Sample Time]                                           *
 * @param X0        [Init Val]                                              *
 * @param Reset     [Reset Trigger]                                         *
 * @param Up        [Rising Rate Value]                                     *
 * @param Down      [Falling Rate Value]                                    *
 * @param Value     [Value to Rate Limit]                                   *
 * @param Rated_Val [Rate limited output value]                             *
 ****************************************************************************/
void CTRL_DM::RateLimiter_DM(float Te,
                             float X0,
                             bool Reset,
                             float Up,
                             float Down,
                             float Value,
                             float *Rated_Val)
{
    if (Reset)
    {
        *Rated_Val = X0;
    }
    else
    {
        *Rated_Val = (std::max(std::min((Value - *Rated_Val), Up * Te), Down * Te)) + *Rated_Val;
    }
}

float CTRL_DM::Saturate_DM(float Value, float Min, float Max)
{
    float Sat_Val;
    Sat_Val = std::max(std::min(Value, Max), Min);
    return Sat_Val;
}

int CTRL_DM::Saturate_DM(int Value, int Min, int Max)
{
    int Sat_Val;
    Sat_Val = std::max(std::min(Value, Max), Min);
    return Sat_Val;
}

void CTRL_DM::FilteredDerivative_DM(float Te,
                                    float X0,
                                    bool Reset,
                                    float Tau,
                                    float Value,
                                    float *OldValue,
                                    float *Filtered_Derivated_Val)
{
    float OutputDeriv;
    if (Reset)
    {
        *Filtered_Derivated_Val = X0;
    }
    else
    {
        OutputDeriv             = (Value - *OldValue) / Te;
        *Filtered_Derivated_Val = (OutputDeriv * Te + *Filtered_Derivated_Val * Tau) / (Te + Tau);
    }
}

float CTRL_DM::Carto1D_DM(float *BreakPoint,
                          float *TableData,
                          float InputPoint,
                          uint16_t Array_Size)
{
    float Result;
    float a;
    float b;
    float f;

    if (InputPoint <= BreakPoint[0])
    {
        return TableData[0];
    }

    else if (InputPoint >= BreakPoint[Array_Size - 1])
    {
        return TableData[Array_Size - 1];
    }

    else
    {
        for (uint16_t i = 0; i < Array_Size; i++)
        {
            if (InputPoint < BreakPoint[i + 1])
            {
                if (InputPoint >= BreakPoint[i])
                {
                    b      = BreakPoint[i + 1] - BreakPoint[i];
                    a      = InputPoint - BreakPoint[i];
                    f      = a / b;
                    Result = (1 - f) * TableData[i] + f * TableData[i + 1];
                    // Result = (TableData[i] + TableData[i + 1])/2;

                    return Result;
                }
            }
        }
    }
    return 0;
}

float CTRL_DM::FastSin_DM(float Value)
{
    float sin;

    if (Value < -3.14159265)
    {
        while (Value < -3.14159265)
        {
            Value += 6.28318531;
        }
    }
    else if (Value > 3.14159265)
    {
        while (Value > 3.14159265)
        {
            Value -= 6.28318531;
        }
    }

    if (Value < 0)
    {
        sin = 1.27323954 * Value + .405284735 * Value * Value;

        if (sin < 0)
        {
            sin = .225 * (sin * -sin - sin) + sin;
        }
        else
        {
            sin = .225 * (sin * sin - sin) + sin;
        }

        return sin;
    }
    else
    {
        sin = 1.27323954 * Value - 0.405284735 * Value * Value;

        if (sin < 0)
        {
            sin = .225 * (sin * -sin - sin) + sin;
        }
        else
        {
            sin = .225 * (sin * sin - sin) + sin;
        }
        return sin;
    }
}

float CTRL_DM::FastCos_DM(float Value) // cos(x) = sin(x+pi/2)^^
{
    float cos = FastSin_DM(Value + 1.5708);
    return cos;
}

float ASIN_TABLE[81] = {-1.57079632679490,
                        -1.34672104149308,
                        -1.25323589750338,
                        -1.18103559399742,
                        -1.11976951499863,
                        -1.06543581651074,
                        -1.01598529381483,
                        -0.970202199928846,
                        -0.927295218001612,
                        -0.886715094999568,
                        -0.848062078981481,
                        -0.811034394287582,
                        -0.775397496610753,
                        -0.740964702203020,
                        -0.707584436725355,
                        -0.675131532937032,
                        -0.643501108793284,
                        -0.612604148048622,
                        -0.582364237868744,
                        -0.552715113096783,
                        -0.523598775598299,
                        -0.494964031716895,
                        -0.466765339047296,
                        -0.438961885609761,
                        -0.411516846067488,
                        -0.384396774495639,
                        -0.357571103645510,
                        -0.331011728089294,
                        -0.304692654015397,
                        -0.278589702391651,
                        -0.252680255142079,
                        -0.226943036178520,
                        -0.201357920790331,
                        -0.175905768163716,
                        -0.150568272776686,
                        -0.125327831168065,
                        -0.100167421161560,
                        -0.0750704910767165,
                        -0.0500208568057699,
                        -0.0250026048993611,
                        0,
                        0.0250026048993611,
                        0.0500208568057699,
                        0.0750704910767165,
                        0.100167421161560,
                        0.125327831168065,
                        0.150568272776686,
                        0.175905768163716,
                        0.201357920790331,
                        0.226943036178520,
                        0.252680255142079,
                        0.278589702391651,
                        0.304692654015397,
                        0.331011728089294,
                        0.357571103645510,
                        0.384396774495639,
                        0.411516846067488,
                        0.438961885609761,
                        0.466765339047296,
                        0.494964031716895,
                        0.523598775598299,
                        0.552715113096783,
                        0.582364237868744,
                        0.612604148048622,
                        0.643501108793284,
                        0.675131532937032,
                        0.707584436725355,
                        0.740964702203020,
                        0.775397496610753,
                        0.811034394287582,
                        0.848062078981481,
                        0.886715094999568,
                        0.927295218001612,
                        0.970202199928846,
                        1.01598529381483,
                        1.06543581651074,
                        1.11976951499863,
                        1.18103559399742,
                        1.25323589750338,
                        1.34672104149308,
                        1.57079632679490};

float ASIN_TABLE_ABSC[81] = {-1,
                             -0.975000000000000,
                             -0.950000000000000,
                             -0.925000000000000,
                             -0.900000000000000,
                             -0.875000000000000,
                             -0.850000000000000,
                             -0.825000000000000,
                             -0.800000000000000,
                             -0.775000000000000,
                             -0.750000000000000,
                             -0.725000000000000,
                             -0.700000000000000,
                             -0.675000000000000,
                             -0.650000000000000,
                             -0.625000000000000,
                             -0.600000000000000,
                             -0.575000000000000,
                             -0.550000000000000,
                             -0.525000000000000,
                             -0.500000000000000,
                             -0.475000000000000,
                             -0.450000000000000,
                             -0.425000000000000,
                             -0.400000000000000,
                             -0.375000000000000,
                             -0.350000000000000,
                             -0.325000000000000,
                             -0.300000000000000,
                             -0.275000000000000,
                             -0.250000000000000,
                             -0.225000000000000,
                             -0.200000000000000,
                             -0.175000000000000,
                             -0.150000000000000,
                             -0.125000000000000,
                             -0.100000000000000,
                             -0.0750000000000000,
                             -0.0499999999999999,
                             -0.0249999999999999,
                             0,
                             0.0249999999999999,
                             0.0499999999999999,
                             0.0750000000000000,
                             0.100000000000000,
                             0.125000000000000,
                             0.150000000000000,
                             0.175000000000000,
                             0.200000000000000,
                             0.225000000000000,
                             0.250000000000000,
                             0.275000000000000,
                             0.300000000000000,
                             0.325000000000000,
                             0.350000000000000,
                             0.375000000000000,
                             0.400000000000000,
                             0.425000000000000,
                             0.450000000000000,
                             0.475000000000000,
                             0.500000000000000,
                             0.525000000000000,
                             0.550000000000000,
                             0.575000000000000,
                             0.600000000000000,
                             0.625000000000000,
                             0.650000000000000,
                             0.675000000000000,
                             0.700000000000000,
                             0.725000000000000,
                             0.750000000000000,
                             0.775000000000000,
                             0.800000000000000,
                             0.825000000000000,
                             0.850000000000000,
                             0.875000000000000,
                             0.900000000000000,
                             0.925000000000000,
                             0.950000000000000,
                             0.975000000000000,
                             1};

float CTRL_DM::FastASin_DM(float Value)
{
    float ASIN_DM;

    ASIN_DM = Carto1D_DM(ASIN_TABLE_ABSC, ASIN_TABLE, Value, 81);

    return ASIN_DM;
}

void CTRL_DM::AddMatrix_DM(std::vector<std::vector<float>> &Mat1,
                           std::vector<std::vector<float>> &Mat2,
                           uint8_t Mat1Row,
                           uint8_t Mat1Col,
                           uint8_t Mat2Row,
                           uint8_t Mat2Col,
                           std::vector<std::vector<float>> &MatResult)
{
    // long t_now2 = micros();
    // Passer le résultat par référence cf déclaration longue sa mère...
    // std::vector<std::vector<float> > curMatrix;
    //
    // curMatrix.resize(Mat1Row);
    //
    // for (int i = 0; i < Mat1Row; i++)
    // {
    //         curMatrix[i].resize(Mat2Col);
    // }

    for (int i = 0; i < Mat1Row; i++)
    {
        for (int j = 0; j < Mat1Col; j++)
        {
            MatResult[i][j] = Mat1[i][j] + Mat2[i][j];
        }
    }

    // Serial.println(micros() - t_now2);
}

void CTRL_DM::AddMatrix_DM(std::vector<std::vector<float>> &Mat1,
                           std::vector<std::vector<float>> &Mat2,
                           std::vector<std::vector<float>> &Mat3,
                           uint8_t Mat1Row,
                           uint8_t Mat1Col,
                           uint8_t Mat2Row,
                           uint8_t Mat2Col,
                           uint8_t Mat3Row,
                           uint8_t Mat3Col,
                           std::vector<std::vector<float>> &MatResult)
{

    for (int i = 0; i < Mat1Row; i++)
    {
        for (int j = 0; j < Mat1Col; j++)
        {
            MatResult[i][j] = Mat1[i][j] + Mat2[i][j] + Mat3[i][j];
        }
    }
}

void CTRL_DM::SubstractMatrix_DM(std::vector<std::vector<float>> &Mat1,
                                 std::vector<std::vector<float>> &Mat2,
                                 uint8_t Mat1Row,
                                 uint8_t Mat1Col,
                                 uint8_t Mat2Row,
                                 uint8_t Mat2Col,
                                 std::vector<std::vector<float>> &MatResult)
{
    for (int i = 0; i < Mat1Row; i++)
    {
        for (int j = 0; j < Mat1Col; j++)
        {
            MatResult[i][j] = Mat1[i][j] - Mat2[i][j];
        }
    }
}

void CTRL_DM::ProdMatrix_DM(std::vector<std::vector<float>> &Mat1,
                            std::vector<std::vector<float>> &Mat2,
                            uint8_t Mat1Row,
                            uint8_t Mat1Col,
                            uint8_t Mat2Row,
                            uint8_t Mat2Col,
                            std::vector<std::vector<float>> &MatResult)
{
    for (int i = 0; i < Mat1Row; i++)
    {
        for (int j = 0; j < Mat2Col; j++)
        {
            MatResult[i][j] = 0;

            for (int k = 0; k < Mat1Col; k++)
            {
                MatResult[i][j] += Mat1[i][k] * Mat2[k][j];
            }
        }
    }
}

void CTRL_DM::ProdMatrix_DM(std::vector<std::vector<float>> &Mat1,
                            std::vector<std::vector<float>> &Mat2,
                            std::vector<std::vector<float>> &Mat3,
                            uint8_t Mat1Row,
                            uint8_t Mat1Col,
                            uint8_t Mat2Row,
                            uint8_t Mat2Col,
                            uint8_t Mat3Row,
                            uint8_t Mat3Col,
                            std::vector<std::vector<float>> &MatInter,
                            std::vector<std::vector<float>> &MatResult)
{
    ProdMatrix_DM(Mat1, Mat2, Mat1Row, Mat1Col, Mat2Row, Mat2Col, MatInter);
    ProdMatrix_DM(MatInter, Mat3, Mat1Row, Mat2Col, Mat3Row, Mat3Col, MatResult);
}

void CTRL_DM::TransposeMatrix_DM(std::vector<std::vector<float>> &Mat1,
                                 uint8_t Mat1Row,
                                 uint8_t Mat1Col,
                                 std::vector<std::vector<float>> &MatResult)
{
    for (int i = 0; i < Mat1Row; i++)
    {
        for (int j = 0; j < Mat1Col; j++)
        {
            MatResult[i][j] = Mat1[j][i];
        }
    }
}

float CTRL_DM::GetDetMatrix_3x3_DM(std::vector<std::vector<float>> &Mat1)
{
    float Det;

    Det = Mat1[0][0] * Mat1[1][1] * Mat1[2][2] - Mat1[2][0] * Mat1[1][1] * Mat1[0][2]
          + Mat1[0][1] * Mat1[1][2] * Mat1[2][0] - Mat1[2][1] * Mat1[1][2] * Mat1[0][0]
          + Mat1[0][2] * Mat1[1][0] * Mat1[2][1] - Mat1[2][2] * Mat1[1][0] * Mat1[0][1];

    return Det;
}

float CTRL_DM::GetDetMatrix_2x2_DM(std::vector<std::vector<float>> &Mat1)
{
    float Det;

    Det = Mat1[0][0] * Mat1[1][1] - Mat1[1][0] * Mat1[0][1];

    return Det;
}

void CTRL_DM::GetComatrix_3x3_DM(std::vector<std::vector<float>> &Mat1,
                                 std::vector<std::vector<float>> &MatResult)
{
    MatResult[0][0] = Mat1[1][1] * Mat1[2][2] - Mat1[2][1] * Mat1[1][2];
    MatResult[0][1] = Mat1[2][0] * Mat1[1][2] - Mat1[1][0] * Mat1[2][2];
    MatResult[0][2] = Mat1[1][0] * Mat1[2][1] - Mat1[2][0] * Mat1[1][1];

    MatResult[1][0] = Mat1[2][1] * Mat1[0][2] - Mat1[0][1] * Mat1[2][2];
    MatResult[1][1] = Mat1[0][0] * Mat1[2][2] - Mat1[2][0] * Mat1[0][2];
    MatResult[1][2] = Mat1[2][0] * Mat1[0][1] - Mat1[0][0] * Mat1[2][1];

    MatResult[2][0] = Mat1[0][1] * Mat1[1][2] - Mat1[1][1] * Mat1[0][2];
    MatResult[2][1] = Mat1[1][0] * Mat1[0][2] - Mat1[0][0] * Mat1[1][2];
    MatResult[2][2] = Mat1[0][0] * Mat1[1][1] - Mat1[1][0] * Mat1[0][1];
}

void CTRL_DM::GetComatrix_2x2_DM(std::vector<std::vector<float>> &Mat1,
                                 std::vector<std::vector<float>> &MatResult)
{
    MatResult[0][0] = Mat1[1][1];
    MatResult[0][1] = -Mat1[1][0];

    MatResult[1][0] = -Mat1[0][1];
    MatResult[1][1] = Mat1[0][0];
}

bool CTRL_DM::InvertMatrix_3x3_DM(std::vector<std::vector<float>> &Mat1,
                                  std::vector<std::vector<float>> &MatInter,
                                  std::vector<std::vector<float>> &MatResult)
{
    float curFactor = 1 / GetDetMatrix_3x3_DM(Mat1);
    if (curFactor > 1e16)
        return 0; // Non inversible matrix
    else
    {
        GetComatrix_3x3_DM(Mat1, MatInter);
        TransposeMatrix_DM(MatInter, 3, 3, MatResult);

        MatResult[0][0] *= curFactor;
        MatResult[0][1] *= curFactor;
        MatResult[0][2] *= curFactor;

        MatResult[1][0] *= curFactor;
        MatResult[1][1] *= curFactor;
        MatResult[1][2] *= curFactor;

        MatResult[2][0] *= curFactor;
        MatResult[2][1] *= curFactor;
        MatResult[2][2] *= curFactor;
        return 1;
    }
}

bool CTRL_DM::InvertMatrix_2x2_DM(std::vector<std::vector<float>> &Mat1,
                                  std::vector<std::vector<float>> &MatInter,
                                  std::vector<std::vector<float>> &MatResult)
{
    float curFactor = 1 / GetDetMatrix_2x2_DM(Mat1);
    if (curFactor > 1e16)
        return 0; // Non inversible matrix
    else
    {

        GetComatrix_2x2_DM(Mat1, MatInter);
        TransposeMatrix_DM(MatInter, 2, 2, MatResult);

        MatResult[0][0] *= curFactor;
        MatResult[0][1] *= curFactor;

        MatResult[1][0] *= curFactor;
        MatResult[1][1] *= curFactor;
        return 1;
    }
}

void CTRL_DM::PrintMatrix_DM(std::vector<std::vector<float>> &Mat1,
                             uint8_t Mat1Row,
                             uint8_t Mat1Col)

{
    for (int i = 0; i < Mat1Row; i++)
    {
        for (int j = 0; j < Mat1Col; j++)
        {
            Serial.print(Mat1[i][j]);
            if (j == Mat1Col - 1)
            {
                Serial.println();
            }
            else
            {
                Serial.print("\t");
            }
        }
    }
}

void CTRL_DM::Virtual_Quad(float Te,
                           float VertForce,
                           float DFX,
                           float DFY,
                           float TorqueOffsetX,
                           float TorqueOffsetY,
                           float Mass,
                           float Ix,
                           float Iy,
                           float VirtualLeverageDistanceX,
                           float VirtualLeverageDistanceY,
                           float *VirtualQuadValues)

{
    // Temporary values
    float RawTorqueX  = 0.0;
    float RawTorqueY  = 0.0;
    float ProcTorqueX = 0.0;
    float ProcTorqueY = 0.0;

    // Rotational model
    // X axis:
    RawTorqueX = VirtualLeverageDistanceX * DFX;
    LPF1_DM(Te, 0.0, 0, 0.1, RawTorqueX, &DelayedTorqueX);
    ProcTorqueX = DelayedTorqueX + TorqueOffsetX;
    ProcTorqueX /= Ix;
    Integrator_DM(Te, 0.0, 0, -5.0, 5.0, ProcTorqueX, &AgvX);
    Integrator_DM(Te, 0.0, 0, -50, 50, AgvX, &VirtualQuadValues[3]);
    VirtualQuadValues[4] = AgvX;
    VirtualQuadValues[5] = ProcTorqueX;

    // Y axis:
    RawTorqueY = VirtualLeverageDistanceY * DFY;
    LPF1_DM(Te, 0.0, 0, 0.1, RawTorqueY, &DelayedTorqueY);
    ProcTorqueY = DelayedTorqueY + TorqueOffsetY;
    ProcTorqueY /= Iy;
    Integrator_DM(Te, 0.0, 0, -5.0, 5.0, ProcTorqueY, &AgvY);
    Integrator_DM(Te, 0.0, 0, -50, 50, AgvY, &VirtualQuadValues[6]);
    VirtualQuadValues[7] = AgvY;
    VirtualQuadValues[8] = ProcTorqueY;

    // Translational model
    // Z axis
    LPF1_DM(Te, 0.0, 0, 0.1, VertForce, &DelayedForceZ);
    VirtualQuadValues[2] = DelayedForceZ / Mass - 9.81;
    Integrator_DM(Te, 0.0, 0, -10, 10, VirtualQuadValues[2], &VirtualQuadValues[1]);
    Integrator_DM(Te, 0.0, 0, 0, 50, VirtualQuadValues[1], &VirtualQuadValues[0]);
}

float CTRL_DM::TunePotVal_DM(uint16_t PotVal,
                             uint16_t PotValMin,
                             uint16_t PotValMax,
                             float ValueMin,
                             float ValueMax)
{
    float PotInput[2]  = {(float)PotValMin, (float)PotValMax};
    float ValuesOut[2] = {ValueMin, ValueMax};

    return Carto1D_DM(PotInput, ValuesOut, (float)PotVal, 2);
}

float CTRL_DM::SumVector_DM(float *Vector, uint16_t Size)
{
    float Sum = 0.0;
    for (int i = 0; i < Size; i++)
    {
        Sum += Vector[i];
    }
    return Sum;
}

void CTRL_DM::Rotation3D_XY_DM(std::vector<std::vector<float>> &Vector,
                               float Angle1,
                               float Angle2,
                               std::vector<std::vector<float>> &MAT_ROTX,
                               std::vector<std::vector<float>> &MAT_ROTY,
                               std::vector<std::vector<float>> &MAT_TEMP,
                               std::vector<std::vector<float>> &VECT_RESULT)
{
    // Construct matrixes
    MAT_ROTX[0][0] = 1;
    MAT_ROTX[1][0] = 0;
    MAT_ROTX[2][0] = 0;
    MAT_ROTX[0][1] = 0;
    MAT_ROTX[1][1] = FastCos_DM(Angle1);
    MAT_ROTX[2][1] = -FastSin_DM(Angle1);
    MAT_ROTX[0][2] = 0;
    MAT_ROTX[1][2] = FastSin_DM(Angle1);
    MAT_ROTX[2][2] = FastCos_DM(Angle1);

    MAT_ROTY[0][0] = FastCos_DM(Angle2);
    MAT_ROTY[1][0] = 0;
    MAT_ROTY[2][0] = FastSin_DM(Angle2);
    MAT_ROTY[0][1] = 0;
    MAT_ROTY[1][1] = 1;
    MAT_ROTY[2][1] = 0;
    MAT_ROTY[0][2] = -FastSin_DM(Angle2);
    MAT_ROTY[1][2] = 0;
    MAT_ROTY[2][2] = FastCos_DM(Angle2);

    // Compute XY 3D rotation matrix
    ProdMatrix_DM(MAT_ROTX, MAT_ROTY, 3, 3, 3, 3, MAT_TEMP);

    // Rotate vector
    ProdMatrix_DM(MAT_TEMP, Vector, 3, 3, 3, 1, VECT_RESULT);
}

CTRL_DM::BIQUAD::BIQUAD(float Te,
                        uint8_t Order,
                        float *TableGains,
                        float TableNum[][3],
                        float TableDen[][3]) // Send vectors!!
{
    curTe             = Te;
    curOrder          = Order;
    UnitFiltersNumber = Order / 2;

    for (uint8_t i = 0; i < UnitFiltersNumber; i++)
    {
        // Update Num and den
        for (uint8_t j = 0; j < 3; j++)
        {
            curTableNum[i][j] = TableNum[i][j];
            curTableDen[i][j] = TableDen[i][j];
        }

        // Update gains
        curTableGains[i] = TableGains[i];

        // Compute invert of a0
        curTableDenInvert[i] = 1 / TableDen[i][0];
    }
}

float CTRL_DM::BIQUAD::Filter(float Value)
{
    float tempVal = Value;
    for (uint8_t i = 0; i < UnitFiltersNumber; i++)
    {
        tempVal = UnitFilter(tempVal, i);
    }
    return tempVal;
}

float CTRL_DM::BIQUAD::UnitFilter(float TempValue, uint8_t idxFilter)
{
    float tempVal = TempValue;
    float tempMem = 0.0;

    // First part
    tempVal *= curTableGains[idxFilter];
    tempVal -= Memory1[idxFilter][0] * curTableDen[idxFilter][1];
    tempVal -= Memory1[idxFilter][1] * curTableDen[idxFilter][2];
    tempVal *= curTableDenInvert[idxFilter];

    tempMem = tempVal;

    // Second part
    tempVal *= curTableNum[idxFilter][0];
    tempVal += curTableNum[idxFilter][1] * Memory1[idxFilter][0];
    tempVal += curTableNum[idxFilter][2] * Memory1[idxFilter][1];

    // Memories
    Memory1[idxFilter][1] = Memory1[idxFilter][0];
    Memory1[idxFilter][0] = tempMem;

    return tempVal;
}