#ifndef CTRL_DM_hpp
#define CTRL_DM_hpp
#include "Arduino.h"
#include "vector"
#include <algorithm>
#include <cmath>

/////////////
// CTRL DM //
/////////////

// Functions used for control purpose

class CTRL_DM
{
public:
    void Integrator_DM(float Te,
                       float X0,
                       bool Reset,
                       float Min,
                       float Max,
                       float Value,
                       float *Integ_Val);
    void LPF1_DM(float Te, float X0, bool Reset, float Tau, float Value, float *Filtered_Val);
    void RateLimiter_DM(float Te,
                        float X0,
                        bool Reset,
                        float Up,
                        float Down,
                        float Value,
                        float *Rated_Val);
    float Saturate_DM(float Value, float Min, float Max);
    int Saturate_DM(int Value, int Min, int Max);
    void FilteredDerivative_DM(float Te,
                               float X0,
                               bool Reset,
                               float Tau,
                               float Value,
                               float *OldValue,
                               float *Filtered_Derivated_Val);
    void LPF2_DM(float Te,
                 float X0,
                 bool Reset,
                 float Tau1,
                 float Tau2,
                 float Value,
                 float *Filtered_Val1,
                 float *Filtered_Val2);
    void KALMAN_DM();
    float Carto1D_DM(float *BreakPoint, float *TableData, float InputPoint, uint16_t Array_Size);
    float FastSin_DM(float Value);
    float FastCos_DM(float Value);
    float FastASin_DM(float Value);
    float TunePotVal_DM(uint16_t PotVal,
                        uint16_t PotValMin,
                        uint16_t PotValMax,
                        float ValueMin,
                        float ValueMax);
    float SumVector_DM(float *Vector, uint16_t Size);

    void ProdMatrix_DM(std::vector<std::vector<float>> &Mat1,
                       std::vector<std::vector<float>> &Mat2,
                       uint8_t Mat1Row,
                       uint8_t Mat1Col,
                       uint8_t Mat2Row,
                       uint8_t Mat2Col,
                       std::vector<std::vector<float>> &MatResult);
    void ProdMatrix_DM(std::vector<std::vector<float>> &Mat1,
                       std::vector<std::vector<float>> &Mat2,
                       std::vector<std::vector<float>> &Mat3,
                       uint8_t Mat1Row,
                       uint8_t Mat1Col,
                       uint8_t Mat2Row,
                       uint8_t Mat2Col,
                       uint8_t Mat3Row,
                       uint8_t Mat3Col,
                       std::vector<std::vector<float>> &MatInter,
                       std::vector<std::vector<float>> &MatResult);
    void AddMatrix_DM(std::vector<std::vector<float>> &Mat1,
                      std::vector<std::vector<float>> &Mat2,
                      uint8_t Mat1Row,
                      uint8_t Mat1Col,
                      uint8_t Mat2Row,
                      uint8_t Mat2Col,
                      std::vector<std::vector<float>> &MatResult);
    void AddMatrix_DM(std::vector<std::vector<float>> &Mat1,
                      std::vector<std::vector<float>> &Mat2,
                      std::vector<std::vector<float>> &Mat3,
                      uint8_t Mat1Row,
                      uint8_t Mat1Col,
                      uint8_t Mat2Row,
                      uint8_t Mat2Col,
                      uint8_t Mat3Row,
                      uint8_t Mat3Col,
                      std::vector<std::vector<float>> &MatResult);
    void SubstractMatrix_DM(std::vector<std::vector<float>> &Mat1,
                            std::vector<std::vector<float>> &Mat2,
                            uint8_t Mat1Row,
                            uint8_t Mat1Col,
                            uint8_t Mat2Row,
                            uint8_t Mat2Col,
                            std::vector<std::vector<float>> &MatResult);
    void TransposeMatrix_DM(std::vector<std::vector<float>> &Mat1,
                            uint8_t Mat1Row,
                            uint8_t Mat1Col,
                            std::vector<std::vector<float>> &MatResult);
    bool InvertMatrix_3x3_DM(std::vector<std::vector<float>> &Mat1,
                             std::vector<std::vector<float>> &MatInter,
                             std::vector<std::vector<float>> &MatResult);
    bool InvertMatrix_2x2_DM(std::vector<std::vector<float>> &Mat1,
                             std::vector<std::vector<float>> &MatInter,
                             std::vector<std::vector<float>> &MatResult);
    float GetDetMatrix_3x3_DM(std::vector<std::vector<float>> &Mat1);
    float GetDetMatrix_2x2_DM(std::vector<std::vector<float>> &Mat1);
    void GetComatrix_3x3_DM(std::vector<std::vector<float>> &Mat1,
                            std::vector<std::vector<float>> &MatResult);
    void GetComatrix_2x2_DM(std::vector<std::vector<float>> &Mat1,
                            std::vector<std::vector<float>> &MatResult);
    void PrintMatrix_DM(std::vector<std::vector<float>> &Mat1, uint8_t Mat1Row, uint8_t Mat1Col);

    void Rotation3D_XY_DM(std::vector<std::vector<float>> &Vector,
                          float Angle1,
                          float Angle2,
                          std::vector<std::vector<float>> &MAT_ROTX,
                          std::vector<std::vector<float>> &MAT_ROTY,
                          std::vector<std::vector<float>> &MAT_TEMP,
                          std::vector<std::vector<float>>
                                  &VECT_RESULT); // Faster version for aZAbs cf no use of Z angle

    /******************************************
     * Virtual quad for control validation    *
     * @param VertForce         [description] *
     * @param DFX               [description] *
     * @param DFY               [description] *
     * @param VirtualQuadValues [description] *
     ******************************************/
    // Translational
    float DelayedForceZ = 0.0;

    // Rotational
    float AgvX           = 0.0;
    float AgvY           = 0.0;
    float DelayedTorqueX = 0.0;
    float DelayedTorqueY = 0.0;

    void Virtual_Quad(float Te,
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
                      float *VirtualQuadValues);

    struct BIQUAD // 6 stages max, meaning 12 degrees max, useless to do more! Can
                  // be used for low pass, passband...
    {

    public:
        BIQUAD(float Te,
               uint8_t Order,
               float *TableGains,
               float TableNum[][3],
               float TableDen[][3]); // Send vectors!!

        float Filter(float Value);
        float UnitFilter(float TempValue, uint8_t idxFilter);

    private:
        float curTe               = 0.01;
        uint8_t curOrder          = 2;
        uint8_t UnitFiltersNumber = 1;
        float Memory1[6][2];
        // float Memory2[6][2];

        float curTableGains[6];
        float curTableNum[6][3];    // b0 + b1.z-1 + b2.z-2
        float curTableDen[6][3];    // 1 + a1.z-1 + a2.z-2, a0 = 1
        float curTableDenInvert[6]; // Faster multiply for gain
    };

private:
};

#endif