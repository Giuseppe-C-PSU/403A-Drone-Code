#ifndef EKF_H
#define EKF_H

// You can change the dimensions here
#define STATE_DIM 3  // Number of states
#define MEAS_DIM 3   // Number of measurements

class EKF {
public:
    EKF();  // Constructor to initialize the EKF

    void predict();  // Prediction step
    void update(float z[MEAS_DIM]);  // Update step

    void printState();  // Utility function to print the state

private:
    // State Vector
    float x_[STATE_DIM];

    // State Covariance Matrix (P_)
    float P_[STATE_DIM][STATE_DIM];

    // State Transition Matrix (F_)
    float F_[STATE_DIM][STATE_DIM];

    // Process Noise Covariance Matrix (Q_)
    float Q_[STATE_DIM][STATE_DIM];

    // Measurement Matrix (H_)
    float H_[MEAS_DIM][STATE_DIM];

    // Measurement Noise Covariance Matrix (R_)
    float R_[MEAS_DIM][MEAS_DIM];

    // Kalman Gain Matrix (K_)
    float K_[STATE_DIM][MEAS_DIM];

    // Matrix Operations
    void matrixVectorMultiply(float mat[STATE_DIM][STATE_DIM], float vec[STATE_DIM], float result[STATE_DIM]);
    void matrixMultiply(float mat1[STATE_DIM][STATE_DIM], float mat2[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]);
    void matrixAdd(float mat1[STATE_DIM][STATE_DIM], float mat2[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]);
    void matrixSubtract(float mat1[STATE_DIM][STATE_DIM], float mat2[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]);
    void matrixTranspose(float mat[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]);
    bool matrixInverse(float mat[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]);
};

#endif  // EKF_H
