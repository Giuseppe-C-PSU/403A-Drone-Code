#include "EKF.h"
#include <Arduino.h>

// Constructor to initialize the EKF
EKF::EKF() {
    // Initialize the state vector
    for (int i = 0; i < STATE_DIM; i++) {
        x_[i] = 0;
    }

    // Initialize the state covariance matrix (P_)
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            P_[i][j] = (i == j) ? 1 : 0;  // Identity matrix
        }
    }

    // Initialize the state transition matrix (F_)
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            F_[i][j] = (i == j) ? 1 : 0;  // Identity matrix
        }
    }

    // Initialize the process noise covariance matrix (Q_)
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            Q_[i][j] = (i == j) ? 0.1 : 0;  // Diagonal with small values
        }
    }

    // Initialize the measurement matrix (H_)
    for (int i = 0; i < MEAS_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            H_[i][j] = (i == j) ? 1 : 0;  // Identity matrix
        }
    }

    // Initialize the measurement noise covariance matrix (R_)
    for (int i = 0; i < MEAS_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            R_[i][j] = (i == j) ? 0.01 : 0;  // Diagonal with small values
        }
    }

    // Initialize the Kalman gain matrix (K_)
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            K_[i][j] = 0;
        }
    }
}

// Prediction step
void EKF::predict() {
    float x_pred[STATE_DIM];
    matrixVectorMultiply(F_, x_, x_pred);

    for (int i = 0; i < STATE_DIM; i++) {
        x_[i] = x_pred[i];
    }

    float P_pred[STATE_DIM][STATE_DIM];
    matrixMultiply(F_, P_, P_pred);

    float temp[STATE_DIM][STATE_DIM];
    matrixMultiply(P_pred, F_, temp);
    matrixAdd(temp, Q_, P_);
}

// Update step
void EKF::update(float z[MEAS_DIM]) {
    float y[MEAS_DIM]; // Innovation
    for (int i = 0; i < MEAS_DIM; i++) {
        y[i] = z[i] - (H_[i][0] * x_[0] + H_[i][1] * x_[1] + H_[i][2] * x_[2]);
    }

    float H_P[MEAS_DIM][STATE_DIM];
    for (int i = 0; i < MEAS_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            H_P[i][j] = H_[i][0] * P_[0][j] + H_[i][1] * P_[1][j] + H_[i][2] * P_[2][j];
        }
    }

    float S[MEAS_DIM][MEAS_DIM];
    matrixMultiply(H_P, H_, S);
    float S_inv[MEAS_DIM][MEAS_DIM];
    if (!matrixInverse(S, S_inv)) {
        return;
    }

    matrixMultiply(P_, H_, K_);
    float temp_K[STATE_DIM][MEAS_DIM];
    matrixMultiply(P_, H_, temp_K);
    matrixMultiply(temp_K, S_inv, K_);

    for (int i = 0; i < STATE_DIM; i++) {
        x_[i] += K_[i][0] * y[0] + K_[i][1] * y[1] + K_[i][2] * y[2];
    }

    float I[STATE_DIM][STATE_DIM] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    float K_H[STATE_DIM][STATE_DIM];
    matrixMultiply(K_, H_, K_H);
    matrixSubtract(I, K_H, P_);
}

// Print current state
void EKF::printState() {
    for (int i = 0; i < STATE_DIM; i++) {
        Serial.print(x_[i]);
        Serial.print(" ");
    }
    Serial.println();
}

// Matrix Operations
void EKF::matrixVectorMultiply(float mat[STATE_DIM][STATE_DIM], float vec[STATE_DIM], float result[STATE_DIM]) {
    for (int i = 0; i < STATE_DIM; i++) {
        result[i] = 0;
        for (int j = 0; j < STATE_DIM; j++) {
            result[i] += mat[i][j] * vec[j];
        }
    }
}

void EKF::matrixMultiply(float mat1[STATE_DIM][STATE_DIM], float mat2[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]) {
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            result[i][j] = 0;
            for (int k = 0; k < STATE_DIM; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void EKF::matrixAdd(float mat1[STATE_DIM][STATE_DIM], float mat2[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]) {
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            result[i][j] = mat1[i][j] + mat2[i][j];
        }
    }
}

void EKF::matrixSubtract(float mat1[STATE_DIM][STATE_DIM], float mat2[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]) {
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            result[i][j] = mat1[i][j] - mat2[i][j];
        }
    }
}

void EKF::matrixTranspose(float mat[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]) {
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            result[j][i] = mat[i][j];
        }
    }
}

bool EKF::matrixInverse(float mat[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]) {
    // Simple 3x3 matrix inversion
    // Can be replaced with a more robust method if needed
    float temp[STATE_DIM][STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            temp[i][j] = mat[i][j];
        }
    }

    for (int i = 0; i < STATE_DIM; i++) {
        float diag = temp[i][i];
        if (diag == 0) return false;
        for (int j = 0; j < STATE_DIM; j++) {
            temp[i][j] /= diag;
            result[i][j] = temp[i][j];
        }
        for (int j = 0; j < STATE_DIM; j++) {
            if (j != i) {
                float factor = temp[j][i];
                for (int k = 0; k < STATE_DIM; k++) {
                    temp[j][k] -= factor * temp[i][k];
                    result[j][k] -= factor * result[i][k];
                }
            }
        }
    }
    return true;
}
