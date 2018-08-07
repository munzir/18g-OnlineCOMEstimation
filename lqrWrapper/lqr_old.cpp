// Author Akash Patel (apatel435@gatech.edu)
// Based on Octave implentation of lqr by schlagenhauf (https://github.com/schlagenhauf)
// https://github.com/schlagenhauf/lqr_solve/blob/master/lqr_solve.cpp
//
// Purpose: Helper source code file for determining optimal gain matrix using lqr method (discrete time)
// LQR Solver for Discrete Time Infinite Horizon Problems
// Returns true if successful and outputs calculated gain matrix in input
// pointer

// Includes
#include <dart/dart.hpp>
#include <octave/oct.h>

#include "lqr.hpp"

// Namespaces
using namespace std;

// Function
// // Main method
int main () {

    cout << "Hello Octave World!\n";

    system("../lqr");

    cout << "Goodbye Octave World!\n";
    return 0;
}

// // LQR method (without n)
bool lqr(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd q, Eigen::MatrixXd r, Eigen::MatrixXd *k) {
    Eigen::MatrixXd n = Eigen::MatrixXd::Zero(a.rows(), b.cols());

    return lqr(a, b, q, r, n, k);
}

// // LQR Method (full)
bool lqr(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd q, Eigen::MatrixXd r, Eigen::MatrixXd n, Eigen::MatrixXd *k) {
    // Check dimensionality of input matrices

    // Check a
    if (a.rows() != a.cols()) {
        cout << "LQR Error: First (a) matrix must be square!" << endl;
        return false;
    }

    // Check b
    if (b.rows() != a.cols()) {
        cout << "LQR Error: First (a) and second (b) matrix must be conformal!" << endl;
        return false;
    }

    // Check q
    if (q.rows() != q.cols() || q.rows() != a.cols()) {
        cout << "LQR Error: Third (q) matrix must be square and conformal with the first (a) matrix!" << endl;
        return false;
    }

    // Check r
    if (r.rows() != r.cols() || r.cols() != b.cols()) {
        cout << "LQR Error: Fourth (r) matrix must be square and conformal with column dimension of second (b) matrix!" << endl;
        return false;
    }

    // Check n
    if (n.rows() != b.rows() || n.cols() != b.cols()) {
        cout << "LQR Error: Fifth (n) matrix must be identically dimensioned with second (b) matrix!" << endl;
        return false;
    }

    // Precompute as much as possible
    Eigen::MatrixXd bT = b.transpose();
    Eigen::MatrixXd aCal = a - b * r.inverse() * n.transpose();
    Eigen::MatrixXd aCalT = aCal.transpose();
    Eigen::MatrixXd qCal = q - n * r.inverse() * n.transpose();

    // Initialize p with q
    Eigen::MatrixXd p = q;

    // Iterate until p converges
    int numIterations = 0;
    Eigen::MatrixXd pOld = p;

    double eps = 1e-15;
    while (true) {
        numIterations++;
        // Compute new p
        p = aCalT * p * aCal - aCalT * p * b * (r + bT * p * b).inverse() * bT * p * aCal + qCal;

        // Update delta
        Eigen::MatrixXd delta = p - pOld;
        if (abs(delta.maxCoeff()) < eps) {
            //cout << "LQR: Number of iterations for convergence: " << numIterations << endl;
            break;
        }

        pOld = p;
    }

    // Compute k from p
    *k = (r + bT * p * b).inverse() * (bT * p * a + n.transpose());

    return true;
}
