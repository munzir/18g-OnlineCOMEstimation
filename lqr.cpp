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
#include <octave/octave.h>
#include <octave/parse.h>
#include <octave/toplev.h>
//#include <octave/interpreter.h>

#include "lqr.hpp"

// Namespaces
using namespace std;

// Function
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

    // Start octave interpreter to use lqr from
    string_vector argv(2);
    argv(0) = "embedded";
    argv(1) = "-q";
    octave_main(2, argv.c_str_vec(), 1);

    // Fill in the eigen matrices to octave matrices
    Matrix ao = Matrix(a.rows(), a.cols());
    Matrix bo = Matrix(b.rows(), b.cols());
    Matrix qo = Matrix(q.rows(), q.cols());
    Matrix ro = Matrix(r.rows(), r.cols());
    Matrix no = Matrix(n.rows(), n.cols());

    for (int i = 0; i < ao.rows(); i++) {
        for (int j = 0; j < ao.cols(); j++) {
            ao(i, j) = a(i, j);
        }
    }

    for (int i = 0; i < bo.rows(); i++) {
        for (int j = 0; j < bo.cols(); j++) {
            bo(i, j) = b(i, j);
        }
    }

    for (int i = 0; i < qo.rows(); i++) {
        for (int j = 0; j < qo.cols(); j++) {
            qo(i, j) = q(i, j);
        }
    }

    for (int i = 0; i < ro.rows(); i++) {
        for (int j = 0; j < ro.cols(); j++) {
            ro(i, j) = r(i, j);
        }
    }

    for (int i = 0; i < no.rows(); i++) {
        for (int j = 0; j < no.cols(); j++) {
            no(i, j) = n(i, j);
        }
    }

    octave_value_list lqrin;
    lqrin(0) = ao;
    lqrin(1) = bo;
    lqrin(2) = qo;
    lqrin(3) = ro;
    lqrin(4) = no;

    // Load control module
    feval("pkg", ovl("load", "control"), 0);
    // Run lqr method
    octave_value_list lqrout = feval("lqr", lqrin, 3);

    Matrix ko = lqrout(0).matrix_value();
    Eigen::MatrixXd gains(ko.rows(), ko.cols());

    for (int i = 0; i < ko.rows(); i++) {
        for (int j = 0; j < ko.cols(); j++) {
            gains(i, j) = ko(i, j);
        }
    }

    *k = gains;

    return true;
}
