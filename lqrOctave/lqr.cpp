// Author Akash Patel (apatel435@gatech.edu)
// Based on Octave implentation of lqr by schlagenhauf (https://github.com/schlagenhauf)
// https://github.com/schlagenhauf/lqr_solve/blob/master/lqr_solve.cpp
//
// Purpose: Helper source code file for determining optimal gain matrix using lqr method (discrete time)
// LQR Solver for Discrete Time Infinite Horizon Problems
// Returns true if successful and outputs calculated gain matrix in input
// pointer



#include <iostream>
#include <octave/oct.h>
#include <octave/octave.h>
#include <octave/parse.h>
#include <octave/toplev.h>
//#include <octave/interpreter.h>

int main () {

    // Initialize Octave Interpreter

    string_vector argv (2);
    argv(0) = "embedded";
    argv(1) = "-q";
    octave_main (2, argv.c_str_vec (), 1);

    // LQR Test
    int n = 4;
    int m = 1;

    Matrix a = Matrix (n, n);
    a(0, 0) = 0;
    a(0, 1) = 0;
    a(0, 2) = 1;
    a(0, 3) = 0;
    a(1, 0) = 0;
    a(1, 1) = 0;
    a(1, 2) = 0;
    a(1, 3) = 1;
    a(2, 0) = 17.78;
    a(2, 1) = 0;
    a(2, 2) = -0.00858;
    a(2, 3) = 0.00858;
    a(3, 0) = 47.8688;
    a(3, 1) = 0;
    a(3, 2) = 0.0288;
    a(3, 3) = -0.0288;

    Matrix b = Matrix (n, m);
    b(0, 0) = 0;
    b(1, 0) = 0;
    b(2, 0) = -0.0858;
    b(3, 0) = 0.288;

    Matrix q = Matrix (n, n);
    for (octave_idx_type i = 0; i < n; i++)
        for (octave_idx_type j = 0; j < n; j++)
            q(i, j) = 1;

    Matrix r = Matrix (m, m);
    for (octave_idx_type i = 0; i < m; i++)
        for (octave_idx_type j = 0; j < m; j++)
            r(i, j) = 1;

    octave_value_list lqrin;
    lqrin(0) = a;
    lqrin(1) = b;
    lqrin(2) = q;
    lqrin(3) = r;

    std::cout << lqrin(0).matrix_value() << std::endl;
    std::cout << lqrin(1).matrix_value() << std::endl;
    std::cout << lqrin(2).matrix_value() << std::endl;
    std::cout << lqrin(3).matrix_value() << std::endl;

    std::cout << "Trying feval" << std::endl;

    feval("pkg", ovl("load", "control"), 0);

    octave_value_list lqrout = feval ("lqr", lqrin, 3);

    std::cout << lqrout(0).matrix_value() << std::endl;
    std::cout << lqrout(1).matrix_value() << std::endl;
    std::cout << lqrout(2).matrix_value() << std::endl;

    std::cout << "Success!" << std::endl;
    clean_up_and_exit(0);
}

