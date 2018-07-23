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
#include <octave/interpreter.h>

int
main (void)
{
  string_vector argv (2);
  argv(0) = "embedded";
  argv(1) = "-q";

  octave_main (2, argv.c_str_vec (), 1);

  octave_idx_type n = 2;
  octave_value_list in;

  for (octave_idx_type i = 0; i < n; i++)
    in(i) = octave_value (5 * (i + 2));

  octave_value_list out = feval("gcd", in, 1);

  if (! error_state && out.length () > 0)
    std::cout << "GCD of ["
              << in(0).int_value ()
              << ", "
              << in(1).int_value ()
              << "] is " << out(0).int_value ()
              << std::endl;
  else
    std::cout << "invalid\n";

    // LQR Test
    // TODOS: make sure i can call lqr
    n = 2;
    int m = 1;

    Matrix a = Matrix (n, n);
    for (octave_idx_type i = 0; i < n; i++)
        for (octave_idx_type j = 0; j < n; j++)
            a(i, j) = 1;

    Matrix b = Matrix (n, m);
    for (octave_idx_type i = 0; i < n; i++)
        for (octave_idx_type j = 0; j < m; j++)
            b(i, j) = 1;

    Matrix q = Matrix (n, n);
    for (octave_idx_type i = 0; i < n; i++)
        for (octave_idx_type j = 0; j < n; j++)
            q(i, j) = 1;

    Matrix r = Matrix (m, m);
    for (octave_idx_type i = 0; i < m; i++)
        for (octave_idx_type j = 0; j < m; j++)
            q(i, j) = 1;

    octave_value_list lqrin;
    lqrin(0) = a;
    lqrin(1) = b;
    lqrin(2) = q;
    lqrin(3) = r;

    std::cout << lqrin(0).matrix_value() << std::endl;


    std::cout << "Trying feval" << std::endl;

    //feval("pkg", ovl("list"), 0);

    //feval("pkg", ovl("load", "control"), 0);

    octave_value_list f_arg, f_ret;
	f_arg(0) = octave_value(-1);
	f_ret = feval("acos", f_arg);
	Matrix unis (f_ret(0).matrix_value ());
	std::cout << unis;

    octave_value_list is = octave_value('s');
    //feval("tf", is, 1);

    //octave_value_list os = feval("acos");

    //octave_value_list lqrout = feval ("lqr", lqrin, 3);

    std::cout << "Success!" << std::endl;
    clean_up_and_exit(0);
}

