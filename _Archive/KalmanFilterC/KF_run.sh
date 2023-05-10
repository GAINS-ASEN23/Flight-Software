#!/bin/bash

###    Compile a C program named KF_run    ###
gcc -g -o KF_run KF_accel.c CControl/Sources/LinearAlgebra/mul.c CControl/Sources/Miscellaneous/print.c CControl/Sources/LinearAlgebra/tran.c CControl/Sources/LinearAlgebra/inv.c CControl/Sources/LinearAlgebra/lup.c

###    Run the compiled script    ###
./KF_run