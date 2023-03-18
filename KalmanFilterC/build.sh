#!/bin/bash

###    Compile a C program named KF_run    ###
gcc -o KF main.c kf.c operations.c CControl/Sources/LinearAlgebra/mul.c CControl/Sources/Miscellaneous/print.c CControl/Sources/LinearAlgebra/tran.c CControl/Sources/LinearAlgebra/inv.c CControl/Sources/LinearAlgebra/lup.c