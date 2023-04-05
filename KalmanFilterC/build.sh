#!/bin/bash

###    Compile a C program named KF_run    ###
g++ -g -std=c++11 -o KF main.cpp kf.cpp operations.cpp CControl/Sources/LinearAlgebra/mul.c CControl/Sources/Miscellaneous/print.c CControl/Sources/LinearAlgebra/tran.c CControl/Sources/LinearAlgebra/inv.c CControl/Sources/LinearAlgebra/lup.c
