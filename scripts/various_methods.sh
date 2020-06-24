#!/usr/bin/env bash

source ~/.bashrc

for f in "DIRECT" "G_DIRECT" "DIRECT_L" "G_DIRECT_L" "CRS" "STOGO" "ISRES" "CCSA" "SLSQP" "L_BFGS" "TN" "SL_VM" "COBYLA" "BOBYQA" "NEWUOA" "PRAXIS" "NelderMeadSimplex" "Sbplx"
do
    roseus alg/$f.l &
done
