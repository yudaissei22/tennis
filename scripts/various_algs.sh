#!/usr/bin/env bash

source ~/.bashrc

#for f in "DIRECT" "G_DIRECT" "DIRECT_L" "G_DIRECT_L" "CRS" "STOGO" "ISRES" "CCSA" "SLSQP" "L_BFGS" "TN" "SL_VM" "COBYLA" "BOBYQA" "NEWUOA" "PRAXIS" "NelderMeadSimplex" "Sbplx"
# for f in 0 10 20 30 40 50 60 70 80 90 100
for f in 80.0 80.1 80.2 80.3 80.4 80.5 80.6 80.7 80.8 80.9 81.0
do
    roseus alg/CCSA_$f\_$f.l &
done
