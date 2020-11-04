#!/usr/bin/env bash

in_dir="/userdir/logs/motion-planning"
opt_list=`ls ${in_dir} | grep Nov`
for f in ${opt_list}
do
  margin=`echo $f | sed 's/^.*sp-\([0-9\.]*\)-.*$/\1/'`
  alg_str=`echo $f | sed 's/^.*speed_\([a-zA-Z\-]*\)_.*$/\1/'`
  obj_str=`cat ${in_dir}/$f/maximize-speed_obj.dat | tail -n 1`
  eq_str=`cat ${in_dir}/$f/maximize-speed_eq.dat | tail -n 1`
  ieq_str=`cat ${in_dir}/$f/maximize-speed_ieq.dat | tail -n 1`
  obj_str_length=`echo $obj_str | awk '{print NF}'`
  eq_str_length=`echo $eq_str | awk '{print NF}'`
  ieq_str_length=`echo $ieq_str | awk '{print NF}'`
#  echo $margin
#  echo $obj_str_length
#  echo $eq_str_length
#  echo $ieq_str_length
  printf "alg: %19s, margin: %3f" $alg_str $margin
  printf "%15f" $obj_str
  printf "%15f" `echo $eq_str| awk -v len="$eq_str_length" \
'BEGIN{sum = 0;} { for (i = 1; i <= len; i++) { sum += $i * $i; } } END{print sum;}'`
  printf "%15f" `echo $ieq_str| awk -v len="$ieq_str_length" \
'BEGIN{sum = 0;} { for (i = 1; i <= len; i++) { if ($i > 0) {sum += $i;} } } END{print sum;}'`
  echo ""
done
#cat ${in_dir}/Feb-22-16-37-05-2019_forehand_maximize-speed_CCSA_48.0h_M-14_N-5_x-max-1.0_x-hit-0.5_maxvel-1_minjerk-0.0005_delta-1.7e-04_eqthre-1.0e-08_ftol-1.0e-15_xtol-1.0e-10_interval-20_sp-0-0-0-100-50_root-joint_modify-ec
