#!/usr/bin/env bash

echo "proper filename filtering is needed so far"
file="/tmp/"`ls /tmp/ | grep Feb-22 | sort -nr | head -n 1`
if [ ! -e $file ]; then
    echo "$file not found."
    exit
fi
# awk '{ if ($1 < 1000) { print $1 } }' ${file}/maximize-speed_obj.dat > po.dat

i=`wc -l $file/maximize-speed_obj.dat | awk '{print $1}'`
x_min="$((i-100))"
x_max="*"
y_min="*"
y_max="*"

gnuplot -e "
    while (1) {
        set terminal qt 0;
        set xrange [${x_min}:${x_max}];
        set yrange [${y_min}:${y_max}];
        set title \"obj function\";
        plot \"${file}/maximize-speed_obj.dat\" using 0:1 w lp;

        set terminal qt 1;
        set xrange [${x_min}:${x_max}];
        set yrange [${y_min}:${y_max}];
        set title \"eq function\";
        plot \"${file}/maximize-speed_eq.dat\" using 0:1 w lp;
        plot \"${file}/maximize-speed_eq.dat\" using 0:2 w lp;
        plot \"${file}/maximize-speed_eq.dat\" using 0:3 w lp;
        plot \"${file}/maximize-speed_eq.dat\" using 0:4 w lp;
        plot \"${file}/maximize-speed_eq.dat\" using 0:5 w lp;
        plot \"${file}/maximize-speed_eq.dat\" using 0:6 w lp;

        set terminal qt 2;
        set xrange [${x_min}:${x_max}];
        set yrange [${y_min}:${y_max}];
        set title \"ieq function(zmp)\";
        plot \"${file}/maximize-speed_ieq.dat\" using 0:143 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:144 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:145 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:146 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:147 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:148 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:149 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:150 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:151 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:152 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:153 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:154 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:155 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:156 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:157 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:158 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:159 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:160 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:161 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:162 w lp;
        replot \"${file}/maximize-speed_ieq.dat\" using 0:163 w lp;
        pause 30;
    }
"

# 143-191 zmp
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:164 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:165 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:166 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:167 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:168 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:169 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:170 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:171 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:172 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:173 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:174 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:175 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:176 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:177 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:178 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:179 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:180 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:181 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:182 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:183 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:184 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:185 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:186 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:187 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:188 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:189 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:190 w lp;
#        replot \"${file}/maximize-speed_ieq.dat\" using 0:191 w lp;
