(setq *bat-mc* (make-solid-of-revolution '(#f(30 0 0) #f(30 0 -300) #f(15 0 -500) #f(15 0 -729) #f(25 0 -732) #f(25 0 -750))))

(setq *a* (list head-coords1 head-coords2 head-coords3 head-coords4 head-coords5 head-coords6 head-coords7 head-coords8 head-coords9 head-coords10 head-coords11 head-coords12 head-coords13 head-coords14 head-coords15 head-coords16 head-coords17 head-coords18 head-coords19 head-coords20 head-coords21 head-coords22 head-coords23 head-coords24 head-coords25 head-coords26))

(objects (list *bat-mc* *robot* head-coords1 head-coords2 head-coords3 head-coords4 head-coords5 head-coords6 head-coords7 head-coords8 head-coords9 head-coords10 head-coords11 head-coords12 head-coords13 head-coords14 head-coords15 head-coords16 head-coords17 head-coords18 head-coords19 head-coords20 head-coords21 head-coords22 head-coords23 head-coords24 head-coords25 head-coords26))

(progn
 (send *bat-mc* :move-to head-coords1 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords2 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords3 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords4 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords5 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords6 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords7 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords8 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords9 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords10 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords11 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords12 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords13 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords14 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords15 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords16 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords17 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords18 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords19 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords20 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords21 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords22 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords23 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords24 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords25 :world) (send *irtviewer* :draw-objects)
 (unix:usleep 500000)
 (send *bat-mc* :move-to head-coords26 :world) (send *irtviewer* :draw-objects)
 )


(progn
  (setq head-coords1 (make-cascoords :pos (float-vector -13.1720006466 267.254203558 1807.6312542) :rpy (list  0.275387478673 -0.633074786359 -0.641450334361)))
  (setq head-coords2 (make-cascoords :pos (float-vector -17.0811247081 235.83970964 1784.05845165) :rpy (list  0.320838235974 -0.67191989189 -0.629690529439)))
  (setq head-coords3 (make-cascoords :pos (float-vector -17.446860671 194.210097194 1758.86797905) :rpy (list  0.383071855072 -0.708205646388 -0.615304005902)))
  (setq head-coords4 (make-cascoords :pos (float-vector -13.3680356666 140.978008509 1732.2820425) :rpy (list  0.443157811144 -0.754733810476 -0.583390773565)))
  (setq head-coords5 (make-cascoords :pos (float-vector -1.78297038656 74.6671557426 1702.70609856) :rpy (list  0.512222131466 -0.808858504939 -0.541117594662)))
  (setq head-coords6 (make-cascoords :pos (float-vector 21.397318691 -5.01893134788 1668.11478138) :rpy (list  0.61655428796 -0.865174256593 -0.508181374626)))
  (setq head-coords7 (make-cascoords :pos (float-vector 63.3072257042 -98.8735258579 1625.80192089) :rpy (list  0.69825187964 -0.941849885776 -0.432789207043)))
  (setq head-coords8 (make-cascoords :pos (float-vector 129.671603441 -200.776562095 1573.71485233) :rpy (list  0.79145208247 -1.0320392008 -0.348401531949)))
  (setq head-coords9 (make-cascoords :pos (float-vector 229.506000876 -303.824156523 1508.11445713) :rpy (list  0.90936982733 -1.13370488653 -0.264087628197)))
  (setq head-coords10 (make-cascoords :pos (float-vector 367.8175807 -395.170032978 1429.27718163) :rpy (list  1.01770871823 -1.25136503763 -0.150106310408)))
  (setq head-coords11 (make-cascoords :pos (float-vector 546.349585056 -457.409381866 1337.42976189) :rpy (list  1.21522561358 -1.37471484984 -0.100830794766)))
  (setq head-coords12 (make-cascoords :pos (float-vector 760.990083218 -472.341358662 1234.9666357) :rpy (list  0.796779241769 -1.51139299497 0.583358798246)))
  (setq head-coords13 (make-cascoords :pos (float-vector 1002.68912315 -416.932165623 1138.13769817) :rpy (list  -0.861018418183 -1.4672498641 2.54924523673)))
  (setq head-coords14 (make-cascoords :pos (float-vector 1232.21588135 -281.469643116 1048.94340038) :rpy (list  -1.18435964308  -1.34651583275 -3.10780490592)))
  (setq head-coords15 (make-cascoords :pos (float-vector 1432.1937561 -69.9869841337 988.833665848) :rpy (list  -0.876401020843 -1.25027544201 -3.07051879672)))
  (setq head-coords16 (make-cascoords :pos (float-vector 1631.42096996 494.009613991 977.934479713) :rpy (list  -0.0261591084483 -1.21539745382 3.08874196798)))
  (setq head-coords17 (make-cascoords :pos (float-vector 1608.13558102 784.642219543 1024.18982983) :rpy (list  0.805357175991 -1.22772113659 2.61459477598)))
  (setq head-coords18 (make-cascoords :pos (float-vector 926.572024822 1429.24535275 1323.8594532) :rpy (list  2.95174114141 0.269476189972 1.43712147035)))
  (setq head-coords19 (make-cascoords :pos (float-vector 728.549897671 1436.9314909 1386.38305664) :rpy (list  -3.12459694668 0.365423131426 1.35872143085)))
  (setq head-coords20 (make-cascoords :pos (float-vector 388.344019651 1343.87397766 1486.92059517) :rpy (list  -2.71749580885 0.456277393681 1.26912928619)))
  (setq head-coords21 (make-cascoords :pos (float-vector 258.82422924 1259.38844681 1533.3122015) :rpy (list  -2.63384498813 0.65818172551 1.162365583233)))
  (setq head-coords22 (make-cascoords :pos (float-vector 151.946201921 1162.41490841 1569.12922859) :rpy (list  -2.54358932669 0.795420398195 1.07291277677)))
  (setq head-coords23 (make-cascoords :pos (float-vector -55.2035421133 776.472985744 1674.5454073) :rpy (list  -3.11536105571 1.16993290635 -0.0215554655146)))
  (setq head-coords24 (make-cascoords :pos (float-vector -66.8284744024 689.833700657 1696.53260708) :rpy (list  3.03760460478 1.1453549604 -0.263471978777)))
  (setq head-coords25 (make-cascoords :pos (float-vector -65.2137026191 610.385656357 1715.82877636) :rpy (list  3.01976411051 1.12870354818 -0.382057383999)))
  (setq head-coords26 (make-cascoords :pos (float-vector -57.6804056764 541.133761406 1736.78636551) :rpy (list  3.02127493926 1.0913489712 -0.472422327621)))
  )



(mapcar #'(lambda (coord) (send coord :transform (send (send *jaxon* :worldcoords) :inverse-transformation) :world)) *a*)
(setq jaxon-ground (make-cascoords :coords (send *jaxon* :copy-worldcoords)))
(setf (elt (send jaxon-ground :worldpos) 2) 0)

;; (set-init-pos -100 500 -60)


;; pose list
31.irteusgl$ set-mocap-pose 100 :pose 1
(:angle-vector #f(4.97916 -16.8192 -33.3986 66.8237 -30.9193 17.5677 3.50378 17.3505 -22.9971 66.8275 -41.3174 -16.6329 -4.90782 -0.517035 -8.68388 31.0 0.0 2.59954 -71.5395 -41.7866 -1.03801 -103.383 -47.658 2.18008 59.9868 -32.6006 -83.2466 15.8304 -50.6256 -72.12 42.1057 10.1767 22.536) :root-coords #<coordinates #Xb091260  17.301 -5.475 -158.201 / -0.074 -0.043 -0.01> :time 5000)
32.irteusgl$ set-mocap-pose 110 :cog 0.54
(:angle-vector #f(4.86745 -14.952 -33.8852 68.6107 -32.4193 15.4288 3.49917 19.1219 -21.1535 63.8219 -40.31 -18.6717 -5.54977 -1.06779 -11.3787 31.0 0.0 3.32947 -71.5541 -47.2291 0.511056 -102.578 -55.538 0.167456 59.996 -31.7138 -83.4492 15.83 -50.1297 -73.2768 45.0982 8.10104 26.2338) :root-coords #<coordinates #X85e4f20  21.603 -27.401 -156.773 / -0.074 -0.039 -0.005> :time 5000)
33.irteusgl$ set-mocap-pose 120 :cog 0.56
(:angle-vector #f(4.844 -14.2209 -33.9847 69.4427 -33.2427 14.5698 3.52512 19.8211 -20.2584 62.598 -40.0569 -19.497 -5.04344 -1.46283 -13.3965 31.0 0.0 4.42207 -70.7995 -51.1913 0.057929 -102.258 -60.7 -0.475544 59.9954 -31.4116 -83.8384 15.8301 -49.8564 -73.6451 47.04 6.42494 28.5699) :root-coords #<coordinates #Xd151cc8  24.935 -35.968 -156.893 / -0.075 -0.038 -0.003> :time 5000)
34.irteusgl$ set-mocap-pose 140 :cog 0.6
(:angle-vector #f(4.77951 -12.9156 -34.7528 72.0497 -35.3055 12.9467 3.57982 21.2318 -19.0668 61.1982 -40.049 -21.221 -3.60578 -2.06635 -15.7841 31.0 0.0 4.05475 -70.3694 -54.6287 -1.56753 -104.563 -66.5638 0.980734 59.7359 -30.4026 -84.1452 15.83 -49.5972 -76.3594 48.2413 3.50525 28.5705) :root-coords #<coordinates #X161793d8  30.175 -51.757 -161.556 / -0.076 -0.034 0.002> :time 5000)
35.irteusgl$ set-mocap-pose 150 :cog 0.62
(:angle-vector #f(4.73904 -12.1099 -35.2758 73.6404 -36.4449 11.9777 3.57365 22.103 -18.4901 60.5287 -40.0129 -22.2543 -3.23437 -2.3145 -16.7674 31.0 0.0 4.70365 -69.8932 -57.312 -2.76101 -105.732 -71.0796 2.69985 59.2248 -29.1101 -84.1762 16.1631 -49.2045 -78.4196 49.3039 0.936938 28.4716) :root-coords #<coordinates #X16008560  32.91 -61.46 -164.979 / -0.076 -0.032 0.005> :time 5000)
36.irteusgl$ set-mocap-pose 160 :cog 0.64
(:angle-vector #f(4.68615 -11.2865 -35.7393 75.1398 -37.5487 10.9922 3.55375 22.9669 -17.8765 59.7771 -39.9282 -23.2791 -2.92326 -2.69504 -17.366 31.0 0.0 5.20052 -69.4919 -59.4595 -3.99241 -106.599 -75.9426 4.44276 59.1766 -27.5568 -84.259 16.537 -48.7972 -80.6418 51.0589 -1.79691 29.0593) :root-coords #<coordinates #X1547c050  35.7 -71.154 -168.362 / -0.076 -0.031 0.008> :time 5000)
37.irteusgl$ set-mocap-pose 170 :cog 0.64
(:angle-vector #f(4.7482 -11.2618 -35.8137 75.5711 -37.8753 10.9685 3.59411 23.0672 -17.86 60.0842 -40.2179 -23.379 -2.95886 -2.93949 -19.6174 31.0 0.0 5.12692 -66.9303 -62.5475 -2.54685 -106.817 -79.657 5.44164 53.7519 -30.7608 -84.446 22.9124 -49.2379 -82.6389 53.7111 -4.33128 29.1257) :root-coords #<coordinates #Xc1a8858  37.43 -71.771 -170.007 / -0.077 -0.032 0.008> :time 5000)
38.irteusgl$ set-mocap-pose 170 :cog 0.66
(:angle-vector #f(4.68342 -10.0625 -36.167 76.5307 -38.5282 9.60943 3.55345 24.111 -16.8926 58.2118 -39.3391 -24.5819 -3.81037 -2.9308 -20.3099 31.0 0.0 5.73772 -66.1733 -65.6217 -0.976109 -106.618 -84.1694 6.58963 50.7534 -32.5779 -84.9569 27.9242 -49.3763 -84.6809 57.5224 -6.87772 30.2881) :root-coords #<coordinates #Xae652a0  38.593 -85.099 -170.854 / -0.076 -0.031 0.01> :time 5000)
39.irteusgl$ set-mocap-pose 180 :cog 0.68
(:angle-vector #f(4.63261 -9.03493 -36.5916 77.8754 -39.4983 8.41623 3.5265 25.0835 -16.1126 56.9902 -38.9307 -25.7192 -3.85725 -2.99894 -21.3087 31.0 0.0 5.34908 -65.4723 -68.1086 -0.473893 -108.052 -89.3268 9.06919 47.7893 -33.0558 -85.8184 32.0824 -49.5664 -88.0878 61.0752 -10.2826 31.185) :root-coords #<coordinates #X1534fb10  40.935 -96.614 -173.986 / -0.076 -0.03 0.013> :time 5000)
40.irteusgl$ set-mocap-pose 190 :cog 0.68
(:angle-vector #f(4.64898 -8.95716 -36.7959 78.525 -39.8888 8.33342 3.50422 25.2626 -16.2089 57.4779 -39.2589 -25.9045 -3.83004 -2.99937 -22.3014 31.0 0.0 5.45625 -64.209 -70.2072 -0.182677 -109.276 -91.9283 11.9347 44.0606 -33.7884 -86.5655 36.0567 -49.8766 -90.5899 62.7381 -13.0904 30.9709) :root-coords #<coordinates #X161b2810  42.453 -97.844 -176.502 / -0.077 -0.031 0.013> :time 5000)
41.irteusgl$ set-mocap-pose 200 :cog 0.66
(:angle-vector #f(4.73824 -9.92237 -36.911 78.7193 -39.7986 9.45059 3.48396 24.6262 -17.3466 60.0336 -40.5092 -25.1196 -3.43687 -2.99941 -22.8918 31.0 0.0 6.01686 -62.0789 -71.6273 0.248623 -110.222 -91.7984 15.7019 39.1111 -34.3394 -86.5103 40.1157 -50.0522 -92.5734 62.2317 -15.7755 29.179) :root-coords #<coordinates #X16485a78  43.101 -88.058 -179.392 / -0.077 -0.034 0.011> :time 5000)
42.irteusgl$ set-mocap-pose 210 :cog 0.64
(:angle-vector #f(4.79917 -11.0628 -37.2569 79.2561 -39.7852 10.7329 3.41003 23.9153 -18.8188 63.1955 -41.9997 -24.2716 -2.39302 -2.9983 -22.2451 31.0 0.0 7.44441 -59.6775 -72.0822 0.691262 -111.401 -89.6499 20.4747 33.482 -33.578 -85.8878 44.4853 -49.8971 -95.1496 60.6172 -19.0209 26.4639) :root-coords #<coordinates #X15c94ef0  43.191 -76.633 -184.227 / -0.077 -0.037 0.009> :time 5000)
43.irteusgl$ set-mocap-pose 220 :cog 0.62
(:angle-vector #f(4.80055 -12.5299 -37.7927 80.0148 -39.755 12.3348 3.24246 22.9937 -20.709 67.077 -43.7518 -23.2215 -0.048438 -2.98266 -19.5565 31.0 0.0 10.2597 -56.5221 -70.9456 1.54933 -112.182 -85.0499 25.9488 26.8213 -30.9834 -84.4532 49.4027 -49.5081 -98.1962 58.5772 -22.5614 23.5075) :root-coords #<coordinates #X154715b0  42.505 -61.983 -190.998 / -0.075 -0.042 0.007> :time 5000)
44.irteusgl$ set-mocap-pose 230 :cog 0.6
(:angle-vector #f(4.68981 -14.5793 -39.1926 81.76 -39.7919 14.4824 2.91396 21.8278 -23.7949 72.7617 -46.0731 -21.9663 4.52262 -2.37088 -14.0009 31.0 0.0 16.0445 -50.0999 -66.6819 3.19885 -110.698 -73.6982 31.4891 16.3673 -26.9346 -79.6492 56.3589 -48.2911 -99.5958 54.2901 -27.0658 17.9386) :root-coords #<coordinates #Xb2150d0  38.014 -42.276 -203.817 / -0.07 -0.047 0.005> :time 5000)
45.irteusgl$ set-mocap-pose 240 :cog 0.58
(:angle-vector #f(4.38383 -17.1838 -41.2381 84.4949 -40.2689 17.0736 2.43443 20.4614 -27.6103 79.6006 -48.9436 -20.6202 11.3469 -2.80044 -5.24674 31.0 0.0 24.5193 -37.1048 -57.2641 6.07468 -99.2713 -54.0419 35.7637 -5.02739 -23.079 -68.6921 62.8465 -48.5066 -93.6451 45.877 -30.2778 8.63571) :root-coords #<coordinates #Xa906578  31.27 -18.346 -223.082 / -0.061 -0.05 0.005> :time 5000)
46.irteusgl$ send *jaxon* :larm-shoulder-r :joint-angle
62.8465
47.irteusgl$ send *jaxon* :larm-shoulder-r :joint-angle 0
;; #<rotational-joint #X6c095e8 LARM_JOINT2> :joint-angle(0) violate min-angle(15.8251)
15.8251
48.irteusgl$ send *jaxon* :larm-shoulder-y :joint-angle 
-48.5066
49.irteusgl$ send *jaxon* :larm-shoulder-y :joint-angle -100
-100
50.irteusgl$ send *jaxon* :larm-shoulder-y :joint-angle 100
100
51.irteusgl$ send *jaxon* :larm-shoulder-y :joint-angle 50
50
52.irteusgl$ send *jaxon* :larm-shoulder-y :joint-angle -30
-30
53.irteusgl$ send *jaxon* :larm-wrist-y :joint-angle -0
0
54.irteusgl$ send *jaxon* :larm-wrist-y :joint-angle 45
45
55.irteusgl$ send *jaxon* :larm-shoulder-p :joint-angle 
-68.6921
56.irteusgl$ send *jaxon* :larm-shoulder-p :joint-angle -80
-80
57.irteusgl$ send *jaxon* :larm-shoulder-p :joint-angle -40
-40
58.irteusgl$ set-mocap-pose 240 :cog 0.58
(:angle-vector #f(4.32191 -17.2881 -39.7579 82.2445 -39.4043 17.0649 2.34543 19.7867 -26.4535 77.5593 -47.9781 -20.0607 11.336 -2.98241 -3.52099 31.0 0.0 22.921 -35.5273 -59.6412 6.50369 -98.5364 -53.7862 37.7911 -8.27634 -17.1187 -54.263 41.7822 -26.9997 -93.8943 31.9108 -34.4065 17.3279) :root-coords #<coordinates #X1c0fdee0  34.745 -13.682 -213.215 / -0.06 -0.051 0.007> :time 5000)
59.irteusgl$ set-mocap-pose 250 :cog 0.56
(:angle-vector #f(3.78189 -18.4462 -40.3609 82.0688 -38.4632 17.3391 1.71966 18.5612 -27.4249 77.6297 -46.9579 -19.7237 11.3239 -2.68685 7.19156 31.0 0.0 37.9645 -19.0083 -42.6975 8.47536 -73.491 -52.8604 28.2667 -42.2451 -19.495 -49.7083 52.2149 -35.6155 -84.3949 26.6848 -31.3571 26.0694) :root-coords #<coordinates #X15fad878  27.16 3.353 -211.89 / -0.049 -0.053 0.022> :time 5000)
60.irteusgl$ send *jaxon* :rarm-shoulder-p :joint-angle 
-19.0083
61.irteusgl$ send *jaxon* :rarm-shoulder-p :joint-angle -40
-40
62.irteusgl$ send *jaxon* :rarm-shoulder-p :joint-angle 10
10
63.irteusgl$ send *jaxon* :rarm-shoulder-r :joint-angle 
-42.6975
64.irteusgl$ send *jaxon* :rarm-shoulder-r :joint-angle 0
;; #<rotational-joint #Xcae3780 RARM_JOINT2> :joint-angle(0) violate max-angle(-15.8251)
-15.8251
65.irteusgl$ send *jaxon* :rarm-elbow-p :joint-angle
-73.491
66.irteusgl$ send *jaxon* :rarm-elbow-p :joint-angle -100
-100
67.irteusgl$ send *jaxon* :rarm-wrist-r :joint-angle 
28.2667
68.irteusgl$ send *jaxon* :rarm-wrist-r :joint-angle -20
-20
69.irteusgl$ send *jaxon* :rarm-wrist-r :joint-angle -40
-40
70.irteusgl$ set-mocap-pose 250 :cog 0.56
(:angle-vector #f(3.83272 -17.8689 -39.8715 82.2305 -39.1308 16.7416 1.78042 19.0312 -26.3394 76.5452 -46.9557 -20.2135 11.3201 -2.76411 8.0122 31.0 0.0 37.9508 -17.2588 -15.8455 2.2803 -104.316 -46.3818 -33.2604 -18.0751 -20.7901 -49.131 54.764 -36.355 -85.1676 28.3466 -31.207 25.1347) :root-coords #<coordinates #X1c0ab598  33.117 -2.697 -210.395 / -0.051 -0.053 0.023> :time 5000) ;; seems good
;; #f(-17.5153 -55.4781 16.0305 -49.7462 -32.6772 -6.12728 -15.7892 59.9536) seems very good (larm meet point)
114.irteusgl$ set-mocap-pose 260 :cog 0.52
(:angle-vector #f(6.06409 -30.097 -31.8385 72.3946 -34.8393 21.3938 2.74948 5.08147 -20.7022 69.048 -43.3824 -13.9278 11.3761 -2.99692 16.7796 31.0 0.0 52.3678 -20.2526 -27.9192 2.86761 -71.2344 -103.89 -20.7195 -25.6426 -2.49774 -55.6347 22.6605 -94.9101 -56.6302 7.24618 41.1017 45.1218) :root-coords #<coordinates #X11c91db0  68.093 178.983 -162.743 / -0.068 -0.077 0.159> :time 5000)
115.irteusgl$ set-mocap-pose 265 :cog 0.5
(:angle-vector #f(6.06394 -31.8213 -31.0258 69.4212 -32.1396 22.4 2.48041 2.96028 -21.4233 68.3616 -41.6186 -12.5479 11.3761 1.3688 27.4599 31.0 0.0 58.413 -30.0262 -36.787 -3.90787 -45.7638 -114.97 -23.0254 6.06741 -9.37406 -68.2402 22.0261 -95.8964 -45.4123 34.5276 7.42594 60.0) :root-coords #<coordinates #X14be41a8  59.559 205.336 -153.287 / -0.063 -0.083 0.172> :time 5000)







(set-mocap-pose 260 :pose 4 :rot-axis (list t t t t) :cog 0.6)

(set-mocap-pose 290 :link-list '(:larm :lleg :rleg) :target-coords (list *bat-lhand-coords* *lfoot* *rfoot*) :cog 0.55 :rot-axis (list t t t))

(mapc #'(lambda (i) (set-mocap-pose (+ (* i 10) 100) :rot-axis (list t t t t) :cog 0.6)) (range 200))


;; practice LoadPattern
(send *jaxon* :angle-vector #f(-7.28452 -14.0779 -32.5283 65.61 -31.7946 13.4947 -8.03163 19.1654 -22.1046 57.588 -34.1617 -19.7567 6.63551 -1.01633 -1.9049 10.0 4.0 0.0 0.0 -30.0 0.0 -30.0 0.0 0.0 0.0 -16.7659 -65.4748 30.9058 -68.549 -95.1418 20.8698 13.5902 6.2699)) ;; 240



;; (:angle-vector #f(-0.444384 -20.2809 -27.0878 59.3841 -29.3999 14.7237 -2.0145 11.7477 -13.3894 47.4522 -31.288 -17.3421 6.34905 -2.99532 -1.52489 10.0 4.0 20.0 -20.0 -15.8251 20.0 -65.0 -50.0 -30.0 -5.0 -3.45342 -59.4044 24.7205 -70.7737 -91.1457 22.155 15.0162 -2.32095) :root-coords #<coordinates #Xbe2c920  -11.73 -30.923 904.314 / 0.021 -0.05 0.096> :time 5000) ;; 240

;; (:angle-vector #f(-0.567392 -19.4194 -27.6835 59.9365 -29.5857 14.3815 -2.01999 12.6557 -13.883 47.8983 -31.4372 -17.7254 9.17181 -2.99492 -0.489954 10.0 4.0 20.0 -20.0 -15.8251 20.0 -65.0 -50.0 -30.0 -5.0 2.87446 -55.9676 34.2339 -74.8883 -79.0158 18.88 9.70791 8.71257) :root-coords #<coordinates #Xbde0678  -12.335 -34.378 902.822 / 0.021 -0.046 0.087> :time 5000) ;;250

;; (:angle-vector #f(-0.693066 -18.7674 -27.673 59.3303 -29.2771 14.0949 -1.98805 13.2002 -13.7459 47.0327 -30.9721 -17.8982 11.298 -2.98443 2.18609 10.0 4.0 20.0 -20.0 -15.8251 20.0 -65.0 -50.0 -30.0 -5.0 4.32795 -53.0691 38.6386 -75.6033 -66.7461 16.2406 5.59443 36.9633) :root-coords #<coordinates #X1bb75eb8  -11.856 -36.551 905.086 / 0.022 -0.041 0.081> :time 5000) ;; 260

;; (:angle-vector #f(0.574354 -22.6895 -22.8959 52.8702 -26.5641 14.5723 -1.20554 8.36492 -6.73537 35.5397 -25.6243 -16.531 9.84535 -2.86876 1.08879 10.0 4.0 20.0 -20.0 -15.8251 20.0 -65.0 -50.0 -30.0 -5.0 7.86891 -50.6182 15.8849 -146.284 15.2043 104.475 -19.0485 59.6797) :root-coords #<coordinates #X6a9a4e8  -6.531 -33.14 928.319 / 0.005 -0.056 0.142> :time 5000) ;; 270

;; ;; worldcoords
;; (setq lp1-rc (nth 5 lp1))
;; (setq lp2-rc (nth 5 lp2))
;; (setq lp3-rc (nth 5 lp3))
;; (setq lp4-rc (nth 5 lp4))
;; ;; worldpos-list
;; (setq lp-rc-pos (list (send lp1-rc :worldpos) (send lp2-rc :worldpos) (send lp3-rc :worldpos) (send lp4-rc :worldpos)))
;; ;; worldrot-list
;; (setq lp-rc-rot (list (matrix-log (send lp1-rc :rot)) (matrix-log (send lp2-rc :rot)) (matrix-log (send lp3-rc :rot)) (matrix-log (send lp4-rc :rot))))


(matrix-log <coords :rot >)
(norm *)



;; latest
(load-mocap)
(load-pose)
(set-init-pos -100 450 -60)
(transform-objects)
(pose1)
(jsk)
(reset)
(set-mocap-pose 100 :pose 1 :rot-axis (list t t t t) :cog 0.6)
(send *jaxon* :angle-vector #f(-7.28452 -14.0779 -32.5283 65.61 -31.7946 13.4947 -8.03163 19.1654 -22.1046 57.588 -34.1617 -19.7567 6.63551 -1.01633 -1.9049 10.0 4.0 0.0 0.0 -30.0 0.0 -30.0 0.0 0.0 0.0 -16.7659 -65.4748 30.9058 -68.549 -95.1418 20.8698 13.5902 6.2699)) ;; 240
;; angle-vector rootcoords time
(setq lp1 (set-mocap-pose 240 :link-list '(:larm :lleg :rleg) :target-coords (list *bat-lhand-coords* *lfoot* *rfoot*) :cog 0.6 :rot-axis (list t t t)))
(setq lp2 (set-mocap-pose 250 :link-list '(:larm :lleg :rleg) :target-coords (list *bat-lhand-coords* *lfoot* *rfoot*) :cog 0.6 :rot-axis (list t t t)))
(setq lp3 (set-mocap-pose 260 :link-list '(:larm :lleg :rleg) :target-coords (list *bat-lhand-coords* *lfoot* *rfoot*) :cog 0.6 :rot-axis (list t t t)))
(setq lp4 (set-mocap-pose 267 :link-list '(:larm :lleg :rleg) :target-coords (list *bat-lhand-coords* *lfoot* *rfoot*) :cog 0.6 :rot-axis (list t t t)))

(make-loadpattern (list lp1 lp2 lp3 lp4) :tm-list (list 3 3 3) :dt 1 :lp-file "../config/swing")
(send *ri* :load-pattern "/home/terasawa/ros/hydro/src/jsk-ros-pkg/euslib/demo/terasawa/catkin_ws/src/tennis/config/swing" 10)
