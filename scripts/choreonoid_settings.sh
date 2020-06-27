#!/bin/bash

MOTION="forehand"

if [ $# -ge 1 ]; then
    MOTION=$1
fi
if [ $MOTION = "forehand" ]; then
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC_WITH_RACKET.wrl $(rospack find jvrc_models)/JAXON_JVRC/RARM_LINK7_JVRC.wrl
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC_WITH_RACKET.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/RARM_LINK7_JVRC.wrl
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/.
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/.
    cp $(rospack find tennis)/scripts/CnoidPyUtil.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
    cp $(rospack find tennis)/scripts/jaxon_red_setup.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
elif [ $MOTION = "punch" ]; then
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/.
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/.
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/.
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/.
    cp $(rospack find tennis)/config/JAXON_RED_PUNCH.cnoid.in $(rospack find hrpsys_choreonoid_tutorials)/config/.
    cp $(rospack find tennis)/scripts/jaxon_red_setup.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
elif [ $MOTION = "kick" ]; then
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/.
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/.
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/.
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/.
    cp $(rospack find tennis)/config/JAXON_RED_KICK.cnoid.in $(rospack find hrpsys_choreonoid_tutorials)/config/.
    cp $(rospack find tennis)/scripts/jaxon_red_setup.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
elif [ $MOTION = "smash" ]; then
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC_WITH_RACKET.wrl $(rospack find jvrc_models)/JAXON_JVRC/RARM_LINK7_JVRC.wrl
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC_WITH_RACKET.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/RARM_LINK7_JVRC.wrl
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/.
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/.
    cp $(rospack find tennis)/config/JAXON_RED_SMASH.cnoid.in $(rospack find hrpsys_choreonoid_tutorials)/config/.
    cp $(rospack find tennis)/scripts/CnoidPyUtil.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
    cp $(rospack find tennis)/scripts/moving_ball.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
    cp $(rospack find tennis)/scripts/jaxon_red_setup.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
elif [ $MOTION = "batting" ]; then
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/.
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/.
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC_WITH_BAT.wrl $(rospack find jvrc_models)/JAXON_JVRC/LARM_LINK7_JVRC.wrl
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/.
    cp $(rospack find tennis)/config/JAXON_RED_BATTING.cnoid.in $(rospack find hrpsys_choreonoid_tutorials)/config/.
    cp $(rospack find tennis)/scripts/jaxon_red_setup.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
elif [ $MOTION = "forehand-step" ]; then
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC_WITH_RACKET.wrl $(rospack find jvrc_models)/JAXON_JVRC/RARM_LINK7_JVRC.wrl
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC_WITH_RACKET.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/RARM_LINK7_JVRC.wrl
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/.
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/.
    cp $(rospack find tennis)/config/JAXON_RED_FOREHAND-STEP.cnoid.in $(rospack find hrpsys_choreonoid_tutorials)/config/.
    cp $(rospack find tennis)/scripts/CnoidPyUtil.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
    cp $(rospack find tennis)/scripts/moving_ball.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
    cp $(rospack find tennis)/scripts/jaxon_red_setup.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
else
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/.
    cp $(rospack find tennis)/model/RARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/.
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/.
    cp $(rospack find tennis)/model/LARM_LINK7_JVRC.wrl $(rospack find jvrc_models)/JAXON_JVRC/convex_hull/.
    cp $(rospack find tennis)/scripts/jaxon_red_setup.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.
fi
cp $(rospack find tennis)/scripts/jaxon_red_setup.py $(rospack find hrpsys_choreonoid_tutorials)/scripts/.

echo "please add cnoid generation script in hrpsys_choreonoid_tutorials/CMakeLists.txt"
