SOURCE_SCRIPT="tennis"

source $(rospack find tennis)/scripts/upstart/byobu-utils.bash

create-session

new-window choreonoid "${SOURCE_SCRIPT} && rtmlaunch hrpsys_choreonoid_tutorials jaxon_red_choreonoid.launch TASK:=FOREHAND"
new-window euslist "${SOUECE_SCRIPT} && em"
