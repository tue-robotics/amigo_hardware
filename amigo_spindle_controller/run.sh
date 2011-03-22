#!/bin/bash

rosrun xacro xacro.py `rospack find amigo_spindle_controller`/deployement_configuration.xml.xacro -o `rospack find amigo_spindle_controller`/deployement_configuration.xml

