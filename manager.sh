#!/bin/bash

runvision ()
{
    pkill tune
    pgrep "runvision" || build/runvision &
}

restart_runvision ()
{
    pkill runvision
    build/runvision &
}

tune ()
{
    pkill runvision
    pgrep "tune" || build/tune & 
}

cd /home/patriotrobotics/Documents/vision

while true
do
    mode=$(<mode)
    case "$mode" in
        tune)
            echo "Tuning mode"
            tune
            ;;
        restart)
            if [[ "$lastmode" != "restart" ]]; then
                echo "Restarting code"
                restart_runvision
            else
                runvision
            fi
            ;;
        *)
            echo "Regular vision mode"
            runvision
            ;;
    esac
    lastmode=$mode
    sleep 1
done
