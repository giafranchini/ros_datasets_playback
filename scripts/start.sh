#!/bin/bash

wcorr=68  # manual fix for vertical panels
hcorr=26  # manual fix for horizontal panels


tmps=$(LANG=C xrandr|grep -om1 'current.*,')
tmps=${tmps/,}
tmps=${tmps/current }
echo "screen resolution = $tmps pixels"
wscr=${tmps/ x*}
hscr=${tmps/*x }
wter=$(( (wscr-wcorr)/2 ))
hter=$(( (hscr-hcorr)/2 ))
echo "terminal width  = $wter pixels"
echo "terminal height = $hter pixels"

terminator --borderless --geometry="${wter}x${hter}+0+0" -x /home/user/humble_ws/src/etna/s3li_playback/scripts/shell1.sh &
terminator --borderless --geometry="${wter}x${hter}+0-0" -x /home/user/humble_ws/src/etna/s3li_playback/scripts/shell2.sh &
terminator --borderless --geometry="${wter}x${hter}-0+0" -x /home/user/humble_ws/src/etna/s3li_playback/scripts/shell3.sh &
terminator --borderless --geometry="${wter}x${hter}-0-0" -x /home/user/humble_ws/src/etna/s3li_playback/scripts/shell4.sh &