# blot.vpy — photo → single-stroke line art for Blot (125 × 125 mm).
#
# Usage:
#   vpype -I rmrrf/ui/blot.vpy                      # preview in vpype's viewer
#   PTL_DENSITY=0.4 PTL_GAMMA=1.8 vpype -I rmrrf/ui/blot.vpy
#
# Override the photo with:
#   PTL_INPUT=path/to/photo.jpg vpype -I rmrrf/ui/blot.vpy

script rmrrf/ui/photo_to_lineart.py

# Post-processing (all no-ops on a single line, but safe and future-proof).
linemerge --tolerance 0.5mm
linesort --two-opt
linesimplify --tolerance 0.1mm
filter --min-length 0.5mm

# Lock geometry to the 125x125mm Blot canvas.
pagesize 125mmx125mm
layout --fit-to-margins 0mm 125mmx125mm

# Quick stats, then show in the interactive viewer.
stat
show --pen-width 0.4mm --pen-opacity 0.9

# To write G-code instead of showing, comment out `show` above and uncomment below.
# Launch vpype with the Blot profile config:
#   vpype -c rmrrf/ui/blot_vpype.toml -I rmrrf/ui/blot.vpy
# gwrite --profile blot out.gcode
